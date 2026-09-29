"""Regression sim for the Yaskawa_Encoders FPGA module (src/yaskawa_encoders.py).

Replays real encoder frames into the module and checks what the controller would
read back over the BRAM register interface.

usage (from controller-firmware/python):
    ../../.venv/bin/python sim/yaskawa_encoders_sim.py            # all tests, ~1 min
    ../../.venv/bin/python sim/yaskawa_encoders_sim.py --quick    # fewer frames
    ../../.venv/bin/python sim/yaskawa_encoders_sim.py -k request # tests whose name contains 'request'
"""
import argparse
import contextlib
import io
import os
import sys
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from amaranth.sim import Simulator  # noqa: E402

from yaskawa_protocol import (  # noqa: E402
    ABSOLUTE_EXAMPLE_FRAME, FLAG, REFERENCE_REQUEST, REQUEST_PAYLOAD_FFFF, RESPONSE_PREFIX,
    crc16_x25, decode, hdlc_unstuff, line_edges, stuff,
)

with contextlib.redirect_stdout(io.StringIO()):     # silence register-map generation logs
    from yaskawa_encoders import Yaskawa_Encoders  # noqa: E402

FIXTURES = os.path.join(HERE, "data", "yaskawa_incremental_frames.txt")
RESPONSE_DELAY_NS = 17_000      # measured gap between end of request and start of reply
SKEWS = [1.0, 0.98, 1.02]       # encoder clock error applied in rotation across frames


def load_frames():
    with open(FIXTURES) as f:
        return [line.split() for line in f if line.strip() and not line.startswith("#")]


def make_dut(*args, **kwargs):
    with contextlib.redirect_stdout(io.StringIO()):
        return Yaskawa_Encoders(*args, **kwargs)


class Bus:
    """Drives the module's BRAM port and serial lines from a testbench."""

    def __init__(self, dut, ctx):
        self.dut, self.ctx = dut, ctx

    async def write(self, address, data):
        self.ctx.set(self.dut.bram_address, address)
        self.ctx.set(self.dut.bram_write_data, data)
        self.ctx.set(self.dut.bram_write_enable, 1)
        await self.ctx.tick("sync_100")
        self.ctx.set(self.dut.bram_write_enable, 0)

    async def read(self, address):
        self.ctx.set(self.dut.bram_address, address)
        await self.ctx.tick("sync_100").repeat(2)
        return self.ctx.get(self.dut.bram_read_data)

    async def read_encoder(self, index, register):
        rm = self.dut.rm
        return await self.read(index * rm.encoder.alignment + getattr(rm.encoder, register).address_offset)

    async def status(self, index):
        word = await self.read_encoder(index, "status")
        regs = self.dut.rm.encoder.status
        return {name: (word >> getattr(regs, name).starting_bit) & 1
                for name in ("battery_fail", "unindexed", "no_response", "crc_fail", "done")}

    async def request(self, payload=0xFFFF):
        """Send one request and wait until the line is released."""
        await self.write(self.dut.rm.request_packet.address_offset, payload)
        await self.write(self.dut.rm.trigger.address_offset, 1)
        for _ in range(1000):
            if self.ctx.get(self.dut.tx_enable[0]):
                break
            await self.ctx.tick("sync_200")
        else:
            raise AssertionError("tx_enable never asserted")
        while self.ctx.get(self.dut.tx_enable[0]):
            await self.ctx.tick("sync_200")

    async def respond(self, frames, delay_ns=RESPONSE_DELAY_NS, skew=1.0):
        """Drive each port's rx with its frame ({port: on-wire bits}), all starting together."""
        events = []
        for port, bits in frames.items():
            edges, _ = line_edges(bits, symbol_ns=125.0 * skew, start_ns=delay_ns)
            events += [(round(t / 5), port, level) for t, level in edges]   # 5 ns sync_200 grid
        now = 0
        for tick, port, level in sorted(events):
            if tick > now:
                await self.ctx.tick("sync_200").repeat(tick - now)
                now = tick
            self.ctx.set(self.dut.rx[port], level)
        for port in frames:
            await self.ctx.tick("sync_200").repeat(50)
            self.ctx.set(self.dut.rx[port], 1)                       # idle
        await self.ctx.tick("sync_100").repeat(100)


def run(dut, bench):
    sim = Simulator(dut)
    sim.add_clock(1 / 100e6, domain="sync_100")
    sim.add_clock(1 / 200e6, domain="sync_200")

    async def wrapped(ctx):
        ctx.set(dut.rx, (1 << len(dut.rx)) - 1)
        await bench(Bus(dut, ctx))

    sim.add_testbench(wrapped)
    sim.run()


def check(errors, label, got, expected):
    if got != expected:
        errors.append(f"{label}: got {got}, expected {expected}")


# ── tests ──────────────────────────────────────────────────────────────────────

def capture_request(dut):
    """Return the request frame as on-wire bits (Manchester decoded while tx_enable is high)."""
    samples = []

    async def bench(bus):
        await bus.write(dut.rm.request_packet.address_offset, 0xFFFF)
        await bus.write(dut.rm.trigger.address_offset, 1)
        for _ in range(2 * 25 * 110):
            await bus.ctx.tick("sync_200")
            samples.append((bus.ctx.get(dut.tx_enable[0]), bus.ctx.get(dut.tx[0])))

    run(dut, bench)
    enabled = [i for i, (en, _) in enumerate(samples) if en]
    symbols = [samples[i][1] for i in range(enabled[0] + 12, enabled[-1], 25)]
    pairs = [symbols[i:i + 2] for i in range(0, len(symbols) - 1, 2)]
    if any(a == b for a, b in pairs):
        raise AssertionError(f"not Manchester encoded: {symbols}")
    return "".join("1" if p == [1, 0] else "0" for p in pairs)


def test_request_frame_default_unchanged(quick):
    """Default request is what absolute encoders have been getting: 15 preamble bits on the wire."""
    got = capture_request(make_dut(1, [{"mode": "17bit"}]))
    expected = "010101010101010" + FLAG + REQUEST_PAYLOAD_FFFF + FLAG
    return [] if got == expected else [f"got      {got}\nexpected {expected}"]


def test_request_frame_reference_preamble(quick):
    """preamble_bits=25 reproduces the request proven on the incremental encoder, bit for bit."""
    got = capture_request(make_dut(1, [{"mode": "17bit_incremental"}], preamble_bits=25))
    return [] if got == REFERENCE_REQUEST else [f"got      {got}\nexpected {REFERENCE_REQUEST}"]


def test_incremental_frames(quick):
    """Every fixture frame decodes to the reference values, with ±2% encoder clock skew."""
    frames = load_frames()
    if quick:
        frames = frames[::6]
    dut = make_dut(1, [{"mode": "17bit_incremental"}])
    errors = []

    async def bench(bus):
        for n, (capture, bits) in enumerate(frames):
            ref = decode(bits)
            skew = SKEWS[n % len(SKEWS)]
            await bus.request()
            await bus.respond({0: RESPONSE_PREFIX + bits}, skew=skew)
            label = f"{capture} frame {n} (skew {skew})"
            check(errors, f"{label} singleturn", (await bus.read_encoder(0, "singleturn_count")) >> 15, ref["single_turn"])
            check(errors, f"{label} multiturn", await bus.read_encoder(0, "multiturn_count"), ref["mt4"])
            check(errors, f"{label} commutation", await bus.read_encoder(0, "commutation_count"),
                  (ref["single_turn"] << 15) >> 16)     # copied straight to the drive by the DMA
            check(errors, f"{label} timestamp_fast", await bus.read_encoder(0, "timestamp_fast"), ref["ts_fast"])
            info = await bus.read_encoder(0, "incremental_info")
            regs = dut.rm.encoder.incremental_info
            check(errors, f"{label} hall", (info >> regs.hall.starting_bit) & 0b111, ref["hall"])
            check(errors, f"{label} rot3", (info >> regs.rot3.starting_bit) & 0b111, ref["rot3"])
            check(errors, f"{label} status", await bus.status(0),
                  {"battery_fail": ref["battery_fail"], "unindexed": ref["index_not_found"],
                   "no_response": 0, "crc_fail": 0, "done": 1})
            for word in range(3):
                check(errors, f"{label} raw_payload_{word}", await bus.read_encoder(0, f"raw_payload_{word}"),
                      (ref["payload96"] >> (32 * word)) & 0xFFFFFFFF)

    run(dut, bench)
    return errors


def test_absolute_modes_unchanged_and_mixed(quick):
    """Absolute 16/17-bit decode is unchanged, and an incremental port works alongside them."""
    incremental_bits = load_frames()[25][1]
    dut = make_dut(3, [{"mode": "16bit"}, {"mode": "17bit"}, {"mode": "17bit_incremental"}])
    absolute, incremental = decode(ABSOLUTE_EXAMPLE_FRAME), decode(incremental_bits)
    errors = []

    async def bench(bus):
        await bus.request()
        await bus.respond({0: ABSOLUTE_EXAMPLE_FRAME, 1: ABSOLUTE_EXAMPLE_FRAME,
                           2: RESPONSE_PREFIX + incremental_bits})
        check(errors, "16bit singleturn", (await bus.read_encoder(0, "singleturn_count")) >> 16, absolute["16bit_single_turn"])
        check(errors, "16bit multiturn", await bus.read_encoder(0, "multiturn_count"), absolute["16bit_multi_turn"])
        check(errors, "17bit singleturn", (await bus.read_encoder(1, "singleturn_count")) >> 15, absolute["17bit_single_turn"])
        check(errors, "17bit multiturn", await bus.read_encoder(1, "multiturn_count"), absolute["17bit_multi_turn"])
        check(errors, "17bit commutation", await bus.read_encoder(1, "commutation_count"),
              (absolute["17bit_single_turn"] << 15) >> 16)
        check(errors, "incremental singleturn", (await bus.read_encoder(2, "singleturn_count")) >> 15, incremental["single_turn"])
        check(errors, "incremental multiturn", await bus.read_encoder(2, "multiturn_count"), incremental["mt4"])
        for port in range(3):
            status = await bus.status(port)
            check(errors, f"port {port} crc_fail/done", (status["crc_fail"], status["done"]), (0, 1))

    run(dut, bench)
    return errors


def test_crc_error_keeps_last_good_values(quick):
    """A corrupted frame raises crc_fail and leaves the last good position in the registers."""
    frames = load_frames()
    good, other = frames[25][1], frames[30][1]
    bad = list(RESPONSE_PREFIX + other)
    bad[len(RESPONSE_PREFIX) + 40] = "1" if bad[len(RESPONSE_PREFIX) + 40] == "0" else "0"
    dut = make_dut(1, [{"mode": "17bit_incremental"}])
    errors = []

    async def bench(bus):
        await bus.request()
        await bus.respond({0: RESPONSE_PREFIX + good})
        await bus.request()
        await bus.respond({0: "".join(bad)})
        check(errors, "crc_fail", (await bus.status(0))["crc_fail"], 1)
        check(errors, "singleturn held", (await bus.read_encoder(0, "singleturn_count")) >> 15, decode(good)["single_turn"])
        check(errors, "raw payload held", await bus.read_encoder(0, "raw_payload_1"), (decode(good)["payload96"] >> 32) & 0xFFFFFFFF)
        await bus.request()
        await bus.respond({0: RESPONSE_PREFIX + other})
        check(errors, "crc_fail cleared", (await bus.status(0))["crc_fail"], 0)
        check(errors, "singleturn updated", (await bus.read_encoder(0, "singleturn_count")) >> 15, decode(other)["single_turn"])

    run(dut, bench)
    return errors


def test_no_response_times_out(quick):
    """A silent encoder reports no_response and the receiver stops listening until the next request."""
    bits = load_frames()[25][1]
    dut = make_dut(1, [{"mode": "17bit_incremental"}])
    errors = []

    async def bench(bus):
        await bus.request()
        await bus.ctx.tick("sync_100").repeat(15_000)               # 150 us of silence
        status = await bus.status(0)
        check(errors, "no_response after silence", (status["no_response"], status["done"]), (1, 0))
        # a stray edge long after the request must not be mistaken for a reply
        bus.ctx.set(dut.rx[0], 0)
        await bus.ctx.tick("sync_200").repeat(10)
        bus.ctx.set(dut.rx[0], 1)
        await bus.ctx.tick("sync_100").repeat(100)
        check(errors, "no_response after stray edge", (await bus.status(0))["no_response"], 1)
        # and the next exchange still works
        await bus.request()
        await bus.respond({0: RESPONSE_PREFIX + bits})
        status = await bus.status(0)
        check(errors, "recovered", (status["no_response"], status["crc_fail"], status["done"]), (0, 0, 1))
        check(errors, "recovered singleturn", (await bus.read_encoder(0, "singleturn_count")) >> 15, decode(bits)["single_turn"])

    run(dut, bench)
    return errors


def test_garbled_line_does_not_hang(quick):
    """A line that keeps toggling without ever forming a frame (e.g. an RS485 receiver oscillating on an open
    bus) must not hang the receiver: the exchange ends as a failed frame, and later requests work."""
    bits = load_frames()[25][1]
    dut = make_dut(1, [{"mode": "17bit_incremental"}])
    errors = []

    async def bench(bus):
        await bus.request()
        await bus.respond({0: RESPONSE_PREFIX + bits})
        check(errors, "first frame good", (await bus.status(0))["done"], 1)
        # 600 us of Manchester ones: edges every half bit, never a flag, and bit stuffing stops the count
        await bus.request()
        await bus.respond({0: "1" * 2400})
        status = await bus.status(0)
        check(errors, "garbled: failed frame", (status["done"], status["crc_fail"], status["no_response"]), (0, 1, 0))
        # the line goes quiet again and the next exchange works
        await bus.request()
        await bus.respond({0: RESPONSE_PREFIX + bits})
        status = await bus.status(0)
        check(errors, "recovered", (status["done"], status["crc_fail"], status["no_response"]), (1, 0, 0))

    run(dut, bench)
    return errors


def frame_with_index_count(payload, value):
    """On-wire reply: payload (112 bits as a list) with index_count (bits 89:96) set, CRC recomputed, stuffed."""
    p = payload[:]
    for k in range(7):
        p[89 + k] = (value >> k) & 1
    crc = crc16_x25(p)
    for k in range(16):
        p[96 + k] = (crc >> k) & 1
    return "1111110" + stuff("".join(str(b) for b in p)) + "01111110"


def test_every_index_count_decodes(quick):
    """Every index_count value decodes. -2 and +62 put a stuffed 0 just before payload bit 95, which used to end
    the CRC early (found on the bench 2026-09-26: the link froze for the whole turn the count read -2)."""
    payload = hdlc_unstuff(load_frames()[25][1])[:112]
    values = [0, 1, 62, 63, 64, 125, 126, 127] if quick else list(range(128))
    dut = make_dut(1, [{"mode": "17bit_incremental"}])
    errors = []

    async def bench(bus):
        for v in values:
            wire = frame_with_index_count(payload, v)
            await bus.request()
            await bus.respond({0: RESPONSE_PREFIX + wire})
            status = await bus.status(0)
            check(errors, f"index_count {v} status", (status["done"], status["crc_fail"]), (1, 0))
            check(errors, f"index_count {v} raw", (await bus.read_encoder(0, "raw_payload_2")) >> (89 - 64),
                  v)

    run(dut, bench)
    return errors


def test_unindexed_is_bit_6(quick):
    """Incremental ports report unindexed from payload bit 6 (set until the index is first crossed); bit 0 is
    always 1 on the UTSIH-B17CK and is not the index flag."""
    payload = hdlc_unstuff(load_frames()[25][1])[:112]
    dut = make_dut(1, [{"mode": "17bit_incremental"}])
    errors = []

    async def bench(bus):
        for bit6 in (1, 0, 1):
            p = payload[:]
            p[0], p[6] = 1, bit6
            crc = crc16_x25(p)
            for k in range(16):
                p[96 + k] = (crc >> k) & 1
            await bus.request()
            await bus.respond({0: RESPONSE_PREFIX + "1111110" + stuff("".join(str(b) for b in p)) + "01111110"})
            status = await bus.status(0)
            check(errors, f"bit 6 = {bit6}", (status["done"], status["crc_fail"], status["unindexed"]), (1, 0, bit6))

    run(dut, bench)
    return errors


def test_dense_ones_decode(quick):
    """Random payloads that are mostly ones, so stuffed bits land everywhere, all decode."""
    import random
    rng = random.Random(7)
    frames = []
    for _ in range(40 if quick else 400):
        p = [1 if rng.random() < 0.85 else 0 for _ in range(96)] + [0] * 16
        crc = crc16_x25(p)
        for k in range(16):
            p[96 + k] = (crc >> k) & 1
        frames.append((p, "1111110" + stuff("".join(str(b) for b in p)) + "01111110"))
    dut = make_dut(1, [{"mode": "17bit_incremental"}])
    errors = []

    async def bench(bus):
        for n, (p, wire) in enumerate(frames):
            await bus.request()
            await bus.respond({0: RESPONSE_PREFIX + wire})
            status = await bus.status(0)
            words = [await bus.read_encoder(0, f"raw_payload_{w}") for w in range(3)]
            got = [(words[b // 32] >> (b % 32)) & 1 for b in range(96)]
            check(errors, f"dense frame {n} status", (status["done"], status["crc_fail"]), (1, 0))
            check(errors, f"dense frame {n} payload", got, p[:96])

    run(dut, bench)
    return errors


def test_driver_settings(quick):
    """The software driver learns the port count, modes and preamble from fpga_config.json."""
    modes = [{"mode": "17bit"}, {"mode": "17bit_incremental"}]
    settings = make_dut(2, modes, preamble_bits=25).rm.driver_settings
    expected = {"encoder_count": 2, "encoder_modes": ["17bit", "17bit_incremental"], "preamble_bits": 25}
    return [] if settings == expected else [f"got {settings}, expected {expected}"]


TESTS = [v for k, v in list(globals().items()) if k.startswith("test_")]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--quick", action="store_true", help="replay a subset of the fixture frames")
    parser.add_argument("-k", default="", help="only run tests whose name contains this")
    args = parser.parse_args()

    failed = 0
    for test in TESTS:
        if args.k not in test.__name__:
            continue
        try:
            errors = test(args.quick)
        except Exception:
            errors = [traceback.format_exc(limit=3).strip()]
        failed += bool(errors)
        print(f"{'PASS' if not errors else 'FAIL'}  {test.__name__}")
        for e in errors[:10]:
            print("      " + e.replace("\n", "\n      "))
        if len(errors) > 10:
            print(f"      ... {len(errors) - 10} more")
    print(f"\n{failed} failed" if failed else "\nall passed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
