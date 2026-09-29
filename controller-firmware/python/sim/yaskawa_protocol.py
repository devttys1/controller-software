"""Yaskawa serial encoder protocol helpers shared by the Yaskawa sim scripts.

Reference decoder, line encoder and capture reader. The field map and decode
rules are ported from the proven STM32 implementation in the
yaskawa-incremental-encoder project (Core/Src/yaskawa_decode.c), which was
validated against >22000 captured frames.

Frame format (both absolute and incremental encoders):
    4 Mbit/s data, Manchester encoded (125 ns symbols, 1 = HIGH->LOW),
    HDLC framing (0x7E flags, zero inserted after five 1s),
    112 payload bits LSB-first, CRC-16/X.25 over payload bits [0:96].
"""

# Request frame proven on the incremental encoder (yaskawa-incremental-encoder/Core/Src/yaskawa_tx.c)
REFERENCE_PREAMBLE = "0101010101010101010101010"            # 25 bits
FLAG = "01111110"
REQUEST_PAYLOAD_FFFF = "1111101111101111101"                # 0xFFFF after bit stuffing
REFERENCE_REQUEST = REFERENCE_PREAMBLE + FLAG + REQUEST_PAYLOAD_FFFF + FLAG

# Encoders reply with this in front of the start flag. The analyser captures in
# data/ drop it (and the flag's leading 0), so it is prepended when replaying them.
RESPONSE_PREFIX = "0101010101010101" + "0"

# Full on-wire response from an absolute encoder, taken from the bench that used
# to live at the bottom of yaskawa_encoders.py.
ABSOLUTE_EXAMPLE_FRAME = (
    "0101010101010101001111110101000000010010011101000000100111110101110011100000000000011100"
    "01000010001111101111101111101111100000100011101100101001111110"
)

ENCODER_RES_17BIT = 1 << 17

# Incremental (UTSIH-B17CK) payload bit offsets, LSB first. Decoded on the bench 2026-09-26 from the
# raw payload (see Documentation/yaskawa-incremental-encoder.md, "Payload map"); bits not listed were
# constant: 1-5 and 7 are 0.
INCREMENTAL_FIELDS = {
    "bit0": (0, 1),             # always 1 on this encoder, power-ups and index crossings included; NOT the
                                # index flag (index_not_found is). The absolute modes' status.unindexed reads it
    "battery_fail": (2, 1),
    "index_not_found": (6, 1),  # 1 from power-up until the shaft first passes the index mark
    "comp_valid_n": (7, 1),     # always 0 so far
    "ready": (8, 8),            # 0x00 for ~0.1-0.2 s after power-up, then 0xFF
    "ts_slow": (16, 8),         # steps about every 216 ms
    "ts_fast": (24, 16),        # advances 6001 per 1 ms frame: a ~6 MHz up-counter (or an alias of one)
    "position_age": (40, 16),   # ts_fast ticks (6 MHz) since the encoder's internal position last changed,
                                # when the frame was built: ts_fast - position_age is when the reported position
                                # was reached. Wraps at 65536 (10.9 ms). Once called raw40
    "index_pulse": (56, 1),     # 1 while the shaft is within ~0.3 degrees of the index mark
    "hall": (57, 3),
    "single_turn": (60, 17),    # zero moves at every power-up
    "rot3": (77, 3),            # single_turn wraps since power-up (steps at single_turn 0)
    "index_position": (80, 9),  # single_turn of the index mark >> 8, latched at the first crossing, 0 before
    "index_count": (89, 7),     # signed index crossings since power-up: +1 forward, -1 back (0 -> 127 = -1)
    "mt4": (89, 4),             # the low 4 bits of index_count (the "revolution counter near 290 degrees")
    "crc": (96, 16),
}

# Absolute payload fields as currently decoded by the FPGA module
ABSOLUTE_FIELDS = {
    "16bit": {"single_turn": (60, 16), "multi_turn": (76, 16)},
    "17bit": {"single_turn": (60, 17), "multi_turn": (77, 16)},
}


def hdlc_unstuff(wire_bits: str) -> list:
    """Hunt for the opening flag, then drop stuffed zeros until the closing flag."""
    bits = [int(c) for c in wire_bits]
    ones, i, started, out = 0, 0, False, []
    while i < len(bits):
        b = bits[i]
        if not started:
            ones = ones + 1 if b else 0
            if ones == 6:
                if i + 1 < len(bits) and bits[i + 1] == 0:
                    i += 1
                started, ones = True, 0
            i += 1
            continue
        if b == 0:
            if ones == 5:
                ones = 0
                i += 1
                continue
            ones = 0
        else:
            ones += 1
            if ones == 6:
                break
        out.append(b)
        i += 1
    return out


def field(payload: list, offset: int, width: int) -> int:
    return sum(payload[offset + k] << k for k in range(width))


def crc16_x25(payload: list) -> int:
    crc = 0xFFFF
    for byte in range(12):
        crc ^= field(payload, byte * 8, 8)
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc ^ 0xFFFF


def decode(wire_bits: str) -> dict:
    """Decode an on-wire (post-Manchester) bit string into payload fields."""
    payload = hdlc_unstuff(wire_bits)
    if len(payload) < 112:
        raise ValueError(f"short frame: {len(payload)} payload bits")
    d = {name: field(payload, o, w) for name, (o, w) in INCREMENTAL_FIELDS.items()}
    for mode, fields in ABSOLUTE_FIELDS.items():
        for name, (o, w) in fields.items():
            d[f"{mode}_{name}"] = field(payload, o, w)
    d["crc_ok"] = crc16_x25(payload) == d["crc"]
    d["payload96"] = field(payload, 0, 96)
    return d


def stuff(payload_bits: str) -> str:
    """HDLC bit stuffing: insert a 0 after every run of five 1s."""
    out, ones = [], 0
    for c in payload_bits:
        out.append(c)
        ones = ones + 1 if c == "1" else 0
        if ones == 5:
            out.append("0")
            ones = 0
    return "".join(out)


def line_edges(wire_bits: str, symbol_ns: float = 125.0, start_ns: float = 0.0):
    """Manchester-encode bits into (time_ns, level) line transitions.

    Matches the waveform the FPGA receiver was verified against: each bit is two
    symbols with level = bit XOR symbol_clock (clock starting at 0).
    Returns (edges, end_time_ns).
    """
    edges, t, clk, prev = [], start_ns, 0, None
    for c in wire_bits:
        for _ in range(2):
            level = int(c) ^ clk
            if level != prev:
                edges.append((t, level))
                prev = level
            clk ^= 1
            t += symbol_ns
    return edges, t


def read_capture_packets(path: str) -> list:
    """Split a Saleae Manchester-analyser CSV into on-wire packets.

    The analyser loses the preamble and the flag's leading 0 while it syncs, so
    each packet starts at '1111110'. A gap > 500 ns separates packets.
    """
    packets, cur, prev = [], [], None
    with open(path) as f:
        next(f)
        for line in f:
            p = line.strip().split(",")
            ts, v = float(p[1]), int(p[2])
            if prev is not None and ts - prev > 500e-9:
                packets.append("".join(cur))
                cur = []
            prev = ts
            cur.append(str(v))
    if cur:
        packets.append("".join(cur))
    return packets
