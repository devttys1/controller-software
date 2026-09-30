# Yaskawa incremental encoder support

Adding Yaskawa 17-bit **incremental** serial encoders (UTSIH-B17CK, as fitted to an
SGMGH-13DCA6F servo motor, 4 pole pairs, rated 1500 rpm) alongside the existing absolute encoder
support. The first use is a **CNC spindle**, not a robot joint: speed control,
commutation and spindle position matter; multi-turn absolute position does not.

## Status

| Phase | State |
| --- | --- |
| 1. FPGA module: incremental mode, response timeout, preamble option | **Done**, running on the bench. Also exposes the raw payload; `unindexed` is payload bit 6; the stuffed-bit and `crc_valid` receiver bugs are fixed (2026-09-26) |
| 2. C++ driver: status, revolution tracking, speed | **Done**, running on the bench: hall reference, index (`indexed`, `index_angle`, `index_turns`), speed from the encoder's step times, and the monitor ring (2026-09-26) |
| 3. Regenerate bitstream + configs | **Done** for `spindle_bench` (Vivado 2023.1 on the Ubuntu build server, `remote_build.sh`) |
| 4. Spindle commutation and control | Angle side done: the index gives the true shaft angle after at most one turn, and `index_electrical_angle` locks `electrical_angle` to it. M19 logic done (`spindle_orient` node, host-tested, hand-checked on the bench). **Open:** where commutation runs (drive or controller), the spindle speed loop and the drive integration. The spindle encoder's hall table and index angle are to be measured when it is built |
| Bench testing | `spindle_bench` on the Z-Turn with the ffboard's encoder, 2026-09-24 to 26: decode, index, power cycles, speed and M19 by hand |

Branch: `yaskawa-incremental` (controller-software).

## Background

The protocol was reverse-engineered and proven on an STM32 in
[yaskawa-odrive](https://github.com/devttys1/yaskawa-odrive); its README's
[Protocol](https://github.com/devttys1/yaskawa-odrive#protocol) section is the protocol
reference. The sim fixtures in `controller-firmware/python/sim/data` were decoded from that
project's logic-analyser captures, which are not published. The OpenFFBoard integration of the
same motor is on the
[`f767zi-yaskawa` branch of OpenFFBoard](https://github.com/devttys1/OpenFFBoard/tree/f767zi-yaskawa).

Both encoder types use the same link: 4 Mbit/s Manchester (125 ns symbols) over
half-duplex RS-485, HDLC framing with bit stuffing, a 112-bit LSB-first payload and
CRC-16/X.25 over payload bits 0:96. The request payload is `0xFFFF` for both. Only
the payload fields after bit 77 differ:

| Payload bits | Absolute (`17bit` mode) | Incremental (`17bit_incremental` mode) |
| --- | --- | --- |
| 0 / 2 | unindexed / battery fail | same |
| 24:40 | timestamp_fast | same (~2 us/tick down counter) |
| 57:60 | not read | hall U, V, W |
| 60:77 | single-turn (17 bit) | same |
| 77:93 | 16-bit multi-turn | 77:80 `rot3` (steps at the single-turn wrap), 80:89 id latched at power-up, 89:93 `mt4` (steps near 290°) |

## Findings (sim, 2026-09-24)

- The existing receiver already decoded real incremental frames correctly in
  `17bit` mode (single-turn, CRC, unindexed). Only the multi-turn field was wrong:
  it packed `rot3 | id_latched << 3 | mt4 << 12` (e.g. `0x3852`), which would make
  the driver's `total_turns` jump by 4096 turns whenever `mt4` stepped.
- `commutation_count` (single-turn >> 16) is copied straight to the drive by FPGA
  copy instructions, so commutation never depended on the multi-turn field.
- The request frame matches the one proven on the incremental encoder bit for bit,
  except that 15 preamble bits reach the wire instead of 25. Nothing has tested
  whether the encoder accepts 15.
- Bugs in the existing module, now fixed: the receiver waited forever for a reply
  (a stray edge long after a silent encoder cleared `no_response`), and reading
  `status` returned stale bits from the previous register read in bits 5-31.

## Phase 1: FPGA module (done)

`controller-firmware/python/src/yaskawa_encoders.py`:

- New per-port mode `{"mode": "17bit_incremental"}`: `singleturn_count` and
  `commutation_count` as in `17bit` mode, `multiturn_count` = `mt4`, and a new
  packed register `incremental_info` with `hall` and `rot3`.
- A 100 us response timeout: a silent encoder leaves `no_response` set and the
  receiver ignores the line until the next request.
- A `preamble_bits` constructor argument (default 15, unchanged; 25 reproduces the
  proven request exactly). It applies to all ports, because the request line is shared.
- `driver_settings` now carries `encoder_count`, `encoder_modes` and `preamble_bits`,
  so the C++ driver no longer has to hard-code them.
- The module-level test bench, which built a six-encoder instance on every
  import, was replaced by `controller-firmware/python/sim/yaskawa_encoders_sim.py`.

Existing register offsets are unchanged: `incremental_info` takes the free slot
at offset 6 of each encoder's 8-word block.

## Phase 2: C++ driver (done)

`controller-software/core/controller/fpga_module_drivers/src/yaskawa_encoders.cpp`:

- Port count comes from the register map and per-port modes from `driver_settings`.
  An `fpga_config.json` from before this work (no driver settings) still loads, with
  every port treated as absolute.
- Copies `status` for every port, plus `timestamp_fast` and `incremental_info` for
  incremental ports only. The copy instruction memory holds 256 entries and the
  current test config already uses 169 before compaction, so check the condensed
  count on hardware.
- A frame is used only when `done && !crc_fail && !no_response` and, for incremental
  ports, `timestamp_fast` has changed. The FPGA keeps the last good values after a
  bad frame (verified in sim), so otherwise stale data would look valid.
- Node variables (`yaskawa_encoders:<port>.<name>`, see `yaskawa_encoders.h`):
  `total_turns`, `frame_valid`, `no_response`, `unindexed`, `battery_fail`,
  `frame_error_count`, `no_response_count`, plus `velocity_rpm` and `hall` on
  incremental ports.
- `velocity_rpm` is the position change over the real interval since the last good
  frame (the update frequency, times the cycles elapsed), through a first-order
  low-pass. Set `velocity_filter_hz` in the module's section of
  `fpga_driver_config.json` (default 100, 0 disables it). One count per 1 ms frame
  is 0.46 rpm.
- Incremental `total_turns` starts at the shaft angle (single-turn / 2^17) on the
  first frame, so its fraction is the spindle angle.
- No glitch filter. The reference one addressed noise on its SPI hop to the ODrive,
  which this system does not have, and the CRC covers the encoder link.

**Tracking uses `rot3`, not the reference's `mt4` fallback.** `rot3` (payload bits
77:80) steps exactly at the single-turn wrap: it agreed with `single_turn` on all
~22,400 frames of the six captures. Tracking the shortest path of
`(rot3 << 17) | single_turn` is exact for up to ±4 revolutions between good frames
(about 80 lost frames at 3000 rpm with a 1 kHz update). The reference tracks
`single_turn` alone and consults `mt4`, which steps near 290°, only near half a
turn. On thinned-out captures it lost or invented revolutions (1 rev off at every
30th frame, 70 rev at every 45th) where `rot3` tracking stayed exact. At full rate
the two agree exactly. See `core/tests/yaskawa_tracker_test.cpp`.

Found along the way: `Register` computed its bit mask as `(1 << width) - 1`, which is
undefined for 32-bit registers. It gave a full mask on the Zynq's 32-bit ARM by
accident and 0 on 64-bit hosts, so every 32-bit register read as zero there. Fixed
in `register_helper.cpp`.

Bench capture (bench step 2): the monitor web page's "Download CSV" saves the last 16 s
of every cycle, including `single_turn` and `timestamp_fast` (see Running it).

Spindle notes: multi-turn origin and homing do not matter. Continuous position is
still needed for rigid tapping / threading, and a repeatable angle for spindle
orient (M19), which depends on the index behaviour below.

## Phase 3: regenerate

Builds run on a Linux build server (Ubuntu 24.04, x86-64) over ssh. PetaLinux is not
needed: the PS configuration in BOOT.BIN is unchanged and the controller loads
`bitfile.bit.bin` at runtime.

Vivado 2023.1 (matching the saved project; the part and IP are covered by the free
ML Standard edition):

- Install with the AMD Unified web installer, selecting only Vivado ML Standard and
  Zynq-7000 devices, instead of the ~100 GB full download.
- Run it in an Ubuntu 22.04 container: 2023.1 does not support 24.04.
- The server's root volume was grown from 100 GB to 177 GB for this (`lvextend -r`).
- Needs from the user: the web installer from amd.com (account login) and one
  interactive `xsetup -b AuthTokenGen` run for the download credentials.
- Installed 2026-09-24 with `controller-firmware/vivado-runtime/install_vivado.sh`:
  a 14.6 GB download, 34 GB on disk in `~/Xilinx` (root volume since grown to 177 GB).

Build tooling (`controller-firmware/vivado-runtime/`):

- `controller.py` now only generates `controller.v` and `fpga_config.json` off
  Windows (the Windows Vivado flow is unchanged).
- `remote_build.sh --host user@build-host --name <name>` generates
  locally, copies the Vivado sources to `~/em-builds/<name>` on the host, builds, and
  fetches `bitfile.bit.bin`, `fpga_config.json` and reports into
  `core/zynq_files/controller/config/fpga_configs/<name>/`.
- `build_bitstream.sh` patches the saved project script's `S:/Vivado` paths into a
  copy, creates a fresh project, runs implementation, reports WNS/WHS and runs
  bootgen. The `export bit.bin.tcl` post-bitstream hook is disabled because of its
  Windows paths.
- Vivado 2023.1 aborts in `libudev` (`udev_enumerate_scan_devices` → `realloc`) when
  `launch_runs` collects host info in a container; `LD_PRELOAD` of the system
  libudev fixes it (non-project synthesis was unaffected, disabling WebTalk did not help).

1. Make the FPGA build scriptable on Linux. `controller.py` currently exits unless
   the Windows `S:` subst drive exists and runs Vivado from `C:\Xilinx`, and
   `Vivado/build.tcl` opens `S:/Vivado/...`.
2. Set the spindle port's mode in the `s` list in
   `controller-firmware/python/src/controller.py`, and `preamble_bits=25` if bench
   step 1 shows the encoder needs it.
3. Build the bitstream and `fpga_config.json`. `sim/convert_check.py` checks the
   design converts to Verilog without Vivado.
4. The user FPGA instruction file must include the new copy instructions: take
   them from `config/temp/default_fpga_instructions.json` after the first run.

## Bench example: `spindle_bench`

One incremental encoder, nothing else wired. Build variant `EM_FPGA_DESIGN=spindle_bench`
(`controller.py`): the `yaskawa_encoders` node has a single `17bit_incremental` port with
the 25-bit request preamble proven on these encoders; every other node is unchanged, so
node names and addresses match the default design.

- Bitstream and FPGA config: `core/zynq_files/controller/config/fpga_configs/spindle_bench/`
  (`remote_build.sh --design spindle_bench`; built 2026-09-24, WNS +0.725 ns, WHS +0.020 ns)
- Controller config: `core/zynq_files/controller/config/spindle_bench/`: only serial card C
  port 0 in RS-485 mode, no node networks, the driver-generated default FPGA
  instructions, and the Yaskawa driver printing each encoder every 500 ms.

### Wiring (serial card C port 0 = Yaskawa encoder 0)

In RS-485 mode the serial card logic uses two FPGA pins per port: driver enable, and one
bidirectional data pin that drives TX while enable is high and reads RX otherwise.

| Signal | Slot C | FPGA ball (bank 35 I/O) | Carrier CN2 / Z-Turn header pin |
| --- | --- | --- | --- |
| DE and /RE (tied) | `SLOTC10P` (`SLOT_C[18]`) | B19 (`IO_L2P_T0_AD8P_35`) | 22 |
| DI and RO (tied) | `SLOTC10N` (`SLOT_C[19]`) | A20 (`IO_L2N_T0_AD8N_35`) | 24 |
| GND | | | 20, 21 (also 2, 4) |

From `controller-hardware` (carrier `CN2`, which mates with the Z-Turn's 2×40 1.27 mm
header) and `Vivado/constraints/system.xdc` (LVCMOS33). Confirm against MYIR's Z-Turn pin
table that balls B19 and A20 are on pins 22 and 24 of that header.

- Use a 3.3 V RS-485 transceiver (e.g. SN65HVD72, MAX3485), as in the reference project.
- Put 100-330 Ω in series between RO and the data pin, so the transceiver and FPGA don't
  fight during the direction change.
- 10 kΩ pull-down on DE, so the bus stays released until the bitstream loads.
- A/B to the encoder's data pair with 120 Ω termination at the far end; encoder 5 V from a
  bench supply with grounds common.
- Carrier `CN2` pin 3 is +3.3 V, but check the Z-Turn's rating before powering the
  transceiver from it.

### Running it

From `controller-software/core` (details in [`build_and_zynq_setup.md`](../controller-software/core/build_and_zynq_setup.md)):

```sh
./zynq.sh deploy
./zynq.sh start controller/config/spindle_bench/config.json
./zynq.sh log      # or: ./zynq.sh status / ./zynq.sh stop
```

Every 500 ms the log shows
`yaskawa_encoders:0 turns … rpm … hall … valid … unindexed … no_response … frame_errors … no_responses …`.
With no encoder wired it reports `no_response 1` and `no_responses` rising by 1000 a
second (one request per 1 ms cycle), which is what the 2026-09-24 run on the Z-Turn showed.
`./zynq.sh start` also starts the monitor web page, `http://em-os.<your local domain>:8080/`.
It shows speed, position, electrical angle against the hall sectors, the hall state,
the hall reference and link errors live at 1 kHz, and downloads the last 16 s as CSV
(see [`build_and_zynq_setup.md`](../controller-software/core/build_and_zynq_setup.md)).

First run with the encoder wired (2026-09-25, motor at standstill, not yet turned by hand):
`valid 1`, `no_response 0`, `hall 3`, `unindexed 1` (as expected before the first index),
`turns +0.997`, and `rpm` within ±0.2 (filter noise at standstill). `frame_errors` rose by
8 in about 22 s (about 1 in 3000 frames), spread out rather than at startup. That is not
zero, so check the RS485 wiring (series resistor on RO, ground, cable length, termination)
before trusting it at speed.

Turning the shaft by hand (2026-09-25): `turns` follows smoothly through whole turns
(about 7 turns at about −90 rpm, with `rpm` matching the rate of change of `turns`), and
`hall` only takes the values 1–6. The 500 ms prints are too slow to check the hall
sequence. `unindexed` stayed set after more than 10 turns, as in the reference project
(its README lists this as open): what clears it is still unknown.

The same run logged "FPGA interrupt missed" / "Error: FPGA update failed" every 5 s, with
"Update duration" about 7 ms. The debug print writes to stdout from the realtime loop, and
the log was on the SD card: a write that lands during the kernel's 5 s writeback blocks.
`zynq.sh` now logs to `/var/volatile/controller.log` (tmpfs). After that, 75 s showed no
overruns (apart from the usual one on the first cycle) and no frame errors. The 47 frame
errors in the earlier run were close to its 53 overruns, so they may have been caused by
the overruns, but that isn't proven. The real fix is to print from a non-realtime thread.

### Found while bringing it up (2026-09-24)

- **FPGA update interrupts.** In the saved block design `Controller_0/pl_ps_interrupts`
  (16 bits) went straight to `processing_system7_0/IRQ_F2P`, which stayed 1 bit wide (the
  long-standing "width mismatch" critical warning), so only "update running" reached the
  PS. Every bitstream built from the repo's project script hung the controller waiting
  for "update done". The working `test` bitstream must have come from a project where the
  width was set. `controller_firmware.tcl` now routes the interrupts through an
  `xlconcat` (`irq_concat`), which makes `IRQ_F2P` 16 bits, and `build_bitstream.tcl`
  fails the build if it is ever narrower than 4.
- **Empty node config.** `Node_Core::run_update` crashed (null network) when the node
  config had no networks, or any gap in execution orders. Fixed, with a host test.
- **Not fixed:** `Fpga_Interface::wait_for_update` does a blocking `read()` on the
  "update done" interrupt even when its 1 ms poll timed out, so a missing done interrupt
  hangs the controller instead of reporting "Timeout waiting for FPGA update finish".

## Payload map (decoded on the bench 2026-09-26)

The FPGA exposes the whole 96-bit payload of the last good frame (`raw_payload_0..2`
in the `yaskawa_encoders` register map; payload bit 0 is bit 0 of word 0). The driver
copies it on incremental ports into the monitor ring, and so into the CSV.
- `core/tools/record_monitor.py out.csv --seconds N` records the monitor stream for any length of time.
- `core/tools/analyse_yaskawa_payload.py` maps the constant and changing bits, and lists every change of a
  rarely-changing field with where the shaft was.
- The capture this map comes from is `core/tools/data/yaskawa_payload_bench_2026-09-26.csv.gz`: 212 s,
  about 191,000 frames. It covers standstill, slow turning both ways, a creep, spins to ±190 rpm, and 4
  encoder power cycles.
- The names are in `controller-firmware/python/sim/yaskawa_protocol.py` (`INCREMENTAL_FIELDS`).

| Bits | Field | Behaviour |
| --- | --- | --- |
| 0 | (called `unindexed`) | Always 1, across power-ups and index crossings. **Not the index flag.** The FPGA's `status.unindexed` reads this bit. |
| 1–5, 7 | — | Always 0. Bit 2 is `battery_fail` and bit 7 was called `comp_valid_n`. |
| 6 | `index_not_found` | 1 from power-up until the shaft first passes the index mark, then 0. |
| 8:16 | `ready` | 0x00 for about 0.1–0.2 s after power-up, then 0xFF. |
| 16:24 | `ts_slow` | Steps about every 216 ms. |
| 24:40 | `ts_fast` | Advances exactly 6001 per 1 ms frame: a ~6 MHz up-counter (or an alias of one). The reference README's "2 µs down-counter" does not fit this. |
| 40:56 | `position_age` | Time since the encoder's internal position last changed, in `ts_fast` ticks (6 MHz, 0.167 µs), taken when the frame was built. `ts_fast − position_age` is when the reported position was reached. Wraps at 65536 (10.9 ms). It was called `raw40` until identified (see below). |
| 56 | `index_pulse` | 1 while the shaft is within about ±120 counts (±0.33°) of the index mark. |
| 57:60 | `hall` | |
| 60:77 | `single_turn` | Its zero moves at every power-up. |
| 77:80 | `rot3` | Counts `single_turn` wraps since power-up; 0 at power-up. |
| 80:89 | `index_position` | `single_turn` at the index mark, >> 8. 0 until the first crossing, then latched. |
| 89:96 | `index_count` | Signed 7-bit count of index crossings since power-up: +1 forward, −1 back. It is 0 at power-up and went 0 → 127 (−1) at a first crossing in reverse. The old `mt4` is its low 4 bits, and `status_hi` is its top 3. |

**`position_age` (bits 40:56), identified 2026-09-26** from about 40 minutes of bench frames:
- **It is a time in the timestamp's ticks.** In 555 frames at rest where it grew by a whole frame, it grew by
  exactly 6000 or 6001, the same as `ts_fast`. `ts_fast − position_age` stayed constant, and `single_turn`
  did not change.
- **It resets when the position changes.** Whenever it restarted between frames, the position had moved.
  In 115,676 of those cases `single_turn` itself changed; in the other 75,127 it was a dither too small or
  too brief to show in the 17-bit count. It wraps at 65536.
- **At rest** the internal position dithers every few µs (median about 50 ticks), with quiet spells of
  milliseconds; those are the large values once thought to be noise.
- **At a steady speed** its mean is about 1500 ÷ (counts per ms) ticks. That is half a count's worth of time,
  so the internal position steps about twice per reported count: an 18-bit internal resolution.
- **Speed from the step times (2026-09-26):** `yaskawa_velocity.h` measures speed between the moments the
  encoder reports its position changed (`ts_fast − position_age`), rather than between frames.
  - Its window reaches back up to `velocity_window_ms` (20) or until `velocity_window_counts` (64) counts have
    passed.
  - When counts stop arriving it falls to zero as 1 ÷ (time since the last one).
  - The encoder's clock rate (6001 ticks per controller ms) is learned against the controller's.
  - The driver uses it for `velocity_rpm` when the raw payload is there, still through `velocity_filter_hz`.
  - Simulated: 3.6× quieter at rest, 9× at 0.5 rpm, 3.3× at 5 rpm, and the same at 50 and 1500 rpm. The mean
    is right to 0.003 rpm.
  - On the bench at rest: 0.018 rpm rms against 0.115 rpm for the per-frame estimate on the same frames.
- **Not yet taken up:** the reported position was reached `position_age` ticks before the frame, so
  position could be extrapolated to the frame time.

**Power-up zero shifts are whole multiples of 256 counts.** The index mark fell at the same place modulo 256
(within about 55 counts, which is crossing noise) in every power-up. `position_age` also shows the same
two-humped pattern over each 256-count cycle across all power-ups. So 256 counts is the optical line pitch
(512 lines per turn). The encoder knows its phase within a line at power-up, but not which line it is on.

**The encoder has a once-per-turn index.**
- The index mark sat at the same rotor position in both power-ups that crossed it: 118–122° electrical,
  in hall sector 1.
- `index_position` × 256 matched the counter's step point: 40448 against 40452–40502 before the power
  cycles, and 30208 against 30227 after.
- So after one pass of the index (at most one turn after power-up), the shaft angle is
  `single_turn − index position`, absolute and repeatable. That gives spindle orientation (M19) without a
  separate sensor, and commutation from the index instead of the hall edges. `index_count` gives
  whole turns since power-up.
- **Index support (2026-09-26):**
  - The FPGA's `status.unindexed` reads bit 6 on incremental ports.
  - `yaskawa_index.h` finds the mark. It starts from the encoder's 256-count `index_position` and refines
    it from the frames where `index_count` steps, averaging the forward and backward crossings.
  - The driver publishes `indexed`, `index_angle` (the true shaft angle, zero at the mark) and
    `index_turns`.
  - With `index_electrical_angle` in the driver config, `electrical_angle` locks to the index once it is
    found (`hall_reference` 3). This replaces the hall edges, which stay a cross-check (`hall_deviation`).
    The value is where the mark sits in the hall frame; the driver measures it as
    `index_electrical_angle_measured` while the lock is from hall edges. For this encoder it is 0.332
    (119.6°), from the six power-ups whose hall lock was good (`spindle_bench` config).
  - Host-tested on 16,000 real frames around 83 crossings over 13 power-ups (`tests/yaskawa_index_test.cpp`):
    - every pair of same-direction crossings is exactly one turn apart;
    - each crossing reads within 36 counts beyond what the frame step allows.
  - The monitor page shows it.
  - Bench-checked 2026-09-26: after an encoder replug the index angle at the shaft mark came back the
    same, and the driver went from sector estimate to index lock on the first pass of the mark. The hall
    edges agreed with the index lock to within 143 counts (1.6 electrical degrees).

**Found on the way:**
- **Hall reference after encoder power loss:** the driver now restarts the hall reference after 20 ms
  without a good frame (`hall_reference_restarts`). An encoder power cycle moves `single_turn`'s zero, so
  the old lock would be wrong.
- **Errors before the first good frame:** frame errors now count from the first exchange. Before, a link
  that never produced a good frame showed no errors at all.
- **"Freeze" at index count −2: an FPGA bug, fixed 2026-09-26.** On the bench the link froze for
  exactly the turn in which `index_count` read −2, in either direction. It recovered at the next index
  crossing with the right count, and all other counts were fine. The encoder was sending good frames
  throughout. −2 (and +62) put five 1s in payload bits 90–94 after a 0, so the encoder stuffs a 0 just
  before bit 95. The receiver took that stuffed bit as "last payload bit reached" and left the real
  bit 95 out of the CRC, so every such frame failed.
  - A second bug hid the failures: the receiver cleared `crc_valid` from a one-cycle sync_200
    condition that the sync_100 logic missed on the hardware. A failed frame therefore read as valid,
    with the previous frame's data (FPGA status `done` with no `crc_fail`, and a stale timestamp).
  - Both are fixed in `yaskawa_encoders.py`. The sim checks all 128 counts and 400 random dense-ones
    payloads; both tests fail on the old receiver.
  - Confirmed on the bench (2026-09-26, bitstream `771546f`): the counter went from −11 to +11 and
    passed through −2 five times, in both directions at 47–105 rpm, with no dropouts. 47 index
    crossings fell at 117.7–121.1° electrical. The only frame errors were at the replug: 1 as it
    went, and 26 in the encoder's first 26 ms.
- **The index is repeatable across power-ups.** In a controlled second run with 5 power-ups, the index
  sat at 119.0–119.9° electrical at 24 of 25 crossings; one read 109.5°, most likely a hall-lock error.
  `index_position` × 256 was within 62 counts of the crossing every time.

## Phase 4: spindle commutation## Phase 4: spindle commutation

**Bench result (2026-09-25): the encoder loses its position when it powers down.**
`single_turn`'s zero moves at every encoder power-up, which the
[OpenFFBoard integration](https://github.com/devttys1/OpenFFBoard/tree/f767zi-yaskawa) also measured on 2026-09-24 (about 170 electrical degrees between two power-ups). The
ODrive's `pre_calibrated` offset only looked repeatable because the encoder hadn't lost
power. A stale offset doesn't fault: it commutates smoothly with the torque reversed. So
nothing stored against `single_turn` or `total_turns` survives an encoder power cycle:
not a commutation offset, and not a spindle orientation.

**What does survive is the hall (commutation) track.** Within one power-up the six hall
edges sit at fixed `single_turn` values, repeatable to a few counts. The track is on the
disk, so it is fixed to the rotor, but it repeats every electrical revolution (a quarter
turn with 4 pole pairs).

`fpga_module_drivers/inc/yaskawa_hall_reference.h` ports the ffboard's `yaskawa_hallref.c`,
and the driver publishes its output as `yaskawa_encoders:<n>.electrical_angle`, in
electrical turns with zero at the hall 2|3 edge:

- `hall_reference` 1: from the first frame, the sector centre, at most ~33 electrical
  degrees out (still more than 83% of the torque, in the right direction).
- `hall_reference` 2: from the first clean crossing of any hall edge, within ~3 electrical
  degrees and final. Edges are 15 mechanical degrees apart, so any motion of more than
  that locks it. The correction is slewed 8 counts per frame, so the angle never jumps.
- `hall_deviation`: the worst disagreement of a later edge with the lock. Up to ~300 is
  normal; thousands means the edge table does not fit this encoder.

The sectors are uneven, so the lock uses a table of where each edge sits. The default is
the ffboard encoder's table. The bench uses that same encoder, so the default fits. The
spindle build will use a different encoder: **measure its table when it is built**:

1. The `spindle_bench` config sets `"debug_print_hall_edges": true`, so after the lock the
   log prints `yaskawa_encoders:0 hall edges [0, …] fewest crossings per direction N`.
2. Turn the shaft slowly by hand, a few turns each way, until N is 10 or more.
3. Copy the table into `hall_reference.edges` in `fpga_driver_config.json`, redeploy, and
   check that `halldev` stays below ~300 across several encoder power cycles.

If the hall states never appear in the order `3, 1, 5, 4, 6, 2` as `turns` increases,
set `hall_reference.sequence` instead. A wrong sequence never locks (`hallref` stays 1).

For the spindle this means:

- **Commutation:** calibrate the phase offset once, against `electrical_angle` after
  `hall_reference` reads 2. It then holds across power cycles. At start-up the drive can
  run on the sector estimate (at least 83% torque) until the first edge, 15 mechanical
  degrees later, locks it. A spindle doesn't need the ffboard's power-up jog, which exists
  because a steering wheel mustn't move on its own.
- **Orientation (M19, tool change):** the encoder's own index gives a true shaft angle after at most
  one turn (see Payload map), so no separate sensor is needed.
  - The `spindle_orient` node (`nodes/generic_nodes`, logic in `spindle_orient_core.h`) follows
    LinuxCNC's M19:
    - `angle` in degrees, measured from the index mark plus `angle_offset_deg`;
    - `mode`: 0 the shorter way, 1 forward only, 2 reverse only;
    - outputs `is_oriented` and `fault`;
    - dropping `enable` releases the spindle and clears a fault.
  - Its output is a speed command for the spindle's speed loop. A reference motion, limited by `max_rpm`
    and `accel_rpm_per_s`, stops exactly on the angle. The spindle follows it through `kp_per_s`, with
    acceleration feedforward set by `speed_loop_lag_s`.
  - It takes over from the spindle's actual speed, so an M19 while running carries on and brakes onto the
    angle.
  - Not indexed yet: it searches at `search_rpm` in the mode's direction.
  - Faults: no index within `search_turns_max`, the index lost (encoder powered down), or `timeout_s`.
  - Host-tested against a simulated spindle with a 20 ms speed loop (`tests/spindle_orient_test.cpp`,
    `spindle_orient_node_test.cpp`):
    - 180 orients the shorter way, all within 0.001° and 0.52 s, with at most 0.36° overshoot;
    - forward-only and reverse-only never travel the wrong way;
    - searching, orienting while running, disturbance, index loss, timeout and angle offset.
    - Without the feedforward, the overshoot grows to about 4°.
  - `spindle_bench` runs it against the live encoder, targeting 90°. There's no motor yet, so orient by
    hand: `./zynq.sh log` shows "M19 oriented: 1" when the shaft is held within 0.1° of 90° from the
    mark (the monitor's Index card).
  - Still needed to turn the spindle: the speed loop and the drive's commutation (above).
- **Still to decide:** where commutation runs. At the motor's rated 1500 rpm the rotor
  turns 100 electrical revolutions a second, so at the controller's 1 kHz cycle the angle
  moves 36 electrical degrees between samples. That is far too coarse to commutate from
  directly. Either the drive reads the encoder itself and gets this hall reference, which
  is a drive firmware change, or the controller sends the angle and speed with
  timestamps for the drive to extrapolate. That is workable for a spindle, whose speed
  changes slowly, but a lost frame costs 36 degrees of extrapolation.
- The lock only trusts edges crossed at up to ~230 rpm (`max_step`, 512 counts per
  frame), so it locks during the run-up from standstill, not at full speed. That's fine
  for a spindle that always starts from rest.

## Testing

### Offline (no hardware)

- `controller-firmware/python/sim/yaskawa_encoders_sim.py`: 120 real frames from
  six captures (stationary, slow, drill CW/CCW) replayed with ±2% clock skew, plus
  request-frame, absolute regression, mixed-port, CRC and timeout tests. All pass.
- `controller-software/core/tests/run_host_tests.sh` (runs in the Docker image on
  macOS):
  - `yaskawa_tracker_test`: real frame sequences, full rate and thinned out to
    ~2.5 rev per step, checked against the true position and, at full rate, against
    the reference `yask_track_update()`.
  - `yaskawa_driver_test`: the real driver loaded against a register map exported
    from the FPGA module and against the older test `fpga_config.json`, with frames
    written where the DMA copies land. Covers tracking over 300 real frames, CRC
    failures, no response, repeated frames, speed across missed frames, the
    velocity filter and absolute ports.

### Bench (hardware)

Setup: Zynq controller with serial card C, encoder on a port set to RS485 in
`fpga_driver_config.json`, motor unpowered for steps 1-6, and a logic analyser on
the card's TTL RO/DI/DE lines.

| Step | Check | Pass |
| --- | --- | --- |
| 0 | Current bitstream, port in `17bit` mode, watch `singleturn_count` | tracks the shaft (proves the 15-bit preamble and wiring) |
| 1 | Request on the wire, reply timing | reply ~17-30 us after request; 0 CRC / no-response over 60 s |
| 2 | Decode the analyser capture with the reference `test_decode`, join to the controller's values on `timestamp_fast` | 100% of joined frames identical |
| 3 | Turn slowly by hand | 6 hall states, 24 transitions per rev (4 pole pairs) |
| 4 | N marked turns CW, then back | `total_turns` = N ± 0.01, returns within a few counts |
| 5 | Drive the shaft with a drill | no step > 0.5 rev; totals agree with `rot3`/`mt4` |
| 6 | Power-cycle at several marked angles; turn past the index | record whether single-turn is repeatable and whether `unindexed` clears / single-turn jumps |
| 7 | Absolute encoders on the other ports | unchanged |
| 8 | Motor powered by the drive, chassis earthed | repeat 1 and 4; then speed steps up to the motor's rated 1500 rpm, checking `velocity_rpm` against a tachometer |

For step 2, the monitor page's "Download CSV" gets the controller's values off the
board: the last 16 s, every cycle, with `timestamp_fast` to join on.

Noise lessons from the ffboard work: earth the motor chassis, and twist each
signal with its own ground return.

## Development on macOS

`./setup_macos.sh --verify` installs cmake, ninja and Python 3.12, fills `.venv`
from `controller-firmware/python/requirements.txt`, builds the `em-zynq-toolchain`
Docker image (Debian bookworm cross gcc 12.2 / glibc 2.36, matching PetaLinux
2023.1), then runs the sims and a `controller_core` cross build.
`controller-software/core/docker_build.sh` builds the core the same way afterwards.
Vivado and PetaLinux builds still need a Linux (or Windows, for Vivado) machine.
