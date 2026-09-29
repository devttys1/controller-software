# FPGA simulations

Testbenches and checks for the Amaranth modules in `../src`. Run them from
`controller-firmware/python` with the repo's `.venv` (see `setup_macos.sh`).

| Script | What it does |
| --- | --- |
| `yaskawa_encoders_sim.py` | Regression sim for `yaskawa_encoders.py`: request frame, absolute and incremental decode (real captured frames, ±2% clock skew), CRC errors, no-response timeout. `--quick` for a subset, `-k name` to filter. |
| `convert_check.py` | Converts the whole design in `controller.py` to Verilog without Vivado, to catch elaboration errors early. |
| `make_yaskawa_fixtures.py` | Regenerates `data/yaskawa_incremental_frames.txt` from the yaskawa-incremental-encoder logic analyser captures. |
| `yaskawa_protocol.py` | Shared Yaskawa reference decoder, line encoder and capture reader. |

```sh
cd controller-firmware/python
../../.venv/bin/python sim/yaskawa_encoders_sim.py
```

New sim scripts belong here rather than in `src/sandbox/`, and should exit
non-zero on failure so they can be run as regression checks.
