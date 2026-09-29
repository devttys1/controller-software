"""Build data/yaskawa_fpga_config.json for yaskawa_driver_test.cpp.

An fpga_config.json in the format controller.py writes, containing a
yaskawa_encoders node exported from the real FPGA module with one absolute and one
incremental port, so the driver test runs against the register map the FPGA
build actually produces.

usage (from controller-software/): .venv/bin/python controller-software/core/tests/make_yaskawa_driver_fixtures.py
"""
import contextlib
import io
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "controller-firmware", "python", "src"))
LEGACY_CONFIG = os.path.join(HERE, "..", "zynq_files", "controller", "config", "fpga_configs", "test", "fpga_config.json")
OUT = os.path.join(HERE, "data", "yaskawa_fpga_config.json")


def main():
    with contextlib.redirect_stdout(io.StringIO()):     # register-map generation logs
        from yaskawa_encoders import Yaskawa_Encoders
        encoders = Yaskawa_Encoders(2, [{"mode": "17bit"}, {"mode": "17bit_incremental"}])

    with open(LEGACY_CONFIG) as f:
        controller = json.load(f)["controller"]

    config = {
        "controller": controller,
        "node_4_yaskawa_encoders": {"node_address": 4, "node": encoders.rm.export()},
    }
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        json.dump(config, f, indent=4)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
