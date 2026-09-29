"""Convert the full controller design (controller.py's `nodes`) to Verilog, without Vivado.

Catches elaboration errors in any module before a bitstream build, which is
useful on machines that cannot run Vivado (e.g. macOS). Writes nothing into the
repo unless --out is given.

usage (from controller-software/):
    .venv/bin/python controller-firmware/python/sim/convert_check.py
    .venv/bin/python controller-firmware/python/sim/convert_check.py --incremental-port 5 --preamble-bits 25
"""
import argparse
import contextlib
import io
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--incremental-port", type=int, action="append", default=[],
                        help="switch this yaskawa_encoders port to 17bit_incremental (repeatable)")
    parser.add_argument("--preamble-bits", type=int, default=None, help="yaskawa_encoders request preamble length")
    parser.add_argument("--out", help="write the Verilog here")
    args = parser.parse_args()

    with contextlib.redirect_stdout(io.StringIO()):     # controller.py prints the whole register map
        import controller
        from amaranth.back import verilog
        from yaskawa_encoders import Yaskawa_Encoders

        nodes = dict(controller.nodes)
        yaskawa = nodes.get("yaskawa_encoders")
        if yaskawa is not None and (args.incremental_port or args.preamble_bits):
            modes = [dict(m) for m in yaskawa.encoder_settings]
            for port in args.incremental_port:
                modes[port] = {"mode": "17bit_incremental"}
            nodes["yaskawa_encoders"] = Yaskawa_Encoders(
                yaskawa.number_of_encoders, modes, args.preamble_bits or yaskawa.preamble_bits)

        start = time.time()
        text = verilog.convert(controller.Controller(nodes, False), name="Controller")

    if args.out:
        with open(args.out, "w") as f:
            f.write(text)
    print(f"ok: converted in {time.time() - start:.0f}s, {len(text) // 1024} KB of Verilog")


if __name__ == "__main__":
    main()
