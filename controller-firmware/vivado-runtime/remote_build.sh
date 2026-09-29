#!/usr/bin/env bash
# Build the FPGA bitstream on a Linux build host over ssh.
#
# Generates controller.v and fpga_config.json here (controller.py), copies the Vivado
# sources to the host, runs build_bitstream.sh there, and fetches bitfile.bit.bin, the
# reports and the log into
#     controller-software/core/zynq_files/controller/config/fpga_configs/<name>/
# which is the layout the controller loads (point fpga_config_path in config.json at it).
#
# usage: remote_build.sh [--host user@host] [--name name] [--design design] [--jobs n] [--synth-strategy s] [--no-generate]
#   --host    defaults to $EM_BUILD_HOST
#   --name    defaults to the design name
#   --design  controller.py design variant (EM_FPGA_DESIGN), defaults to "default"
#   --synth-strategy  override the saved project's synthesis strategy
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HOST="${EM_BUILD_HOST:-}"
NAME=""
DESIGN=default
JOBS=8
SYNTH_STRATEGY=""
GENERATE=1
while [[ $# -gt 0 ]]; do
    case "$1" in
        --host) HOST="$2"; shift 2 ;;
        --name) NAME="$2"; shift 2 ;;
        --design) DESIGN="$2"; shift 2 ;;
        --jobs) JOBS="$2"; shift 2 ;;
        --synth-strategy) SYNTH_STRATEGY="$2"; shift 2 ;;
        --no-generate) GENERATE=0; shift ;;
        *) echo "unknown option $1" >&2; exit 1 ;;
    esac
done
[[ -n "$HOST" ]] || { echo "error: pass --host user@host or set EM_BUILD_HOST" >&2; exit 1; }
NAME="${NAME:-$DESIGN}"

VIVADO="$ROOT/controller-firmware/Vivado"
OUT="$ROOT/controller-software/core/zynq_files/controller/config/fpga_configs/$NAME"
REMOTE="em-builds/$NAME"

if [[ $GENERATE == 1 ]]; then
    echo "==> generating controller.v and fpga_config.json (design: $DESIGN)"
    (cd "$ROOT" && EM_FPGA_DESIGN="$DESIGN" .venv/bin/python controller-firmware/python/src/controller.py > /dev/null 2>&1)
fi
[[ -f "$ROOT/fpga_config.json" && -f "$VIVADO/autogen_sources/controller.v" ]] || {
    echo "error: run without --no-generate first" >&2
    exit 1
}

echo "==> copying sources to $HOST:$REMOTE"
ssh "$HOST" "rm -rf '$REMOTE/work' && mkdir -p '$REMOTE/work/constraints' '$REMOTE/work/autogen_sources' em-builds/vivado-runtime"
rsync -a "$ROOT/controller-firmware/vivado-runtime/" "$HOST:em-builds/vivado-runtime/"
rsync -a "$VIVADO/controller_firmware.tcl" "$VIVADO/controller_firmware_top.sv" "$VIVADO/export bit.bin.tcl" "$HOST:$REMOTE/work/"
rsync -a "$VIVADO/constraints/" "$HOST:$REMOTE/work/constraints/"
rsync -a "$VIVADO/autogen_sources/controller.v" "$HOST:$REMOTE/work/autogen_sources/"

echo "==> building on $HOST (synthesis and implementation take a while)"
start=$SECONDS
set +e
ssh "$HOST" "SYNTH_STRATEGY='$SYNTH_STRATEGY' em-builds/vivado-runtime/build_bitstream.sh '$REMOTE/work' '$JOBS'" 2>&1 | grep --line-buffered -E "BUILD |ERROR|CRITICAL WARNING|^launch_runs|Finished Running|wait_on_run"
status=${PIPESTATUS[0]}
set -e
echo "==> build finished in $(( (SECONDS - start) / 60 )) min, exit status $status"

mkdir -p "$OUT/build"
rsync -a "$HOST:$REMOTE/work/build.log" "$HOST:$REMOTE/work/timing_summary.rpt" "$HOST:$REMOTE/work/utilization.rpt" "$OUT/build/" 2>/dev/null || true
[[ $status == 0 ]] || { echo "error: build failed, log in $OUT/build/build.log" >&2; exit "$status"; }

rsync -a "$HOST:$REMOTE/work/bitfile.bit.bin" "$OUT/"
cp "$ROOT/fpga_config.json" "$OUT/fpga_config.json"
echo "==> $OUT"
ls -la "$OUT"
