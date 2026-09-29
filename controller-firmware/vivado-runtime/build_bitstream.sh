#!/usr/bin/env bash
# Build bitfile.bit.bin on a Linux host with Vivado 2023.1 (see install_vivado.sh), inside
# the vivado-2023.1-runtime container.
#
# The work directory must hold the contents of controller-firmware/Vivado (saved project
# Tcl, top level, constraints) with the generated autogen_sources/controller.v.
# remote_build.sh prepares it and runs this over ssh.
#
# usage: build_bitstream.sh <work dir> [jobs] [vivado install dir, default ~/Xilinx]
# SYNTH_STRATEGY in the environment overrides the saved project's synthesis strategy.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="$(cd "$1" && pwd)"
JOBS="${2:-8}"
XILINX="${3:-$HOME/Xilinx}"
IMAGE=vivado-2023.1-runtime

[[ -f "$WORK/controller_firmware.tcl" && -f "$WORK/autogen_sources/controller.v" ]] || {
    echo "error: $WORK needs controller_firmware.tcl and autogen_sources/controller.v" >&2
    exit 1
}
[[ -f "$XILINX/Vivado/2023.1/settings64.sh" ]] || { echo "error: no Vivado 2023.1 in $XILINX" >&2; exit 1; }

docker build --quiet -t "$IMAGE" "$HERE" >/dev/null

# the saved project script (written by Vivado on Windows) points at the S: subst drive and
# imports controller.v from inside the old project; use the work dir and autogen_sources instead
sed -E \
    -e 's#[$][{]?origin_dir[}]?/controller_firmware/controller_firmware.srcs/sources_1/imports/autogen_sources/controller.v#${origin_dir}/autogen_sources/controller.v#' \
    -e 's#S:/Vivado/controller_firmware/controller_firmware.srcs/sources_1/imports/autogen_sources/controller.v#./autogen_sources/controller.v#' \
    -e 's#S:/Vivado#.#g' \
    "$WORK/controller_firmware.tcl" > "$WORK/controller_firmware_linux.tcl"

mounts=(-v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro -v "$HOME:$HOME" -v "$HERE:$HERE" -v "$WORK:$WORK" -v "$XILINX:$XILINX")

# launch_runs reads host info for WebTalk registration through libudev, which aborts in
# realloc (heap mix-up with Vivado's allocator) unless libudev is loaded up front
docker run --rm "${mounts[@]}" -e HOME="$HOME" -e SYNTH_STRATEGY="${SYNTH_STRATEGY:-}" -e LD_PRELOAD=/lib/x86_64-linux-gnu/libudev.so.1 \
    --user "$(id -u):$(id -g)" -w "$WORK" "$IMAGE" \
    bash -c "source '$XILINX/Vivado/2023.1/settings64.sh' && vivado -mode batch -nojournal -log build.log -source '$HERE/build_bitstream.tcl' -tclargs '$WORK' '$JOBS'"
