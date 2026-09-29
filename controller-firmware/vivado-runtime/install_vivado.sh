#!/usr/bin/env bash
# Install Vivado 2023.1 ML Standard with only Zynq-7000 devices on a Linux host, using the
# AMD web installer inside the vivado-2023.1-runtime container (Ubuntu 22.04), so hosts
# that 2023.1 does not support (e.g. Ubuntu 24.04) work too. No root needed.
#
# Copy this directory to the build host (it has no other repo dependencies).
# Download "AMD Unified Installer for FPGAs & Adaptive SoCs 2023.1: Linux Self Extracting
# Web Installer" from amd.com and extract it:
#     sh Xilinx_Unified_2023.1_0507_1903_Lin64.bin --noexec --target ~/xilinx-installer
#
# usage: install_vivado.sh --auth [installer dir]      log in to AMD once (interactive, needs a tty)
#        install_vivado.sh [installer dir] [dest]      download and install (default ~/Xilinx)
#        install_vivado.sh --config [installer dir] [dest]   only write the install config
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE=vivado-2023.1-runtime
MODE=install
[[ "${1:-}" == --auth || "${1:-}" == --config ]] && { MODE="${1#--}"; shift; }
INSTALLER="${1:-$HOME/xilinx-installer}"
DEST="${2:-$HOME/Xilinx}"
CONFIG="$HOME/.Xilinx/vivado_zynq7000_config.txt"

[[ -x "$INSTALLER/xsetup" ]] || { echo "error: no xsetup in $INSTALLER (extract the web installer first)" >&2; exit 1; }
docker build --quiet -t "$IMAGE" "$HERE" >/dev/null

# the host user must exist in the container, or the installer cannot find its home directory
run() {
    docker run --rm "$@"
}
container=(-v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro -v "$HOME:$HOME" -e HOME="$HOME" --user "$(id -u):$(id -g)" "$IMAGE")

if [[ $MODE == auth ]]; then
    run -it "${container[@]}" "$INSTALLER/xsetup" -b AuthTokenGen
    exit 0
fi

# generate the default Vivado ML Standard config (product 2, edition 1), then trim it:
# Zynq-7000 devices only, install under $DEST, no desktop integration
printf '2\n1\n' | run -i "${container[@]}" "$INSTALLER/xsetup" -b ConfigGen >/dev/null
sed -E \
    -e "s#^Destination=.*#Destination=$DEST#" \
    -e '/^Modules=/{s/:1(,|$)/:0\1/g; s/Zynq-7000:0/Zynq-7000:1/}' \
    -e 's/^(CreateProgramGroupShortcuts|CreateDesktopShortcuts|CreateFileAssociation)=1/\1=0/' \
    "$HOME/.Xilinx/install_config.txt" > "$CONFIG"
grep -E '^(Edition|Destination|Modules)=' "$CONFIG"
[[ $MODE == config ]] && exit 0

mkdir -p "$DEST"
run "${container[@]}" "$INSTALLER/xsetup" --agree XilinxEULA,3rdPartyEULA --batch Install --config "$CONFIG"
