#!/usr/bin/env bash
# Set up a Mac for controller-software development. Safe to re-run.
#
# Installs:  cmake, ninja, python 3.12 (Homebrew)
#            .venv with controller-firmware/python/requirements.txt (Amaranth, sims)
#            em-zynq-toolchain Docker image (cross compiler for controller_core, native g++ for its host tests)
# Not possible on macOS: Vivado 2023.1 (bitstreams) and PetaLinux 2023.1 (em-os),
#            which need an x86-64 Linux (or Windows, for Vivado) machine.
#
# usage: ./setup_macos.sh [--verify]   --verify also runs the sims, the core host tests and a core build
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERIFY=0
[[ "${1:-}" == "--verify" ]] && VERIFY=1

step() { printf '\n==> %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }

[[ "$(uname)" == Darwin ]] || die "this script is for macOS"

step "Xcode command line tools"
if ! xcode-select -p >/dev/null 2>&1; then
    xcode-select --install || true
    die "finish the Xcode command line tools install, then re-run this script"
fi
echo "ok: $(xcode-select -p)"

step "Homebrew packages"
command -v brew >/dev/null || die "install Homebrew first: https://brew.sh"
for formula in cmake ninja python@3.12; do
    if brew list --formula "$formula" >/dev/null 2>&1; then
        echo "ok: $formula"
    else
        brew install "$formula"
    fi
done

step "Python environment (.venv)"
if [[ ! -x "$ROOT/.venv/bin/python" ]]; then
    "$(brew --prefix python@3.12)/bin/python3.12" -m venv "$ROOT/.venv"
fi
"$ROOT/.venv/bin/pip" install --quiet --disable-pip-version-check -r "$ROOT/controller-firmware/python/requirements.txt"
"$ROOT/.venv/bin/python" -c "import amaranth, sys; print(f'ok: python {sys.version.split()[0]}, amaranth {amaranth.__version__}')"

step "Docker (cross toolchain for controller_core)"
command -v docker >/dev/null || die "install Docker Desktop first: https://www.docker.com/products/docker-desktop/"
if ! docker info >/dev/null 2>&1; then
    echo "starting Docker Desktop..."
    open -a Docker
    for _ in $(seq 60); do
        docker info >/dev/null 2>&1 && break
        sleep 2
    done
    docker info >/dev/null 2>&1 || die "Docker did not start; start Docker Desktop and re-run"
fi
docker build --quiet -t em-zynq-toolchain "$ROOT/controller-software/core/docker" >/dev/null
docker run --rm em-zynq-toolchain arm-linux-gnueabihf-g++ --version | head -1 | sed 's/^/ok: /'

if [[ $VERIFY == 1 ]]; then
    step "Verify: FPGA sims"
    (cd "$ROOT/controller-firmware/python" && ../../.venv/bin/python sim/yaskawa_encoders_sim.py --quick 2>/dev/null)

    step "Verify: controller_core host tests"
    "$ROOT/controller-software/core/tests/run_host_tests.sh" 2>/dev/null | grep -E "^==|PASS|FAIL|passed|failed"

    step "Verify: controller_core cross build"
    "$ROOT/controller-software/core/docker_build.sh"
fi

step "Done"
cat <<EOF
FPGA sims:        cd controller-firmware/python && ../../.venv/bin/python sim/yaskawa_encoders_sim.py
Verilog check:    .venv/bin/python controller-firmware/python/sim/convert_check.py
controller_core:  controller-software/core/docker_build.sh
Host tests:       controller-software/core/tests/run_host_tests.sh
Still needs Linux/Windows: Vivado 2023.1 bitstream builds, PetaLinux 2023.1 image builds.
EOF
