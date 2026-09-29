#!/usr/bin/env bash
# Build and run the controller_core host tests: pieces of the controller compiled
# for this machine (not the Zynq). Exits non-zero on failure.
#
# Each *_test.cpp is its own program. Controller sources it needs besides its
# headers are listed on a "// sources:" line, relative to this directory.
# Each *_test.py is run with python3.
#
# The controller uses Linux-only APIs, so on other systems (macOS) the tests run
# inside the core/docker image.
#
# usage: ./run_host_tests.sh [name filter]
set -euo pipefail

TESTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "$(uname)" != Linux ]]; then
    CORE="$(cd "$TESTS/.." && pwd)"
    docker build --quiet -t em-zynq-toolchain "$CORE/docker" >/dev/null
    exec docker run --rm -v "$CORE:$CORE" -w "$TESTS" --user "$(id -u):$(id -g)" \
        em-zynq-toolchain ./run_host_tests.sh "$@"
fi
CONTROLLER="$TESTS/../controller"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

CXX="${CXX:-c++}"
CXXFLAGS=(-std=gnu++20 -O1 -Wall -Wextra -Wno-unused-parameter -Wno-psabi)
for dir in inc fpga_module_drivers/inc nodes/inc nodes/generic_nodes/inc nodes/global_variables/inc nodes/solvers/inc api_drivers/inc api_drivers/calls/inc; do
    CXXFLAGS+=(-I"$CONTROLLER/$dir")
done

failed=0
cd "$TESTS"
for src in *_test.cpp; do
    name="${src%.cpp}"
    [[ -n "${1:-}" && "$name" != *"$1"* ]] && continue
    read -r -a sources <<< "$(sed -n 's|^// sources:||p' "$src")"
    echo "== $name"
    if ! "$CXX" "${CXXFLAGS[@]}" "$src" ${sources[@]+"${sources[@]}"} -o "$OUT/$name" 2> "$OUT/$name.log"; then
        cat "$OUT/$name.log"
        failed=1
        continue
    fi
    "$OUT/$name" || failed=1
done

for src in *_test.py; do
    name="${src%.py}"
    [[ -n "${1:-}" && "$name" != *"$1"* ]] && continue
    echo "== $name"
    python3 "$src" || failed=1
done

exit $failed
