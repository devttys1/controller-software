#!/usr/bin/env bash
# Build controller_core with the ZYNQ-Debug preset inside the Docker cross toolchain.
# The core directory is mounted at the same path in the container, so build/ is
# interchangeable with a native Linux build and the post-build copy into
# zynq_files/ works as usual.
#
# usage: ./docker_build.sh [extra cmake --build args, e.g. --clean-first]
set -euo pipefail

CORE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE=em-zynq-toolchain

docker build --quiet -t "$IMAGE" "$CORE/docker" >/dev/null    # cached unless the Dockerfile changed

docker run --rm \
    -v "$CORE:$CORE" -w "$CORE" \
    --user "$(id -u):$(id -g)" \
    "$IMAGE" \
    sh -c 'cmake --preset ZYNQ-Debug >/dev/null && cmake --build --preset ZYNQ-Debug "$@"' sh "$@"
