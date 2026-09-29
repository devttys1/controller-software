# Cross toolchain for controller_core, for hosts without one (e.g. macOS).
# Debian bookworm's arm-linux-gnueabihf toolchain (gcc 12.2, glibc 2.36) matches the
# PetaLinux 2023.1 image on the Zynq, so the binary links against the same runtime.
# The native g++ and python3 run the host tests (core/tests), which use Linux-only pthread calls
# and, for the monitor web server, Linux shared memory.
FROM debian:bookworm-slim

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        g++-arm-linux-gnueabihf \
        g++ \
        cmake \
        ninja-build \
        python3 \
    && rm -rf /var/lib/apt/lists/*
