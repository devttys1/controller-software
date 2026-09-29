# Runtime for Vivado 2023.1 in batch mode on hosts it does not support (e.g. Ubuntu 24.04).
# Vivado itself is not in the image: it is installed on the host (e.g. ~/Xilinx) and
# mounted at the same path, so installing it and building with it use one environment.
FROM ubuntu:22.04

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ca-certificates \
        locales \
        libtinfo5 \
        libncurses5 \
        libx11-6 \
        libxext6 \
        libxrender1 \
        libxtst6 \
        libxi6 \
        libfreetype6 \
        fontconfig \
        libglib2.0-0 \
        libsm6 \
        procps \
        make \
        gcc \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8

ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
