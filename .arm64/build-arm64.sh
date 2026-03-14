#!/bin/bash
set -e
cd /root/ws
mkdir -p build-arm64

docker run --rm --privileged tonistiigi/binfmt --install all
docker buildx build \
    --platform linux/arm64 \
    -t gar_rpi_gauge_builder:arm64 \
    -f .arm64/Dockerfile.arm64 \
    --output type=local,dest=build-arm64 \
    --load \
    .
mv build-arm64/install.tar.gz build-arm64/install_$(date +%Y%m%d_%H%M%S).tar.gz