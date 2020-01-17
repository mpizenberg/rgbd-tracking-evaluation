#!/usr/bin/env bash
# Bash3 Boilerplate. Copyright (c) 2014, kvz.io

set -o errexit
set -o pipefail
set -o nounset

# Build docker container with:
# docker build -t rgbd-track-eval -f dockerfiles/Dockerfile-ubuntu-16-04 .

# Mount docker container with:
# docker run --rm -it -v $DATASET:/dataset -v (pwd):/app rgbd-track-eval

cd /app/third-party/fovis
rm -rf build && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j2

cd /app/third-party/dvo
rm -rf build && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j2

cd /app
rm -rf build && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j2
