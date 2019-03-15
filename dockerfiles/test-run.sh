#!/usr/bin/env bash
# Bash3 Boilerplate. Copyright (c) 2014, kvz.io

set -o errexit
set -o pipefail
set -o nounset

# Build docker container with:
# docker build -t rgbd-track-eval -f dockerfiles/Dockerfile-ubuntu-16-04 .

# Mount docker container with:
# docker run --rm -it -v $DATASET:/dataset -v (pwd):/app rgbd-track-eval

DATASET=/dataset

cd /app/tooling
python associate.py $DATASET/depth.txt $DATASET/rgb.txt > $DATASET/associations.txt

cd /app/build
echo "Running DVO ..."
bin/dvo_track fr1 $DATASET/associations.txt > $DATASET/trajectory-dvo.txt
echo "Running Fovis ..."
bin/fovis_track fr1 $DATASET/associations.txt > $DATASET/trajectory-fovis.txt
echo "Running OpenCV ..."
bin/ocv_track Rgbd fr1 $DATASET/associations.txt > $DATASET/trajectory-ocv.txt
echo "Running vors ..."
bin/vors_track fr1 $DATASET/associations.txt > $DATASET/trajectory-vors.txt

cd /app/tooling
python evaluate_rpe.py --verbose --fixed_delta --delta_unit s --delta 1 $DATASET/groundtruth.txt $DATASET/trajectory-dvo.txt
python evaluate_rpe.py --verbose --fixed_delta --delta_unit s --delta 1 $DATASET/groundtruth.txt $DATASET/trajectory-fovis.txt
python evaluate_rpe.py --verbose --fixed_delta --delta_unit s --delta 1 $DATASET/groundtruth.txt $DATASET/trajectory-ocv.txt
python evaluate_rpe.py --verbose --fixed_delta --delta_unit s --delta 1 $DATASET/groundtruth.txt $DATASET/trajectory-vors.txt
