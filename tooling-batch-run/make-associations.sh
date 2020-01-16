#!/usr/bin/env bash

set -e # Make script exit when a command fail.
set -u # Exit on usage of undeclared variable.
# set -x # Trace what gets executed.
set -o pipefail # Catch failures in pipes.

script=$1
dataset_list=$(cat $2)

for dataset in $dataset_list
do
	echo Working on "$dataset"
	python ${script} $dataset/depth.txt $dataset/rgb.txt > $dataset/associations.txt
done
