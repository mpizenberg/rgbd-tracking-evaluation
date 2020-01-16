#!/usr/bin/env bash

set -e # Make script exit when a command fail.
set -u # Exit on usage of undeclared variable.
# set -x # Trace what gets executed.
set -o pipefail # Catch failures in pipes.

algo=$1
dataset_list=$(cat $2)
output_dir=$3
output_name=$4
args="${@:5}"

for dataset in $dataset_list
do
	dataset_name=$(basename -- "${dataset}")
	echo "Running $algo $args ${dataset}/associations.txt > ${output_dir}/${dataset_name}-${output_name}"
	$algo $args ${dataset}/associations.txt > ${output_dir}/${dataset_name}-${output_name}
done
