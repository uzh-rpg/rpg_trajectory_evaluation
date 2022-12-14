#!/bin/bash

file=$1
results_dir=$2

for d in ${results_dir}* ; do
    cp $file $d
done