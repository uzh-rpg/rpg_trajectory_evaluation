#!/bin/bash

results_dir=$1

for d in ${results_dir}* ; do
    python2 rpg_trajectory_evaluation/scripts/analyze_trajectory_single.py $d 
done