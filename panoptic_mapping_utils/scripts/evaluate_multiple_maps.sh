#!/bin/bash
# Run this script to setup semantic labelling.

target_dir="/home/lukas/Documents/PanopticMapping/Data/evaluations/fixed_resolution"
target_files=("run2_2cm" "run2_5cm" "run2_10cm" "run2_20cm" "run2_multi")


# Evaluation
for target_file in "${target_files[@]}"; do
  roslaunch panoptic_mapping_utils evaluate_panmap.launch map_file:="$target_dir/$target_file.panmap" visualize:=false
done
