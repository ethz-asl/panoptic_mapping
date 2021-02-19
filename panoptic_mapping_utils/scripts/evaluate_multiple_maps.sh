#!/bin/bash
# Run this script to setup semantic labelling.

target_dir="/home/lukas/Documents/PanopticMapping/Data/evaluations/fixed_resolution"
target_files=("run1_2cm" "run1_5cm" "run1_10cm" "run1_20cm" "run1_multi")


# Evaluation
for target_file in "${target_files[@]}"; do
  roslaunch panoptic_mapping_utils evaluate_panmap.launch map_file:="$target_dir/$target_file.panmap" visualize:=false
done
