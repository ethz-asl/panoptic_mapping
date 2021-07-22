#!/bin/bash
# Run this script to setup semantic labelling.

target_dir="/home/lukas/Documents/PanopticMapping/pan_single_tsdf"
extension=".panmap"
target_files=("5cm" "10cm" "20cm")


# Evaluation
for target_file in "${target_files[@]}"; do
  roslaunch panoptic_mapping_utils evaluate_panmap.launch map_file:="$target_dir/$target_file$extension" visualize:=false
done
