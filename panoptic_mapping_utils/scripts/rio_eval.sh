#!/bin/bash
# Run this script to setup semantic labelling.

target_dir="/home/lukas/Documents/PanopticMapping/Rio"
target_files=('single_0' 'single_1' 'single_2_with_map' 'single_3' 'single_2' 'single_1_with_map' 'single_3_with_map')


# Evaluation
for target_file in "${target_files[@]}"; do
  roslaunch panoptic_mapping_utils evaluate_series.launch map_file:="$target_dir/$target_file" visualize:=false ground_truth_pointcloud_file:="/home/lukas/Documents/Datasets/3RScan/scene0.ply" output_suffix:="eval_complete"
done

roslaunch panoptic_mapping_utils evaluate_series.launch map_file:="$target_dir/single_0" visualize:=false ground_truth_pointcloud_file:="/home/lukas/Documents/Datasets/3RScan/0cac7578-8d6f-2d13-8c2d-bfa7a04f8af3/gt_10000.ply" output_suffix:="eval_local"

roslaunch panoptic_mapping_utils evaluate_series.launch map_file:="$target_dir/single_1" visualize:=false ground_truth_pointcloud_file:="/home/lukas/Documents/Datasets/3RScan/2451c041-fae8-24f6-9213-b8b6af8d86c1/gt_10000.ply" output_suffix:="eval_local"

roslaunch panoptic_mapping_utils evaluate_series.launch map_file:="$target_dir/single_1_with_map" visualize:=false ground_truth_pointcloud_file:="/home/lukas/Documents/Datasets/3RScan/2451c041-fae8-24f6-9213-b8b6af8d86c1/gt_10000.ply" output_suffix:="eval_local"

roslaunch panoptic_mapping_utils evaluate_series.launch map_file:="$target_dir/single_2" visualize:=false ground_truth_pointcloud_file:="/home/lukas/Documents/Datasets/3RScan/ddc73793-765b-241a-9ecd-b0cebb7cf916/gt_10000.ply" output_suffix:="eval_local"

roslaunch panoptic_mapping_utils evaluate_series.launch map_file:="$target_dir/single_2_with_map" visualize:=false ground_truth_pointcloud_file:="/home/lukas/Documents/Datasets/3RScan/ddc73793-765b-241a-9ecd-b0cebb7cf916/gt_10000.ply" output_suffix:="eval_local"

roslaunch panoptic_mapping_utils evaluate_series.launch map_file:="$target_dir/single_3" visualize:=false ground_truth_pointcloud_file:="/home/lukas/Documents/Datasets/3RScan/ddc73795-765b-241a-9c5d-b97744afe077/gt_10000.ply" output_suffix:="eval_local"

roslaunch panoptic_mapping_utils evaluate_series.launch map_file:="$target_dir/single_3_with_map" visualize:=false ground_truth_pointcloud_file:="/home/lukas/Documents/Datasets/3RScan/ddc73795-765b-241a-9c5d-b97744afe077/gt_10000.ply" output_suffix:="eval_local"
