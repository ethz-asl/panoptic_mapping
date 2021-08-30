#!/bin/bash

# Args
target_map_dir="/home/lukas/Documents/PanopticMapping/Data/maps/rio"
scene_id=0
no_scans=4  # 0: 4, 
method="gt" # gt, detectron, single, single_with_map

run_experiments=true
move_logs=false
evaluate=false


# Auto-generated args
if [ $method == "gt" ]
then
  echo "Using GT config"
  target_data_dir="/home/lukas/Documents/PanopticMapping/Rio/gt"
  config="rio_gt"
  use_detectron="false"
elif [ $method == "detectron" ]
then
  echo "Using Detectron config"
  target_data_dir="/home/lukas/Documents/PanopticMapping/Rio/detectron"
  config="rio_detectron"
  use_detectron="true"
elif [ $method == "single" ]
then
  echo "Using Single TSDF config"
  target_data_dir="/home/lukas/Documents/PanopticMapping/Rio/single"
  config="rio_single"
  use_detectron="false"
elif [ $method == "single_with_map" ]
then
  echo "Using Single TSDF with map config"
  target_data_dir="/home/lukas/Documents/PanopticMapping/Rio/single_with_map"
  config="rio_single"
  use_detectron="false"
else
  echo "Unknown method '$method', exiting."
  return
fi


# Run experiments.
if [ $run_experiments = true ]
then
  for ((i=0; i<$no_scans; i++ ))
  do
    if [[ $i -eq 0 || $method == "single" ]]
    then
      load="false"
    else
      load="true"
    fi
    roslaunch panoptic_mapping_ros run.launch use_rio:=true use_detectron:=$use_detectron scene_id:=$scene_id scan_id:=$i load_map:=$load config:=$config save_map:=$target_map_dir/$config-$i load_file:=$target_map_dir/$config-$(($i-1))
  done
fi

# Move the evaluation logs
if [ $move_logs = true ]
then
  python /home/lukas/catkin_ws/src/panoptic_mapping/panoptic_mapping_utils/src/plotting/move_rio_logs.py /home/lukas/Documents/PanopticMapping/Data/run_logs $target_data_dir/scene_$scene_id $no_scans
fi

# Evaluate the data.
if [ $evaluate = true ]
then
  for ((i=0; i<$no_scans; i++ ))
  do
    roslaunch panoptic_mapping_utils evaluate_series.launch map_file:="$target_data_dir/pan_$i" visualize:=false ground_truth_pointcloud_file:="/home/lukas/Documents/Datasets/3RScan/scene$scene_id.ply" output_suffix:="eval_complete"
   
    roslaunch panoptic_mapping_utils evaluate_series.launch map_file:="$target_data_dir/pan_0" visualize:=false use_rio:=true scene_id:=$scene_id scan_id:=$i output_suffix:="eval_local"
  done
fi
