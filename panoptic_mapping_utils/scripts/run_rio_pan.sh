#!/bin/bash

# ================ Args ================
# General args.
target_map_dir="/home/lukas/Documents/PanopticMapping/Data/maps/rio"
scene_id=2
no_scans=2  # 0:4, 1:4, 2:2
method="detectron" # gt, detectron, single, single_with_map, fusion
visualize=true

# What to evaluate.
run_all_methods=true
run_experiments=true
evaluate=true
remove_maps=true

# ================ Functions ================
auto_generate_args() {
  if [ $method == "gt" ]
  then
    echo "Using GT config"
    target_data_dir="/home/lukas/Documents/PanopticMapping/Rio/scene_$scene_id/gt"
    config="rio_gt"
    use_detectron="false"
  elif [ $method == "detectron" ]
  then
    echo "Using Detectron config"
    target_data_dir="/home/lukas/Documents/PanopticMapping/Rio/scene_$scene_id/detectron"
    config="rio_detectron"
    use_detectron="true"
  elif [ $method == "single" ]
  then
    echo "Using Single TSDF config"
    target_data_dir="/home/lukas/Documents/PanopticMapping/Rio/scene_$scene_id/single"
    config="rio_single"
    use_detectron="false"
  elif [ $method == "single_with_map" ]
  then
    echo "Using Single TSDF with map config"
    target_data_dir="/home/lukas/Documents/PanopticMapping/Rio/scene_$scene_id/single_with_map"
    config="rio_single"
    use_detectron="false"
  elif [ $method == "fusion" ]
  then
    echo "Using Single TSDF with long term fusion config"
    target_data_dir="/home/lukas/Documents/PanopticMapping/Rio/scene_$scene_id/fusion"
    config="rio_fusion"
    use_detectron="false"
  else
    echo "Unknown method '$method', exiting."
    return
  fi
}

run_experiments() {
  for ((i=0; i<$no_scans; i++ ))
  do
    if [[ $i -eq 0 || $method == "single" ]]
    then
      load="false"
    else
      load="true"
    fi
    roslaunch panoptic_mapping_ros run.launch use_rio:=true use_detectron:=$use_detectron scene_id:=$scene_id scan_id:=$i load_map:=$load config:=$config save_map:=$target_map_dir/scene_$scene_id/$method$i load_file:=$target_map_dir/scene_$scene_id/$method$(($i-1)) visualize:=$visualize
  done
}

move_experiment_logs() {
  python /home/lukas/catkin_ws/src/panoptic_mapping/panoptic_mapping_utils/src/plotting/move_rio_logs.py /home/lukas/Documents/PanopticMapping/Data/run_logs $target_data_dir $no_scans
}

evaluate_experiments() {
  if [[ $method == "single" || $method == "single_with_map" || $method == "fusion" ]]
  then
    is_single_tsdf="true"
  else
    is_single_tsdf="false"
  fi
  for ((i=0; i<$no_scans; i++ ))
  do
    roslaunch panoptic_mapping_utils evaluate_series.launch map_file:="$target_data_dir/pan_$i" visualize:=false ground_truth_pointcloud_file:="/home/lukas/Documents/Datasets/3RScan/scene$scene_id.ply" output_suffix:="eval_complete" is_single_tsdf:=$is_single_tsdf
   
    roslaunch panoptic_mapping_utils evaluate_series.launch map_file:="$target_data_dir/pan_$i" visualize:=false use_rio:=true scene_id:=$scene_id scan_id:=$i output_suffix:="eval_local" is_single_tsdf:=$is_single_tsdf
  done
}

remove_maps() {
  find $target_data_dir -name "*.panmap" -type f -delete
}

run_evaluations() {
  auto_generate_args
  if [ $run_experiments = true ]
  then
    run_experiments
    move_experiment_logs
  fi
  if [ $evaluate = true ]
  then
    evaluate_experiments
  fi
  if [ $remove_maps = true ]
  then
    remove_maps
  fi
}

# ================ Run evaluations ================
if [ $run_all_methods == true ]
then
  method="gt"
  run_evaluations
  method="detectron" 
  run_evaluations
  method="single"
  run_evaluations
  method="single_with_map"
  run_evaluations
  method="fusion"
  run_evaluations
else
  run_evaluations
fi
