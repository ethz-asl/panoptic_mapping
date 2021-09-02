#!/bin/bash

# ================ Args ================
# General args.
target_map_dir="/home/lukas/Documents/PanopticMapping/Data/maps/exp2"
evaluate_run_1=true
method="detectron" # gt, detectron, single, single_with_map, fusion
visualize=false

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
    target_data_dir="/home/lukas/Documents/PanopticMapping/Exp2/gt"
    config="exp2_gt"
    use_detectron="false"
  elif [ $method == "detectron" ]
  then
    echo "Using Detectron config"
    target_data_dir="/home/lukas/Documents/PanopticMapping/Rio/Exp2/detectron"
    config="exp2_detectron"
    use_detectron="true"
  elif [ $method == "single" ]
  then
    echo "Using Single TSDF config"
    target_data_dir="/home/lukas/Documents/PanopticMapping/Exp2/single"
    config="exp2_single"
    use_detectron="false"
  elif [ $method == "single_with_map" ]
  then
    echo "Using Single TSDF with map config"
    target_data_dir="/home/lukas/Documents/PanopticMapping/Exp2/single_with_map"
    config="exp2_single"
    use_detectron="false"
  elif [ $method == "fusion" ]
  then
    echo "Using Single TSDF with long term fusion config"
    target_data_dir="/home/lukas/Documents/PanopticMapping/Exp2/fusion"
    config="exp2_fusion"  # Not existent yet!
    use_detectron="false"
  else
    echo "Unknown method '$method', exiting."
    return
  fi
  if [[ $evaluate_run_1 == true ]]
  then
    no_scans=2
  else
    no_scans=1
  fi
}

run_experiments() {
  if [[ $evaluate_run_1 == true ]]
  then
    roslaunch panoptic_mapping_ros run.launch use_rio:=false use_detectron:=$use_detectron  load_map:=false config:=$config save_map:=$target_map_dir/$method-run1 visualize:=$visualize base_path:="/home/lukas/Documents/Datasets/flat_dataset/run1"
  fi
  
  if [[ $method == "single" ]]
  then
    load="false"
  else
    load="true"
  fi
  roslaunch panoptic_mapping_ros run.launch use_rio:=false use_detectron:=$use_detectron  load_map:=$load config:=$config save_map:=$target_map_dir/$method-run2 visualize:=$visualize base_path:="/home/lukas/Documents/Datasets/flat_dataset/run2" load_file:=$target_map_dir/$method-run1
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
  for (( i=1; i<=$no_scans; i++ ))
  do
    roslaunch panoptic_mapping_utils evaluate_series.launch map_file:=$target_data_dir/pan_$((i-1)) visualize:=false ground_truth_pointcloud_file:="/home/lukas/Documents/Datasets/flat_dataset/ground_truth/run$i/flat_${i}_gt_10000.ply" output_suffix:="evaluation_data" is_single_tsdf:=$is_single_tsdf
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
