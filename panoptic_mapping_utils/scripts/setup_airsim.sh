#!/bin/bash
# Run this script to setup semantic labelling.

python ~/catkin_ws/src/unreal_airsim/app/tools/compute_infrared_compensation.py -d /home/lukas/Documents/PanopticMapping/Data -c Id_cam &
BACK_PID=$!

while kill -0 $BACK_PID ; do
    # Waiting for the previous command to finish.
    sleep 0.5
done

python /home/lukas/catkin_ws/src/panoptic_mapping/panoptic_mapping_ros/app/tools/flat_semantic_labelling.py
