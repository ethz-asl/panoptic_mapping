#!/usr/bin/env python

import os
import json
import csv
from pathlib import Path
from time import time

import cv2
import numpy as np
import rospy
import tf
from rospy.service import ServiceException
from cv_bridge import CvBridge
from geometry_msgs.msg import Transform
from sensor_msgs.msg import Image

from panoptic_mapping_msgs.srv import RenderCameraImage


class ScannetV2Pseudolabeller:

    def __init__(self):
        # node params
        self.data_dir_path = Path(rospy.get_param("~data_path"))
        if not self.data_dir_path.exists():
            raise RuntimeError(f"Invalid data dir path: {self.data_dir_path}")
        self.inference_dir_path = Path(rospy.get_param("~inference_path"))
        if not self.inference_dir_path.exists():
            raise RuntimeError(
                f"Invalid inference dir path: {self.inference_dir_path}")
        self.mapname = rospy.get_param("~mapname")
        self.id_filestart = '_'.join(
            os.listdir(self.inference_dir_path)[0].split('_')[:2])

        self.service_proxy = rospy.ServiceProxy('/panoptic_mapper/render_camera_view', RenderCameraImage)

        # Setup
        self.cv_bridge = CvBridge()

    def request_all_labels(self):
        current_index = 0

        while True:
            pose_filepath = self.data_dir_path / "pose" / f"{current_index:d}.txt"
            if not pose_filepath.is_file():
                rospy.logwarn(
                    f"Could not find file '{str(pose_filepath)}', shutting down.")
                rospy.signal_shutdown("Reached end of sequence.")
                return
            transform_msg = self._load_pose(pose_filepath)
            response = self.service_proxy(transform_msg, 'pseudolabel_frame')
            pseudolabel = np.frombuffer(response.class_image.data,
                                        dtype=np.uint8).reshape(
                                            response.class_image.height,
                                            response.class_image.width)
            pseudolabel_path = (
                self.inference_dir_path /
                f"{self.id_filestart}_{current_index:06d}_pseudolabel-{self.mapname}.npy"
            )
            np.save(str(pseudolabel_path), pseudolabel)
            current_index += 1


    def _load_pose(self, pose_file_path):
        # Load the transformation matrix from a text file
        pose_mat = np.loadtxt(str(pose_file_path))
        # Extract rotation and translation
        rotation = tf.transformations.quaternion_from_matrix(pose_mat)
        translation = tuple(pose_mat[0:3, -1])

        transform_msg = Transform()
        transform_msg.translation.x = translation[0]
        transform_msg.translation.y = translation[1]
        transform_msg.translation.z = translation[2]
        transform_msg.rotation.x = rotation[0]
        transform_msg.rotation.y = rotation[1]
        transform_msg.rotation.z = rotation[2]
        transform_msg.rotation.w = rotation[3]
        return transform_msg

if __name__ == "__main__":
    rospy.init_node("scannetv2_pseudolabeller")
    pseudolabeller = ScannetV2Pseudolabeller()
    pseudolabeller.request_all_labels()
    rospy.spin()
