#!/usr/bin/env python

import os
import json
import csv
from pathlib import Path
from time import time
import pickle
import open3d

import cv2
import numpy as np
import rospy
import tf
from rospy.service import ServiceException
from cv_bridge import CvBridge
from geometry_msgs.msg import Transform
from sensor_msgs.msg import Image

from panoptic_mapping_msgs.srv import RenderCameraImage, GetBlockindex


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
        self.rendering_source = rospy.get_param("~rendering_source")
        self.id_filestart = '_'.join(
            os.listdir(self.inference_dir_path)[0].split('_')[:2])

        self.service_proxy = rospy.ServiceProxy(
            '/panoptic_mapper/render_camera_view', RenderCameraImage)
        self.blockidx_service_proxy = rospy.ServiceProxy(
            '/panoptic_mapper/get_blockindex', GetBlockindex)

        # Setup
        self.cv_bridge = CvBridge()

    def request_all_labels(self):
        current_index = 0
        # only relevant for blockindex
        blockindex_to_id = {}
        next_voxel_id = 1
        if self.rendering_source == 'geofeature':
            blockid_to_feature = {}
            with open(self.inference_dir_path / 'blockindex_to_id.pkl',
                      'rb') as f:
                blockindex_to_id = pickle.load(f)
            pointcloud = open3d.io.read_point_cloud(
                str(self.inference_dir_path / 'point_cloud_0.ply'))
            pointcloud = np.asarray(pointcloud.points)
            descriptors = np.load(
                self.inference_dir_path / 'smoothnet' / '32_dim' /
                'pc._0.150000_16_1.750000_3DSmoothNet.npz')['data']
            assert descriptors.shape[0] == pointcloud.shape[
                0], "descriptor and pointcloud do not match in shape"
            missed_count = 0
            for i in range(descriptors.shape[0]):
                response = self.blockidx_service_proxy(pointcloud[i, 0],
                                                       pointcloud[i, 1],
                                                       pointcloud[i, 2])
                blockindex = np.array([response.x, response.y, response.z],
                                      dtype=np.int32).data.tobytes()
                if blockindex not in blockindex_to_id:
                    missed_count += 1
                    continue
                blockid_to_feature[
                    blockindex_to_id[blockindex]] = descriptors[i]
            with open(self.inference_dir_path / 'blockid_to_descriptor.pkl',
                      'wb') as f:
                pickle.dump(blockid_to_feature, f)
            print(f'missed {missed_count} points')
            rospy.signal_shutdown("All features retrieved.")
            return

        while True:
            pose_filepath = self.data_dir_path / "pose" / f"{current_index:d}.txt"
            if not pose_filepath.is_file():
                rospy.logwarn(
                    f"Could not find file '{str(pose_filepath)}', shutting down."
                )
                if self.rendering_source == 'blockindex':
                    with open(self.inference_dir_path / 'blockindex_to_id.pkl',
                              'wb') as f:
                        pickle.dump(blockindex_to_id, f)
                rospy.signal_shutdown("Reached end of sequence.")
                return
            transform_msg = self._load_pose(pose_filepath)
            try:
                response = self.service_proxy(transform_msg,
                                              'pseudolabel_frame',
                                              self.rendering_source)
            except rospy.ServiceException as e:
                rospy.logwarn(
                    f"service exception, skipping frame {current_index}.")
                current_index += 1
                continue
            if self.rendering_source == 'id':
                pseudolabel = np.frombuffer(response.class_image.data,
                                            dtype=np.uint8).reshape(
                                                response.class_image.height,
                                                response.class_image.width)
            elif self.rendering_source == 'score':
                pseudolabel = np.frombuffer(response.class_image.data,
                                            dtype=np.float32).reshape(
                                                response.class_image.height,
                                                response.class_image.width)
            elif self.rendering_source == 'blockindex':
                blockindex = np.frombuffer(response.class_image.data,
                                           dtype=np.int32).reshape(
                                               response.class_image.height,
                                               response.class_image.width, 3)
                pseudolabel = np.zeros(
                    (response.class_image.height, response.class_image.width),
                    dtype='uint32')
                for u in range(response.class_image.height):
                    for v in range(response.class_image.width):
                        # take the blockindex in a hashable form
                        d = blockindex[u, v, :]
                        if d[0] == d[1] == d[2] == 0:
                            continue
                        idx = blockindex[u, v, :].data.tobytes()
                        if idx not in blockindex_to_id:
                            blockindex_to_id[idx] = next_voxel_id
                            next_voxel_id += 1
                        pseudolabel[u, v] = blockindex_to_id[idx]

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
