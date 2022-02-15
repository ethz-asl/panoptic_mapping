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
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import Image

from std_srvs.srv import Empty, EmptyResponse
from panoptic_mapping_msgs.msg import DetectronLabel, DetectronLabels


class ScannetV2DataPlayer:

    def __init__(self):
        # node params
        self.data_dir_path = Path(rospy.get_param("~data_path"))
        if not self.data_dir_path.exists():
            raise RuntimeError(f"Invalid data dir path: {self.data_dir_path}")
        self.inference_dir_path = Path(rospy.get_param("~inference_path"))
        if not self.inference_dir_path.exists():
            raise RuntimeError(f"Invalid inference dir path: {self.inference_dir_path}")
        self.id_fileending = rospy.get_param("~id_fileending")
        self.id_filestart = '_'.join(
            os.listdir(self.inference_dir_path)[0].split('_')[:2])

        self.global_frame_name = rospy.get_param("~global_frame_name", "world")
        self.sensor_frame_name = rospy.get_param("~sensor_frame_name",
                                                 "depth_cam")
        self.play_rate = rospy.get_param("~play_rate", 1.0 / 30)
        self.use_predicted_labels = rospy.get_param("~use_predicted_labels",
                                                    False)
        self.wait = rospy.get_param("~wait", False)

        # Configure sensor data publishers
        self.color_pub = rospy.Publisher("~color_image", Image, queue_size=100)
        self.depth_pub = rospy.Publisher("~depth_image", Image, queue_size=100)
        self.id_pub = rospy.Publisher("~id_image", Image, queue_size=100)
        self.pose_pub = rospy.Publisher("~pub", PoseStamped, queue_size=100)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Setup
        self.cv_bridge = CvBridge()
        self.current_index = 0
        self.start_time = None

        if self.wait:
            self.start_srv = rospy.Service("~start", Empty, self.start)
        else:
            self.start(None)

    def start(self, _):
        self.running = True
        self.timer = rospy.Timer(rospy.Duration(self.play_rate), self.callback)
        return EmptyResponse

    def _load_and_publish_color(self, color_image_file_path, timestamp):
        image = cv2.imread(str(color_image_file_path))
        img_msg = self.cv_bridge.cv2_to_imgmsg(image, "bgr8")
        img_msg.header.stamp = timestamp
        img_msg.header.frame_id = self.sensor_frame_name
        self.color_pub.publish(img_msg)

    def _load_and_publish_depth(self, depth_image_file_path, timestamp):
        raw_depth = cv2.imread(str(depth_image_file_path),
                               cv2.IMREAD_UNCHANGED)
        # Convert depth to meters
        depth = raw_depth.astype(np.float32) / 1000
        img_msg = self.cv_bridge.cv2_to_imgmsg(depth, "32FC1")
        img_msg.header.stamp = timestamp
        img_msg.header.frame_id = self.sensor_frame_name
        self.depth_pub.publish(img_msg)

    def _load_and_publish_id(self, id_filepath, timestamp):
        # Load instance and publish instance
        id_map = np.load(str(id_filepath))
        if id_map.dtype != 'uint8':
            rospy.logwarn(f"ID is of type {id_map.dtype}, converting to uint8.")
            id_map = id_map.astype('uint8')
        img_msg = self.cv_bridge.cv2_to_imgmsg(id_map, "8UC1")
        img_msg.header.stamp = timestamp
        img_msg.header.frame_id = self.sensor_frame_name
        self.id_pub.publish(img_msg)

    def _load_and_publish_pose(self, pose_file_path, timestamp):
        # Load the transformation matrix from a text file
        pose_mat = np.loadtxt(str(pose_file_path))
        # Extract rotation and translation
        rotation = tf.transformations.quaternion_from_matrix(pose_mat)
        translation = tuple(pose_mat[0:3, -1])

        self.tf_broadcaster.sendTransform(
            translation=translation,
            rotation=rotation,
            time=timestamp,
            child=self.sensor_frame_name,
            parent=self.global_frame_name,
        )

        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = self.global_frame_name
        pose_msg.pose.position = Point(**{
            "x": translation[0],
            "y": translation[1],
            "z": translation[2],
        })
        pose_msg.pose.orientation = Quaternion(
            **{
                "x": rotation[0],
                "y": rotation[1],
                "z": rotation[2],
                "w": rotation[3],
            })
        self.pose_pub.publish(pose_msg)

    def callback(self, _):
        if not self.running:
            return
        now = rospy.Time.now()
        if self.start_time is None:
            self.start_time = now
        # Load and publish the next image
        self.current_index += 1
        color_filepath = self.data_dir_path / "color" / f"{self.current_index:d}.jpg"
        depth_filepath = self.data_dir_path / "depth" / f"{self.current_index:d}.png"
        pose_filepath = self.data_dir_path / "pose" / f"{self.current_index:d}.txt"
        idmap_filepath = (
            self.inference_dir_path /
            f"{self.id_filestart}_{self.current_index:06d}_{self.id_fileending}.npy"
        )
        for f in [
                color_filepath, depth_filepath, pose_filepath, idmap_filepath
        ]:
            if not f.is_file():
                rospy.logwarn(
                    f"Could not find file '{str(f)}', shutting down.")
                rospy.sleep(5)
                rospy.signal_shutdown("Player reached end of sequence.")
                return
        self._load_and_publish_color(color_filepath, now)
        self._load_and_publish_id(idmap_filepath, now)
        self._load_and_publish_depth(depth_filepath, now)
        self._load_and_publish_pose(pose_filepath, now)


if __name__ == "__main__":
    rospy.init_node("scannetv2_data_player")
    scannetv2_data_player = ScannetV2DataPlayer()
    rospy.spin()
