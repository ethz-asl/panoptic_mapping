#!/usr/bin/env python

import json
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import cv2
import numpy as np
import rospy
import tf
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from PIL import Image as PilImage
from sensor_msgs.msg import Image

from std_srvs.srv import Empty, EmptyResponse
from panoptic_mapping_msgs.msg import DetectronLabel, DetectronLabels

_COLOR_IMAGES_DIR = "color"
_DEPTH_IMAGES_DIR = "depth"
_PANOPTIC_PRED_DIR = "panoptic_pred"
_POSES_DIR = "pose"
_DEPTH_SHIFT = 1000


class ScannetV2DataPlayer:
    def __init__(self):

        # node params
        self.scans_dir_path = Path(rospy.get_param("~scans_dir_path"))
        self.scan_id = rospy.get_param("~scan_id")
        self.scan_dir_path = self.scans_dir_path / self.scan_id
        if not self.scan_dir_path.exists():
            raise RuntimeError("Invalid scan dir path!")

        self.global_frame_name = rospy.get_param("~global_frame_name", "world")
        self.sensor_frame_name = rospy.get_param("~sensor_frame_name", "depth_cam")
        self.play_rate = rospy.get_param("~play_rate", 1.0)
        self.wait = rospy.get_param("~wait", False)
        self.frame_skip = rospy.get_param("~frame_skip", 0)
        self.refresh_rate = 30  # Hz
        self.use_uncertainty = rospy.get_param("~use_uncertainty", False)

        # Configure sensor data publishers
        self.color_pub = rospy.Publisher("~color_image", Image, queue_size=100)
        self.depth_pub = rospy.Publisher("~depth_image", Image, queue_size=100)
        self.id_pub = rospy.Publisher("~id_image", Image, queue_size=100)
        if self.use_uncertainty:
            self.uncertainty_pub = rospy.Publisher(
                "~uncertainty_image", Image, queue_size=100
            )
        self.label_pub = rospy.Publisher("~labels", DetectronLabels, queue_size=100)
        self.pose_pub = rospy.Publisher("~pose", PoseStamped, queue_size=100)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Setup
        self.cv_bridge = CvBridge()

        self.current_index = 0
        self.times = []
        self.ids = []
        timestamps_file_path = self.scan_dir_path / "timestamps.csv"
        if not timestamps_file_path.is_file():
            rospy.logfatal("Timestamps file not found!")
        with timestamps_file_path.open("r") as f:
            csv_reader = csv.reader(f)
            for row in csv_reader:
                if row[0] == "FrameID":
                    continue
                self.ids.append(str(row[0]))
                self.times.append(float(row[1]) / 1e9)

        self.ids = [x for _, x in sorted(zip(self.times, self.ids))]
        self.times = sorted(self.times)

        # Drop frames
        if self.frame_skip > 0:
            self.ids = [
                x for idx, x in enumerate(self.ids) if idx % self.frame_skip == 0
            ]
            self.times = [
                x for idx, x in enumerate(self.times) if idx % self.frame_skip == 0
            ]

        self.times = [(x - self.times[0]) / self.play_rate for x in self.times]
        self.start_time = None

        if self.wait:
            self.start_srv = rospy.Service("~start", Empty, self.start)
        else:
            self.start(None)

    def start(self, _):
        self.running = True
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.refresh_rate), self.callback)
        return EmptyResponse

    def _load_and_publish_color(self, color_image_file_path, timestamp):
        image = cv2.imread(str(color_image_file_path))
        img_msg = self.cv_bridge.cv2_to_imgmsg(image, "bgr8")
        img_msg.header.stamp = timestamp
        img_msg.header.frame_id = self.sensor_frame_name
        self.color_pub.publish(img_msg)

    def _load_and_publish_depth(self, depth_image_file_path, timestamp):
        raw_depth = np.array(PilImage.open(str(depth_image_file_path)))
        # Convert depth to meters
        depth = raw_depth.astype(np.float32) / _DEPTH_SHIFT
        img_msg = self.cv_bridge.cv2_to_imgmsg(depth, "32FC1")
        img_msg.header.stamp = timestamp
        img_msg.header.frame_id = self.sensor_frame_name
        self.depth_pub.publish(img_msg)

    def _load_and_publish_instance(self, instance_file_path, timestamp):
        # Load instance and publish instance
        instance_map = np.array(PilImage.open(instance_file_path))
        img_msg = self.cv_bridge.cv2_to_imgmsg(instance_map, "8UC1")
        img_msg.header.stamp = timestamp
        img_msg.header.frame_id = self.sensor_frame_name
        self.id_pub.publish(img_msg)

    def _load_and_publish_labels(self, label_file_path, timestamp):
        label_msg = DetectronLabels()
        label_msg.header.stamp = timestamp
        with label_file_path.open("r") as f:
            data = json.load(f)
            for d in data:
                if "instance_id" not in d:
                    d["instance_id"] = 0
                if "score" not in d:
                    d["score"] = 0
                label = DetectronLabel()
                label.id = d["id"]
                label.instance_id = d["instance_id"]
                label.is_thing = d["isthing"]
                label.category_id = d["category_id"]
                label.score = d["score"]
                label_msg.labels.append(label)
        self.label_pub.publish(label_msg)

    def _load_and_publish_uncertainty(self, uncertainty_image_file_path, timestamp):
        if not uncertainty_image_file_path.exists():
            rospy.logwarn(
                f"Uncertainty image file {uncertainty_image_file_path} not found!"
            )
            return

        uncertainty_image = np.array(PilImage.open(str(uncertainty_image_file_path)))
        # Convert depth to meters
        img_msg = self.cv_bridge.cv2_to_imgmsg(uncertainty_image, "32FC1")
        img_msg.header.stamp = timestamp
        img_msg.header.frame_id = self.sensor_frame_name
        self.uncertainty_pub.publish(img_msg)

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
        pose_msg.pose.position = Point(
            **{
                "x": translation[0],
                "y": translation[1],
                "z": translation[2],
            }
        )
        pose_msg.pose.orientation = Quaternion(
            **{
                "x": rotation[0],
                "y": rotation[1],
                "z": rotation[2],
                "w": rotation[3],
            }
        )
        self.pose_pub.publish(pose_msg)

    def callback(self, _):

        if not self.running:
            return

        now = rospy.Time.now()
        if self.start_time is None:
            self.start_time = now

        if self.times[self.current_index] > (now - self.start_time).to_sec():
            return

        # Load and publish the next image
        frame_id = self.ids[self.current_index]

        color_image_file_path = (
            self.scan_dir_path / _COLOR_IMAGES_DIR / "{:05d}.jpg".format(int(frame_id))
        )
        depth_image_file_path = (
            self.scan_dir_path / _DEPTH_IMAGES_DIR / "{:05d}.png".format(int(frame_id))
        )
        pose_file_path = (
            self.scan_dir_path / _POSES_DIR / "{:05d}.txt".format(int(frame_id))
        )
        instance_map_file_path = (
            self.scan_dir_path
            / _PANOPTIC_PRED_DIR
            / "{:05d}_predicted.png".format(int(frame_id))
        )
        labels_file_path = (
            self.scan_dir_path
            / _PANOPTIC_PRED_DIR
            / "{:05d}_labels.json".format(int(frame_id))
        )

        self._load_and_publish_color(color_image_file_path, now)
        self._load_and_publish_instance(instance_map_file_path, now)
        self._load_and_publish_labels(labels_file_path, now)
        self._load_and_publish_depth(depth_image_file_path, now)
        self._load_and_publish_pose(pose_file_path, now)

        if self.use_uncertainty:
            uncertainty_image_file_path = (
                self.scan_dir_path
                / _PANOPTIC_PRED_DIR
                / "{:05d}_uncertainty.tiff".format(int(frame_id))
            )
            self._load_and_publish_uncertainty(uncertainty_image_file_path, now)

        self.current_index += 1
        if self.current_index >= len(self.ids):
            rospy.signal_shutdown("Player reached end of sequence.")


if __name__ == "__main__":
    rospy.init_node("scannetv2_data_player")
    scannetv2_data_player = ScannetV2DataPlayer()
    rospy.spin()
