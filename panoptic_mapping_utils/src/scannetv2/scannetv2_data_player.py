#!/usr/bin/env python

import json
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import List, Dict, Tuple, Optional

import cv2
import numpy as np
import rospy
import tf
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from PIL import Image as PilImage
from std_msgs.msg import Header
from sensor_msgs.msg import Image

from std_srvs.srv import Empty, EmptyResponse
from panoptic_mapping_msgs.msg import DetectronLabel, DetectronLabels

_COLOR_IMAGES_DIR = "color"
_DEPTH_IMAGES_DIR = "depth"
_LABEL_DIVISOR = 1000
_PANOPTIC_GROUNDTRUTH_DIR = "panoptic"
_PANOPTIC_PRED_DIR = "panoptic_pred"
_POSES_DIR = "pose"
_DEPTH_SHIFT = 1000
_INPUT_IMAGE_DIMS = (320, 240)


@dataclass
class FrameData:
    color: np.ndarray
    depth: np.ndarray
    pose: np.ndarray
    segmentation: np.ndarray
    segments_info: List[Dict]
    uncertainty: Optional[np.ndarray] = None


def make_msg_header(self, timestamp, frame_id: str):
    return Header(
        stamp=timestamp,
        frame_id=frame_id,
    )


class FrameDataPublisher:
    def __init__(
        self,
        sensor_frame_name: str,
        global_frame_name: str,
        use_uncertainty=False,
    ):
        self.sensor_frame_name = sensor_frame_name
        self.global_frame_name = global_frame_name

        self.cv_bridge = CvBridge()
        self.color_pub = rospy.Publisher("~color_image", Image, queue_size=100)
        self.depth_pub = rospy.Publisher("~depth_image", Image, queue_size=100)
        self.id_pub = rospy.Publisher("~id_image", Image, queue_size=100)
        self.use_uncertainty = use_uncertainty
        if self.use_uncertainty:
            self.uncertainty_pub = rospy.Publisher(
                "~uncertainty_image", Image, queue_size=100
            )
        self.label_pub = rospy.Publisher("~labels", DetectronLabels, queue_size=100)
        self.pose_pub = rospy.Publisher("~pose", PoseStamped, queue_size=100)
        self.tf_broadcaster = tf.TransformBroadcaster()

    def publish(self, frame_data: FrameData, timestamp):
        # Color
        color_img_msg = self.cv_bridge.cv2_to_imgmsg(frame_data.color, "bgr8")
        color_img_msg.header = make_msg_header(timestamp, self.sensor_frame_name)
        self.color_pub.publish(color_img_msg)

        # Depth
        depth_img_msg = self.cv_bridge.cv2_to_imgmsg(frame_data.depth, "32FC1")
        depth_img_msg.header = make_msg_header(timestamp, self.sensor_frame_name)
        self.depth_pub.publish(depth_img_msg)

        # Segmentation
        id_img_msg = self.cv_bridge.cv2_to_imgmsg(frame_data.segmentation, "32SC1")
        id_img_msg.header = make_msg_header(timestamp, self.sensor_frame_name)
        self.id_pub.publish(id_img_msg)

        # Segments info (Detectron labels)
        label_msg = DetectronLabels()
        label_msg.header = make_msg_header(timestamp, self.sensor_frame_name)
        for d in frame_data.segments_info:
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

        # Transform and pose
        rotation = tf.transformations.quaternion_from_matrix(frame_data.pose)
        translation = tuple(frame_data.pose[0:3, -1])

        self.tf_broadcaster.sendTransform(
            translation=translation,
            rotation=rotation,
            time=timestamp,
            child=self.sensor_frame_name,
            parent=self.global_frame_name,
        )

        pose_msg = PoseStamped()
        pose_msg.header = make_msg_header(timestamp, self.global_frame_name)
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

        # Dense semantic uncertainty
        if self.use_uncertainty:
            uncertainty_img_msg = self.cv_bridge.cv2_to_imgmsg(
                frame_data.uncertainty, "32FC1"
            )
            uncertainty_img_msg.header.stamp = timestamp
            uncertainty_img_msg.header.frame_id = self.sensor_frame_name
            self.uncertainty_pub.publish(uncertainty_img_msg)


class FrameDataLoader:
    def __init__(
        self,
        scan_dir_path: Path,
        image_size: Tuple[int, int] = _INPUT_IMAGE_DIMS,
        use_uncertainty: bool = False,
    ):
        self.scan_dir_path = scan_dir_path
        self.image_size = image_size
        self.use_uncertainty = use_uncertainty

    @staticmethod
    def load_frame_ids_and_timestamps(
        timestamps_file_path: Path,
        play_rate: int,
        frame_skip: int = 0,
    ):
        ids = []
        times = []
        with timestamps_file_path.open("r") as f:
            csv_reader = csv.reader(f)
            for row in csv_reader:
                if row[0] == "FrameID":
                    continue
                ids.append(str(row[0]))
                times.append(float(row[1]) / 1e9)

        ids = [x for _, x in sorted(zip(times, ids))]
        times = sorted(times)

        # Drop frames
        if frame_skip > 0:
            ids = [x for idx, x in enumerate(ids) if idx % frame_skip == 0]
            times = [x for idx, x in enumerate(times) if idx % frame_skip == 0]

        times = [(x - times[0]) / play_rate for x in times]
        return ids, times

    def load(self, frame_id) -> FrameData:
        pose_file_path = (
            self.scan_dir_path / _POSES_DIR / "{:05d}.txt".format(int(frame_id))
        )
        pose = np.loadtxt(str(pose_file_path))
        if np.isinf(pose).any() or np.isnan(pose).any():
            rospy.logwarn(f"Pose at frame {frame_id} contains invalid entries.")
            return None

        color_image_file_path = (
            self.scan_dir_path / _COLOR_IMAGES_DIR / "{:05d}.jpg".format(int(frame_id))
        )
        color = cv2.resize(
            cv2.imread(str(color_image_file_path)),
            dsize=_INPUT_IMAGE_DIMS,
            interpolation=cv2.INTER_AREA,
        )

        depth_image_file_path = (
            self.scan_dir_path / _DEPTH_IMAGES_DIR / "{:05d}.png".format(int(frame_id))
        )
        raw_depth = cv2.resize(
            np.array(PilImage.open(str(depth_image_file_path))),
            dsize=_INPUT_IMAGE_DIMS,
            interpolation=cv2.INTER_NEAREST,
        )
        depth = raw_depth.astype(np.float32) / _DEPTH_SHIFT

        segmentation_file_path = (
            self.scan_dir_path / _PANOPTIC_PRED_DIR / "{:05d}.png".format(int(frame_id))
        )
        segmentation = cv2.resize(
            np.array(PilImage.open(segmentation_file_path)),
            dsize=_INPUT_IMAGE_DIMS,
            interpolation=cv2.INTER_NEAREST,
        )
        segmentation = segmentation.astype(np.int32)

        segments_info_file_path = (
            self.scan_dir_path
            / _PANOPTIC_PRED_DIR
            / "{:05d}_segments_info.json".format(int(frame_id))
        )
        with segments_info_file_path.open("r") as f:
            segments_info = json.load(f)

        frame_data = FrameData(
            color=color,
            depth=depth,
            pose=pose,
            segmentation=segmentation,
            segments_info=segments_info,
        )

        if self.use_uncertainty:
            uncertainty_image_file_path = (
                self.scan_dir_path
                / _PANOPTIC_PRED_DIR
                / "{:05d}_uncertainty.tiff".format(int(frame_id))
            )
            frame_data.uncertainty = np.array(
                PilImage.open(str(uncertainty_image_file_path))
            )

        return frame_data


class ScannetV2DataPlayer:
    def __init__(self):

        # node params
        self.scans_dir_path = Path(rospy.get_param("~scans_dir_path"))
        self.scan_id = rospy.get_param("~scan_id")
        self.scan_dir_path = self.scans_dir_path / self.scan_id
        if not self.scan_dir_path.is_dir():
            rospy.logfatal(f"{str(self.scan_dir_path)} is not a valid directory path!")

        self.global_frame_name = rospy.get_param("~global_frame_name", "world")
        self.sensor_frame_name = rospy.get_param("~sensor_frame_name", "depth_cam")
        self.play_rate = rospy.get_param("~play_rate", 1.0)
        self.wait = rospy.get_param("~wait", False)
        self.frame_skip = rospy.get_param("~frame_skip", 0)
        self.refresh_rate = 30  # Hz
        self.use_uncertainty = rospy.get_param("~use_uncertainty", False)

        # Configure frame data loader
        self.frame_data_loader = FrameDataLoader(
            self.scan_dir_path,
            _INPUT_IMAGE_DIMS,
            self.use_uncertainty,
        )

        # Configure frame data publisher
        self.frame_data_publisher = FrameDataPublisher(
            self.sensor_frame_name,
            self.global_frame_name,
            self.use_uncertainty,
        )

        self.current_index = 0
        self.start_time = None

        timestamps_file_path = self.scan_dir_path / "timestamps.csv"
        if not timestamps_file_path.is_file():
            rospy.logfatal("Timestamps file not found!")
        self.ids, self.times = FrameDataLoader.load_frame_ids_and_timestamps(
            timestamps_file_path,
            self.play_rate,
            self.frame_skip,
        )

        if self.wait:
            self.start_srv = rospy.Service("~start", Empty, self.start)
        else:
            self.start(None)

    def start(self, _):
        self.running = True
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.refresh_rate), self.callback)
        return EmptyResponse

    def callback(self, _):

        if not self.running:
            return

        now = rospy.Time.now()
        if self.start_time is None:
            self.start_time = now

        if self.times[self.current_index] > (now - self.start_time).to_sec():
            return

        frame_id = self.ids[self.current_index]

        # Load and publish next frame data
        frame_data = self.frame_data_loader.load(frame_id)
        if frame_data is not None:
            rospy.logdebug(f"Publishing data for frame {frame_id}")
            self.frame_data_publisher.publish(frame_data, now)
        else:
            rospy.logwarn(f"Data for frame {frame_id} could not be loaded. Skipped.")

        self.current_index += 1
        if self.current_index >= len(self.ids):
            rospy.signal_shutdown("Player reached end of sequence.")


if __name__ == "__main__":
    rospy.init_node("scannetv2_data_player")
    scannetv2_data_player = ScannetV2DataPlayer()
    rospy.spin()
