#!/usr/bin/env python3

import os
import json
import csv

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PIL import Image as PilImage
import numpy as np
import tf

from panoptic_mapping_msgs.msg import DetectronLabel, DetectronLabels


class DetectronPlayer(object):
    def __init__(self):
        """  Initialize ros node and read params """
        # params
        self.data_path = rospy.get_param(
            '~data_path', '/home/lukas/Documents/Datasets/flat_dataset/run1')
        self.global_frame_name = rospy.get_param('~global_frame_name', 'world')

        # ROS
        self.img_pub = rospy.Publisher("~predicted_image",
                                       Image,
                                       queue_size=10)
        self.depth_pub = rospy.Publisher("~depth_image", Image, queue_size=10)
        self.label_pub = rospy.Publisher("~labels",
                                         DetectronLabels,
                                         queue_size=10)
        self.img_sub = rospy.Subscriber("~id_image_in",
                                        Image,
                                        self.img_callback,
                                        queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # setup
        self.cv_bridge = CvBridge()
        stamps_file = os.path.join(self.data_path, 'timestamps.csv')
        self.stamp_to_id = {}
        if not os.path.isfile(stamps_file):
            rospy.logfatal("No timestamp file '%s' found." % stamps_file)
        with open(stamps_file, 'r') as read_obj:
            csv_reader = csv.reader(read_obj)
            for row in csv_reader:
                if row[0] == "ImageID":
                    continue
                self.stamp_to_id[str(row[1])] = str(row[0])

    def img_callback(self, id_img):
        # Verify lookups for requiered datasets.
        timestamp = str(
            id_img.header.stamp.secs) + "%09d" % id_img.header.stamp.nsecs
        if timestamp not in self.stamp_to_id:
            rospy.logwarn(
                "No prediction for message with timestamp '%s' found,"
                " skipping image." % timestamp)
            return
        prediction_file = os.path.join(
            self.data_path, self.stamp_to_id[timestamp] + "_predicted.png")
        if not os.path.isfile(prediction_file):
            rospy.logwarn("Could not find file '%s', skipping image." %
                          prediction_file)
            return
        labels_file = os.path.join(
            self.data_path, self.stamp_to_id[timestamp] + "_labels.json")
        if not os.path.isfile(labels_file):
            rospy.logwarn("Could not find file '%s', skipping image." %
                          labels_file)
            return

        # Load and publish image.
        cv_img = cv2.imread(prediction_file)
        img_msg = self.cv_bridge.cv2_to_imgmsg(cv_img[:, :, 0], "8UC1")
        img_msg.header.stamp = id_img.header.stamp
        img_msg.header.frame_id = id_img.header.frame_id
        self.img_pub.publish(img_msg)

        # Load and publish labels.
        label_msg = DetectronLabels()
        label_msg.header.stamp = img_msg.header.stamp
        with open(labels_file) as json_file:
            data = json.load(json_file)
            for d in data:
                if 'instance_id' not in d:
                    d['instance_id'] = 0
                if 'score' not in d:
                    d['score'] = 0
                label = DetectronLabel()
                label.id = d['id']
                label.instance_id = d['instance_id']
                label.is_thing = d['isthing']
                label.category_id = d['category_id']
                label.score = d['score']
                label_msg.labels.append(label)
        self.label_pub.publish(label_msg)

        # Load and publish depth image. These are optional.
        depth_file = os.path.join(self.data_path,
                                  self.stamp_to_id[timestamp] + "_depth.tiff")
        if os.path.isfile(depth_file):
            cv_img = PilImage.open(depth_file)
            img_msg = self.cv_bridge.cv2_to_imgmsg(np.array(cv_img), "32FC1")
            img_msg.header.stamp = id_img.header.stamp
            # img_msg.header.frame_id = id_img.header.frame_id
            img_msg.header.frame_id = "test_frame"
            self.depth_pub.publish(img_msg)

        # Load and publish transform.
        pose_file = os.path.join(self.data_path,
                                 self.stamp_to_id[timestamp] + "_pose.txt")
        if os.path.isfile(pose_file):
            pose_data = [float(x) for x in open(pose_file, 'r').read().split()]
            transform = np.eye(4)
            for row in range(4):
                for col in range(4):
                    transform[row, col] = pose_data[row * 4 + col]
            rotation = tf.transformations.quaternion_from_matrix(transform)
            self.tf_broadcaster.sendTransform(
                (transform[0, 3], transform[1, 3], transform[2, 3]), rotation,
                id_img.header.stamp, "test_frame", self.global_frame_name)


if __name__ == '__main__':
    rospy.init_node('detectron_player', anonymous=True)
    detectron_player = DetectronPlayer()
    rospy.spin()
