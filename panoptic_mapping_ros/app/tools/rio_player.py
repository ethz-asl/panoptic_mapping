#!/usr/bin/env python

import os
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import tf
import cv2


class RioPlayer(object):
    def __init__(self):
        """  Initialize ros node and read params """
        # params
        self.data_path = rospy.get_param(
            '~data_path', '/home/lukas/Documents/PanopticMapping/Data/3RScan')
        self.data_ids = rospy.get_param('~data_ids', [
            '0cac7578-8d6f-2d13-8c2d-bfa7a04f8af3',
            '2451c041-fae8-24f6-9213-b8b6af8d86c1',
            'ddc73793-765b-241a-9ecd-b0cebb7cf916',
            'ddc73795-765b-241a-9c5d-b97744afe077'
        ])
        self.scan_id = rospy.get_param('~scan_id', 0)
        self.rate = float(rospy.get_param('~rate', 5))  # Hz
        self.frame_name = rospy.get_param('~frame_name', 'rio')
        self.global_frame_name = rospy.get_param('~global_frame_name', 'world')

        # ROS
        self.color_pub = rospy.Publisher("~color_image", Image, queue_size=10)
        self.depth_pub = rospy.Publisher("~depth_image", Image, queue_size=10)
        self.seg_pub = rospy.Publisher("~segmentation_image",
                                       Image,
                                       queue_size=10)
        self.pose_pub = rospy.Publisher("~pose", Pose, queue_size=10)

        # setup
        self.cv_bridge = CvBridge()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.frame_no = 0
        self.timer = rospy.Timer(rospy.Duration(1 / self.rate), self.timer_cb)

    def timer_cb(self, _):
        frame_name = "frame-%06d" % self.frame_no
        time_stamp = rospy.Time.now()

        color_file = os.path.join(self.data_path, self.data_ids[self.scan_id],
                                  "sequence", frame_name + ".color.jpg")
        if not os.path.isfile(color_file):
            rospy.logwarn("No more frames found (published %i)." %
                          self.frame_no)
            return

        # color image
        cv_img = cv2.imread(color_file)
        color_msg = self.cv_bridge.cv2_to_imgmsg(cv_img, "bgr8")
        color_msg.header.stamp = time_stamp
        color_msg.header.frame_id = self.frame_name
        self.color_pub.publish(color_msg)

        # depth image
        depth_file = os.path.join(self.data_path, self.data_ids[self.scan_id],
                                  "sequence", frame_name + ".depth.pgm")
        cv_depth = np.array(cv2.imread(depth_file, -1),
                            dtype=np.float32) / 1000
        depth_msg = self.cv_bridge.cv2_to_imgmsg(cv_depth, "32FC1")  # 16UC1
        depth_msg.header.stamp = time_stamp
        depth_msg.header.frame_id = self.frame_name
        self.depth_pub.publish(depth_msg)

        # segmentation image
        seg_file = os.path.join(self.data_path, self.data_ids[self.scan_id],
                                "rendered",
                                frame_name + ".rendered.panlabels.png")
        cv_seg = cv2.imread(seg_file)
        cv_seg = cv_seg[:, :, 0]
        seg_msg = self.cv_bridge.cv2_to_imgmsg(cv_seg, "mono8")
        seg_msg.header.stamp = time_stamp
        seg_msg.header.frame_id = self.frame_name
        self.seg_pub.publish(seg_msg)

        # transformation
        pose_file = os.path.join(self.data_path, self.data_ids[self.scan_id],
                                 "sequence", frame_name + ".pose.txt")
        pose_data = [float(x) for x in open(pose_file, 'r').read().split()]
        rotation_matrix = np.eye(4)
        for i in range(3):
            for j in range(3):
                rotation_matrix[i, j] = pose_data[i + j * 4]
        rotation = tf.transformations.quaternion_from_matrix(rotation_matrix)
        self.tf_broadcaster.sendTransform(
            (pose_data[3], pose_data[7], pose_data[11]), rotation, time_stamp,
            self.frame_name, self.global_frame_name)
        pose_msg = Pose()
        pose_msg.position.x = pose_data[3]
        pose_msg.position.y = pose_data[7]
        pose_msg.position.z = pose_data[11]
        pose_msg.orientation.x = rotation[0]
        pose_msg.orientation.y = rotation[1]
        pose_msg.orientation.z = rotation[2]
        pose_msg.orientation.w = rotation[3]
        self.pose_pub.publish(pose_msg)

        # finish
        self.frame_no = self.frame_no + 1


if __name__ == '__main__':
    rospy.init_node('rio_player', anonymous=True)
    rio_player = RioPlayer()
    rospy.spin()
