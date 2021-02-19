#!/usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class PathVisualizer(object):
    def __init__(self):
        """  Initialize ros node and read params """
        # Params
        self.red = rospy.get_param('~r', 1)  # color 0-1
        self.green = rospy.get_param('~g', 0)
        self.blue = rospy.get_param('~b', 0)
        self.length = rospy.get_param('~length', 0.3)  # m
        self.width = rospy.get_param('~width', 0.03)  # m
        self.distance = rospy.get_param('~distance', 0.05)  # m
        self.use_arrow = rospy.get_param('~use_arrow', True)  # m

        # ROS
        self.sub = rospy.Subscriber("~pose_in", PoseStamped, self.pose_cb)
        self.pub = rospy.Publisher("~path", Marker, queue_size=10)

        # variables
        self.previous_pose = None
        self.counter = 0

    def pose_cb(self, pose_stamped):
        # Init
        if self.previous_pose is None:
            self.previous_pose = pose_stamped.pose
            return

        # Only plot every 'distance' meters
        pose = pose_stamped.pose
        dist = np.linalg.norm(
            np.array([
                self.previous_pose.position.x, self.previous_pose.position.y,
                self.previous_pose.position.z
            ]) - np.array([pose.position.x, pose.position.y, pose.position.z]))
        if dist < self.distance:
            return
        self.previous_pose = pose_stamped.pose

        # Plot
        msg = Marker()
        msg.header = pose_stamped.header
        msg.pose = pose
        msg.color.r = self.red
        msg.color.g = self.green
        msg.color.b = self.blue
        msg.color.a = 1
        msg.ns = "path"
        if self.use_arrow:
            msg.type = 0
            msg.scale.x = self.length
            msg.scale.y = self.width
            msg.scale.z = self.width * 2
        else:
            # Sphere
            msg.type = 2
            msg.scale.x = self.width
            msg.scale.y = self.width
            msg.scale.z = self.width
        msg.id = self.counter
        self.counter = self.counter + 1
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('path_visualizer', anonymous=True)
    pv = PathVisualizer()
    rospy.spin()
