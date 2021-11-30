#!/usr/bin/env python3

import os
import math
import yaml
import numpy as np

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class WayPointPublisher(object):
    def __init__(self):
        """  Initialize ros node and read params """
        # Params
        self.auto_start_time = rospy.get_param('~auto_start_time', 5)  # s
        waypoint_path = rospy.get_param('~waypoint_path',
                                        "")  # Where to read the points
        if not os.path.isfile(waypoint_path):
            rospy.logerr("'%s' is not a file!")
        try:
            with open(waypoint_path) as yaml_file:
                self.waypoints = yaml.load(yaml_file, Loader=yaml.Loader)
            rospy.loginfo("Read %i waypoints from '%s'." %
                          (len(self.waypoints), waypoint_path))
        except OSError:
            rospy.logerr("Could not read '%s'!" % waypoint_path)
        self.position_threshold = rospy.get_param('~replan_pos_threshold',
                                                  0.2)  # m
        self.yaw_threshold = rospy.get_param('~replan_yaw_threshold',
                                             15)  # deg

        # ROS
        self.pose_sub = rospy.Subscriber("~pose_in", PoseStamped, self.pose_cb)
        self.start_sub = rospy.Subscriber("~simulation_ready", Bool,
                                          self.start_cb)
        self.pose_pub = rospy.Publisher("~waypoints_out", Pose, queue_size=10)

        # variables
        self.current_index = 0
        self.goal_pose = None
        self.running = False
        self.init_time = rospy.Time.now()

    def pose_cb(self, pose_stamped):
        if not self.running:
            if self.auto_start_time > 0:
                if (rospy.Time.now() -
                        self.init_time).to_sec() > self.auto_start_time:
                    self.start_cb(None)
            return
        pose = pose_stamped.pose
        dist = np.linalg.norm(
            np.array(self.goal_pose[:3]) -
            np.array([pose.position.x, pose.position.y, pose.position.z]))
        if dist > self.position_threshold:
            return
        yaw = euler_from_quaternion([
            pose.orientation.x, pose.orientation.y, pose.orientation.z,
            pose.orientation.w
        ])[2]
        yaw_diff = math.fmod(yaw * 180 / math.pi - self.goal_pose[3], 360)
        if yaw_diff < 0:
            yaw_diff = yaw_diff + 360
        if yaw_diff > 180:
            yaw_diff = -yaw_diff + 180
        if yaw_diff > self.yaw_threshold:
            return
        self.current_index = self.current_index + 1
        if self.current_index < len(self.waypoints):
            self.goal_pose = self.waypoints[self.current_index]
            self.publish_goal()
        else:
            text = "* Finished Executing all waypoints. *"
            rospy.loginfo("\n" + "*" * len(text) + "\n" + text + "\n" +
                          "*" * len(text))
            self.running = False
            self.auto_start_time = -1

    def start_cb(self, _):
        rospy.sleep(1.0)
        self.goal_pose = self.waypoints[self.current_index]
        self.running = True
        self.publish_goal()

    def publish_goal(self):
        rospy.loginfo("Published a waypoint.")
        pose = Pose()
        pose.position.x = self.goal_pose[0]
        pose.position.y = self.goal_pose[1]
        pose.position.z = self.goal_pose[2]
        q = quaternion_from_euler(0, 0, self.goal_pose[3] * math.pi / 180)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        self.pose_pub.publish(pose)


if __name__ == '__main__':
    rospy.init_node('waypoint_dataset_generator', anonymous=True)
    wp = WayPointPublisher()
    rospy.spin()
