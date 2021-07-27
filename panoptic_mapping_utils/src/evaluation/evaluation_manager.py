#!/usr/bin/env python

import os
import json
import csv

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
from PIL import Image as PilImage
import numpy as np
import tf

from panoptic_mapping_msgs.msg import SaveLoadMap


class EvaluationManager(object):
    def __init__(self):
        """  Initialize ros node and read params """
        # params
        self.data_path = rospy.get_param('~map_file', '')
        self.eval_srv_name = rospy.get_param(
            '~eval_srv_name', '/multi_map_evaluation/process_map')

        # Check params.
        if not self.data_path:
            rospy.logfatal("The 'map_file' target directory needs to be set.")
        if not os.path.isdir(self.data_path):
            rospy.logfatal("The 'map_file' must be the target directory.")

        # Setup rosservice
        rospy.wait_for_service(self.eval_srv_name)
        self.eval_srv = rospy.ServiceProxy(self.eval_srv_name, SaveLoadMap)
        print("Successfully setup the evaluation manager.")

    def evaluate():
        # Get target maps.
        files = [
            x for x in os.listdir(self.data_path)
            if os.path.isfile(x) and x.endswith('.panmap')
        ]
        print(files)
        return 1


if __name__ == '__main__':
    rospy.init_node('evaluation_manager')
    em = EvaluationManager()
    em.evaluate()
