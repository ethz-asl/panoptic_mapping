#!/usr/bin/env python

import os
import rospy
from panoptic_mapping_msgs.srv import SaveLoadMap


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

    def evaluate(self):
        # Get target maps.
        files = [
            x for x in os.listdir(self.data_path) if x.endswith('.panmap')
        ]

        for i, f in enumerate(sorted(files)):
            print("Evaluating map %i/%i:" % (i, len(files) - 1))
            self.eval_srv(os.path.join(self.data_path, f))
        print("Evaluation finished successfully.")


if __name__ == '__main__':
    rospy.init_node('evaluation_manager')
    em = EvaluationManager()
    em.evaluate()
