#!/usr/bin/env python3

import os
import rospy
from panoptic_mapping_msgs.srv import SaveLoadMap

# Hash number for each scene, DATA_IDS[scene_id][scan_id] = hash.
DATA_IDS = [[
    '0cac7578-8d6f-2d13-8c2d-bfa7a04f8af3',
    '2451c041-fae8-24f6-9213-b8b6af8d86c1',
    'ddc73793-765b-241a-9ecd-b0cebb7cf916',
    'ddc73795-765b-241a-9c5d-b97744afe077'
],
            [
                '20c9939d-698f-29c5-85c6-3c618e00061f',
                'f62fd5f8-9a3f-2f44-8b1e-1289a3a61e26'
            ]]


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

        # RIO
        use_rio = rospy.get_param('~use_rio', False)
        if use_rio:
            scene_id = rospy.get_param('~scene_id', 0)
            scan_id = rospy.get_param('~scan_id', 0)
            if len(DATA_IDS) <= scene_id:
                rospy.logfatal("Scene ID %i is out of bounds (%i)." %
                               (scene_id, len(DATA_IDS) - 1))
            if len(DATA_IDS[scene_id]) <= scan_id:
                rospy.logfatal(
                    "Scan ID %i is out of bounds (%i) for scene %i." %
                    (scan_id, len(DATA_IDS[scene_id]) - 1, scene_id))
            # NOTE(Schmluk): This is currently just hardcoded.
            rospy.set_param(
                "/multi_map_evaluation/ground_truth_pointcloud_file",
                "/home/lukas/Documents/Datasets/3RScan/%s/gt_10000.ply" %
                DATA_IDS[scene_id][scan_id])

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
