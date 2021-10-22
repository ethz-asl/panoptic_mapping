#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from panoptic_mapping_msgs.srv import RenderCameraImage
from cv_bridge import CvBridge
import numpy as np
from embodied_active_learning.airsim_utils import semantics

class Requester():
    def __init__(self):
        """  Initialize ros node and read params """

        self.service_proxy = rospy.ServiceProxy('/render_camera_view', RenderCameraImage)
        self.timer = rospy.Timer(rospy.Duration(3), self.callback)

        self.converter = semantics.AirSimSemanticsConverter(
        "/home/rene/catkin_ws/src/active_learning_for_segmentation/embodied_active_learning/cfg/airsim/semanticClassesCustomFlat.yaml")

        self.c = 0
    def callback(self, timer):
        print("timer got called")
        # req = RenderCameraImage(False, False, None, None, "front_optical", rospy.Time.now())
        # req.use_depth = False
        # req.use_provided_tf = False
        # req.sensor_frame = "front_optic"
        # req.lookup_ts = rospy.Time.now()
        # print("REquest", req)
        print("requesting")
        resp = self.service_proxy(False, False, None, None, "front_optical", rospy.Time.now())
        print(dir(resp))
        import matplotlib.pyplot as plt
        img_msg  = resp.class_image
        img = np.frombuffer(img_msg.data, dtype=np.uint8)
        img = img.reshape(img_msg.height, img_msg.width, 1)
        print(np.unique(img))
        img_col = self.converter.semantic_prediction_to_nyu_color(img.copy()).astype(np.uint8)
        print(img_col.shape)

        from PIL import Image
        im = Image.fromarray(img_col.squeeze())
        im.save("/home/rene/thesis/test_seg_{0:05d}.jpeg".format(self.c))
        self.c += 1

if __name__ == '__main__':
    rospy.init_node('airsim_semantics_converter')
    converter = Requester()
    rospy.spin()
