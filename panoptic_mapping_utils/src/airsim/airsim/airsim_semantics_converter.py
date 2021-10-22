#!/usr/bin/env python
import rospy
from panoptic_mapping_msgs.srv import RenderCameraImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from embodied_active_learning.utils.airsim_semantics import AirSimSemanticsConverter

import cv2
from PIL import Image as PImage

class AirsimConverter(object):
    def __init__(self):
        """  Initialize ros node and read params """

        self.depth = None
        self.depth_2 = None
        self.cnt = 0
        self.c = 0
        self.cv_bridge = CvBridge()
        self.sub = rospy.Subscriber("input_image", Image, self.callback)
        # self.sub = rospy.Subscriber("depth_image", Image, self.callback_depth)
        self.semseg_pub = rospy.Publisher("output_image", Image, queue_size=100)
        self.uncertainty_pub = rospy.Publisher("uncertainy_image", Image, queue_size=100)
        self.converter = AirSimSemanticsConverter( "/home/rene/catkin_ws/src/active_learning_for_segmentation/embodied_active_learning/cfg/airsim/semanticClassesCustomFlat.yaml")

        self.converter.set_airsim_classes()
        self.service_proxy = rospy.ServiceProxy('/render_camera_view', RenderCameraImage)

    def callback(self, img_msg: Image):
        img = np.frombuffer(img_msg.data, dtype=np.uint8)
        img = img.reshape(img_msg.height, img_msg.width, 3)[:, :, 0] # only get one channel
        img = self.converter.map_infrared_to_nyu(img.copy())
        msg = self.cv_bridge.cv2_to_imgmsg(img, "8UC1")
        msg.header = img_msg.header
        self.semseg_pub.publish(msg)

        img = (img * 0 + 0.5).astype(np.float32)
        msg = self.cv_bridge.cv2_to_imgmsg(img, "32FC1")
        msg.header = img_msg.header
        self.uncertainty_pub.publish(msg)

        #
        # print("timer got called")
        # # req = RenderCameraImage(False, False, None, None, "front_optical", rospy.Time.now())
        # # req.use_depth = False
        # # req.use_provided_tf = False
        # # req.sensor_frame = "front_optic"
        # # req.lookup_ts = rospy.Time.now()
        # # print("REquest", req)
        # print("requesting")
        # rospy.sleep(0.1)
        # print(self.service_proxy(False, False, None, None, msg.header.frame_id, msg.header.stamp))
    def callback_depth(self, depth_img):
        self.cnt += 1
        if self.cnt % 5 != 4:
            return
        self.depth = self.depth_2
        self.depth_2 = depth_img
        self.timer = rospy.Timer(rospy.Duration(1), self.callback_time, oneshot = True)

    def callback_time(self, timer):
        print("timer got called")
        # req = RenderCameraImage(False, False, None, None, "front_optical", rospy.Time.now())
        # req.use_depth = False
        # req.use_provided_tf = False
        # req.sensor_frame = "front_optic"
        # req.lookup_ts = rospy.Time.now()
        # print("REquest", req)
        print("requesting")
        if(self.depth is None):
            return

        depth_message = self.depth
        resp = self.service_proxy(True, False, depth_message, None, depth_message.header.frame_id, depth_message.header.stamp)
        print(dir(resp))
        import matplotlib.pyplot as plt
        img_msg  = resp.class_image
        img = np.frombuffer(img_msg.data, dtype=np.uint8)
        img = img.reshape(img_msg.height, img_msg.width, 1)
        print(np.unique(img))
        img_col = self.converter.semantic_prediction_to_nyu_color(img.copy()).astype(np.uint8)
        print(img_col.shape)

        im = PImage.fromarray(img_col.squeeze())
        im.save("/home/rene/thesis/test_seg_{0:05d}.png".format(self.c))

        img_msg  = resp.uncertainty_image
        img = (np.frombuffer(img_msg.data, dtype=np.float32)*255).astype(np.uint8)
        img = img.reshape(img_msg.height, img_msg.width, 1)
        print(np.unique(img))
        img_col =  cv2.cvtColor(img.copy(),cv2.COLOR_GRAY2RGB).astype(np.uint8)
        print(img_col.shape)

        im = PImage.fromarray(img_col.squeeze())
        im.save("/home/rene/thesis/test_seg_uncertainy_{0:05d}.png".format(self.c))

        self.c += 1

if __name__ == '__main__':
    rospy.init_node('airsim_semantics_converter')
    converter = AirsimConverter()
    rospy.spin()
