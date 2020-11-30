#!/usr/bin/env python

import os
from struct import pack, unpack

import numpy as np
import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import tf
import cv2


class CameraIntrinsics(object):
    def __init__(self):
        self.width = 0
        self.height = 0
        self.center_x = 0
        self.center_y = 0
        self.fx = 0
        self.fy = 0


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
        self.rate = float(rospy.get_param('~play_rate', 5))  # Hz
        self.frame_name = rospy.get_param('~frame_name', 'rio')
        self.global_frame_name = rospy.get_param('~global_frame_name', 'world')
        self.use_rendered_data = rospy.get_param('~use_rendered_data', True)
        self.adjust_image_size = rospy.get_param('~adjust_image_size', False)

        # ROS
        self.color_pub = rospy.Publisher("~color_image", Image, queue_size=10)
        self.depth_pub = rospy.Publisher("~depth_image", Image, queue_size=10)
        self.seg_pub = rospy.Publisher("~segmentation_image",
                                       Image,
                                       queue_size=10)
        self.pose_pub = rospy.Publisher("~pose", Pose, queue_size=10)
        self.pcl_pub = rospy.Publisher("~pointcloud",
                                       PointCloud2,
                                       queue_size=10)

        # Read intrinsics
        info_file = os.path.join(self.data_path, self.data_ids[self.scan_id],
                                 "sequence", "_info.txt")
        if not os.path.isfile(info_file):
            rospy.logerr("Info file '%s' does not exist.")
        else:
            lines = open(info_file, 'r').readlines()
            self.color_cam = CameraIntrinsics()
            self.depth_cam = CameraIntrinsics()
            # Currently hard coded since data format hopefully doesnt change.
            self.color_cam.width = int(lines[2][15:])
            self.color_cam.height = int(lines[3][16:])
            self.color_cam.center_x = float(lines[7][30:].split()[2])
            self.color_cam.center_y = float(lines[7][30:].split()[6])
            self.color_cam.fx = float(lines[7][30:].split()[0])
            self.color_cam.fy = float(lines[7][30:].split()[5])
            self.depth_cam.width = int(lines[4][15:])
            self.depth_cam.height = int(lines[5][16:])
            self.depth_cam.center_x = float(lines[9][30:].split()[2])
            self.depth_cam.center_y = float(lines[9][30:].split()[6])
            self.depth_cam.fx = float(lines[9][30:].split()[0])
            self.depth_cam.fy = float(lines[9][30:].split()[5])

        # setup
        self.cv_bridge = CvBridge()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.frame_no = 0
        self.timer = rospy.Timer(rospy.Duration(1 / self.rate), self.timer_cb)

    def timer_cb(self, _):
        frame_name = "frame-%06d" % self.frame_no
        time_stamp = rospy.Time.now()

        # use color to check for existence.
        color_file = os.path.join(self.data_path, self.data_ids[self.scan_id],
                                  "sequence", frame_name + ".color.jpg")
        if not os.path.isfile(color_file):
            rospy.logwarn("No more frames found (published %i)." %
                          self.frame_no)
            rospy.signal_shutdown("No more frames found.")
            return

        # transformation
        pose_file = os.path.join(self.data_path, self.data_ids[self.scan_id],
                                 "sequence", frame_name + ".pose.txt")
        pose_data = [float(x) for x in open(pose_file, 'r').read().split()]

        transform = np.eye(4)
        for row in range(4):
            for col in range(4):
                transform[row, col] = pose_data[row * 4 + col]
        rotation = tf.transformations.quaternion_from_matrix(transform)
        self.tf_broadcaster.sendTransform(
            (transform[0, 3], transform[1, 3], transform[2, 3]), rotation,
            time_stamp, self.frame_name, self.global_frame_name)
        pose_msg = Pose()
        pose_msg.position.x = pose_data[3]
        pose_msg.position.y = pose_data[7]
        pose_msg.position.z = pose_data[11]
        pose_msg.orientation.x = rotation[0]
        pose_msg.orientation.y = rotation[1]
        pose_msg.orientation.z = rotation[2]
        pose_msg.orientation.w = rotation[3]
        self.pose_pub.publish(pose_msg)

        # depth image
        if self.use_rendered_data:
            depth_file = os.path.join(self.data_path,
                                      self.data_ids[self.scan_id], "rendered",
                                      frame_name + ".rendered.depth.png")
            cv_depth = cv2.imread(depth_file, -1)
            cv_depth = cv2.rotate(cv_depth, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv_depth = np.array(cv_depth, dtype=np.float32) / 1000
            # Compute 3d cloud because it's needed anyway later
            cols, rows = np.meshgrid(
                np.linspace(0,
                            self.color_cam.width - 1,
                            num=self.color_cam.width),
                np.linspace(0,
                            self.color_cam.height - 1,
                            num=self.color_cam.height))
            im_x = (cols - self.color_cam.center_x) / self.color_cam.fx
            im_y = (rows - self.color_cam.center_y) / self.color_cam.fy
        else:
            depth_file = os.path.join(self.data_path,
                                      self.data_ids[self.scan_id], "sequence",
                                      frame_name + ".depth.pgm")
            cv_depth = cv2.imread(depth_file, -1)
            cv_depth = np.array(cv_depth, dtype=np.float32) / 1000
            # Compute 3d cloud because it's needed anyway later
            cols, rows = np.meshgrid(
                np.linspace(0,
                            self.depth_cam.width - 1,
                            num=self.depth_cam.width),
                np.linspace(0,
                            self.depth_cam.height - 1,
                            num=self.depth_cam.height))
            im_x = (cols - self.depth_cam.center_x) / self.depth_cam.fx
            im_y = (rows - self.depth_cam.center_y) / self.depth_cam.fy

        depth_msg = self.cv_bridge.cv2_to_imgmsg(cv_depth, "32FC1")
        depth_msg.header.stamp = time_stamp
        depth_msg.header.frame_id = self.frame_name
        self.depth_pub.publish(depth_msg)

        # color image
        if self.use_rendered_data:
            color_file = os.path.join(self.data_path,
                                      self.data_ids[self.scan_id], "rendered",
                                      frame_name + ".rendered.color.jpg")
            cv_img = cv2.imread(color_file)
            cv_img = cv2.rotate(cv_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        else:
            cv_img = cv2.imread(color_file)
            if self.adjust_image_size:
                color_img = np.zeros(
                    (self.depth_cam.height, self.depth_cam.width, 3),
                    dtype=np.uint8)
                im_u = im_x * self.color_cam.fx + self.color_cam.center_x
                im_v = im_y * self.color_cam.fy + self.color_cam.center_y
                im_u = np.array(np.clip(np.round(im_u), 0,
                                        self.color_cam.width - 1),
                                dtype=int)
                im_v = np.array(np.clip(np.round(im_v), 0,
                                        self.color_cam.height - 1),
                                dtype=int)
                for u in range(self.depth_cam.width):
                    for v in range(self.depth_cam.height):
                        color_img[v, u, :] = cv_img[im_v[v, u], im_u[v, u], :]
                cv_img = color_img

        color_msg = self.cv_bridge.cv2_to_imgmsg(cv_img, "bgr8")
        color_msg.header.stamp = time_stamp
        color_msg.header.frame_id = self.frame_name
        self.color_pub.publish(color_msg)

        # segmentation image
        seg_file = os.path.join(self.data_path, self.data_ids[self.scan_id],
                                "rendered",
                                frame_name + ".rendered.panlabels.png")
        cv_seg = cv2.imread(seg_file)
        cv_seg = cv_seg[:, :, 0]
        if self.use_rendered_data:
            cv_seg = cv2.rotate(cv_seg, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif self.adjust_image_size:
            seg_img = np.ones((self.depth_cam.height, self.depth_cam.width),
                              dtype=np.uint8) * 255
            for u in range(self.depth_cam.width):
                for v in range(self.depth_cam.height):
                    seg_img[v, u] = cv_seg[im_u[v, u], im_v[v, u]]
            cv_seg = seg_img

        # TEST
        cv_seg = np.ones(np.shape(cv_seg), dtype=np.uint8) * 15

        seg_msg = self.cv_bridge.cv2_to_imgmsg(cv_seg, "mono8")
        seg_msg.header.stamp = time_stamp
        seg_msg.header.frame_id = self.frame_name
        self.seg_pub.publish(seg_msg)

        # TEST create pointcloud
        # get 3d cloud
        points_x = im_x.reshape(-1)
        points_y = im_y.reshape(-1)
        points_z = cv_depth.reshape(-1)

        # pack color image
        r = np.ravel(cv_img[:, :, 2]).astype(int)
        g = np.ravel(cv_img[:, :, 1]).astype(int)
        b = np.ravel(cv_img[:, :, 0]).astype(int)
        color = np.left_shift(r, 16) + np.left_shift(g, 8) + b
        packed = pack('%di' % len(color), *color)
        unpacked = unpack('%df' % len(color), packed)
        rgb = np.array(unpacked)

        # filter invalid points

        # publish result
        data = np.transpose(np.vstack((points_x, points_y, points_z, rgb)))
        msg = PointCloud2()
        msg.header.stamp = time_stamp
        msg.header.frame_id = self.frame_name
        msg.width = data.shape[0]
        msg.height = 1
        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1)
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = np.float32(data).tostring()
        self.pcl_pub.publish(msg)

        # finish
        self.frame_no = self.frame_no + 1


if __name__ == '__main__':
    rospy.init_node('rio_player', anonymous=True)
    rio_player = RioPlayer()
    rospy.spin()
