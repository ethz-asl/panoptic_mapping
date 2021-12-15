#!/usr/bin/env python3

import os
import csv
from warnings import warn
import rosbag
from cv_bridge import CvBridge
import cv2
from PIL import Image
from tf.transformations import quaternion_matrix

COLOR_TOPIC = '/airsim_drone/Scene_cam'
DEPTH_TOPIC = '/airsim_drone/Depth_cam'
CAMERA_FRAME_NAME = 'airsim_drone/Depth_cam'
ID_TOPIC = '/airsim_drone/Id_corrected'


def extract_bag(source_bag, target_dest):
    # Check valid args.
    if not os.path.isfile(source_bag):
        print("Error: source bag '%s' does not exist." % source_bag)
        return
    if not os.path.isdir(target_dest):
        os.makedirs(target_dest)
        print("Created new output directory at '%s'." % target_dest)

    # Parse the data.
    print("Parsing bag '%s' ..." % source_bag)
    cv_bridge = CvBridge()
    bag = rosbag.Bag(source_bag)
    img_no_and_ts = {}

    # Color data.
    img_no = 0
    for _, msg, _ in bag.read_messages(topics=[COLOR_TOPIC]):
        image_id = "%06d" % img_no
        img_no = img_no + 1
        img_no_and_ts[image_id] = msg.header.stamp
        image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv2.imwrite(os.path.join(target_dest, image_id + "_color.png"), image)
    print("Color: %i frames found." % img_no)

    # Depth data.
    img_no = 0
    for _, msg, _ in bag.read_messages(topics=[DEPTH_TOPIC]):
        image_id = "%06d" % img_no
        img_no = img_no + 1
        if img_no_and_ts[image_id] != msg.header.stamp:
            warn("Image timestamps don't match for frame %s: "
                 "color: %s vs depth: %s" %
                 (image_id, img_no_and_ts[image_id], msg.header.stamp))
        img2 = Image.fromarray(
            cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'))
        img2.save(os.path.join(target_dest, image_id + "_depth.tiff"))
    print("Depth: %i frames found." % img_no)

    # Segmentation data.
    img_no = 0
    for _, msg, _ in bag.read_messages(topics=[ID_TOPIC]):
        image_id = "%06d" % img_no
        img_no = img_no + 1
        if img_no_and_ts[image_id] != msg.header.stamp:
            warn("Image timestamps don't match for frame %s: "
                 "color: %s vs segmentation: %s" %
                 (image_id, img_no_and_ts[image_id], msg.header.stamp))
        image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv2.imwrite(os.path.join(target_dest, image_id + "_segmentation.png"),
                    image)
    print("GTSeg: %i frames found." % img_no)

    # Poses.
    # NOTE(schmluk): There should be a pose exactly stamped for each frame
    # from unreal_airsim.
    found_poses = 0
    for _, msg, _ in bag.read_messages(topics=["/tf"]):
        msg = msg.transforms[0]
        if msg.child_frame_id == CAMERA_FRAME_NAME:
            if msg.header.stamp in img_no_and_ts.values():
                found_poses += 1
                frame_id = img_no_and_ts.keys()[img_no_and_ts.values().index(
                    msg.header.stamp)]
                quat = msg.transform.rotation
                matrix = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
                with open(os.path.join(target_dest, frame_id + "_pose.txt"),
                          'w') as f:
                    f.write('%f %f %f %f\n' %
                            (matrix[0][0], matrix[0][1], matrix[0][2],
                             msg.transform.translation.x))
                    f.write('%f %f %f %f\n' %
                            (matrix[1][0], matrix[1][1], matrix[1][2],
                             msg.transform.translation.y))
                    f.write('%f %f %f %f\n' %
                            (matrix[2][0], matrix[2][1], matrix[2][2],
                             msg.transform.translation.z))
                    f.write('0.0 0.0 0.0 1.0')
    if found_poses < img_no:
        warn("Only found %i of %i poses!" % (found_poses, img_no))
    else:
        print("Poses: %i frames found." % found_poses)

    bag.close()

    # Write timestamp decoding.
    with open(os.path.join(target_dest, "timestamps.csv"), 'w') as csvfile:
        writer = csv.writer(csvfile,
                            delimiter=',',
                            quotechar='|',
                            quoting=csv.QUOTE_MINIMAL)
        writer.writerow(["ImageID", "TimeStamp"])
        for key, value in img_no_and_ts.items():
            writer.writerow([key, value])
    print("Finished parsing '%s' to target '%s'." % (source_bag, target_dest))


if __name__ == '__main__':
    source_bag = '/home/lukas/Documents/Datasets/flat_dataset/' \
                 'run2.bag'
    target_dest = '/home/lukas/Documents/Datasets/flat_dataset/run2'
    extract_bag(source_bag, target_dest)
