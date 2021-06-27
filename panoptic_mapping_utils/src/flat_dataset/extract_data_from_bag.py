#!/usr/bin/env python

import os
import csv
from warnings import warn
import rosbag
from cv_bridge import CvBridge
import cv2
from PIL import Image

COLOR_TOPIC = '/airsim_drone/Scene_cam'
DEPTH_TOPIC = '/airsim_drone/Depth_cam'
ID_TOPIC = '/airsim_drone/Id_corrected'  # Currently unused


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
    img_no_and_ts = {}
    img_no = 0
    cv_bridge = CvBridge()
    bag = rosbag.Bag(source_bag)

    # Color data.
    for _, msg, _ in bag.read_messages(topics=[COLOR_TOPIC]):
        image_id = "%06d" % img_no
        img_no = img_no + 1
        img_no_and_ts[image_id] = msg.header.stamp
        image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv2.imwrite(os.path.join(target_dest, image_id + "_color.png"), image)

    # Depth data.
    img_no = 0
    for _, msg, _ in bag.read_messages(topics=[DEPTH_TOPIC]):
        image_id = "%06d" % img_no
        img_no = img_no + 1
        if img_no_and_ts[image_id] != msg.header.stamp:
            warn(
                "Image timestamps don't match for %s: color: %s vs depth: %s" %
                (image_id, img_no_and_ts[image_id], msg.header.stamp))
        img2 = Image.fromarray(
            cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'))
        img2.save(os.path.join(target_dest, image_id + "_depth.tiff"))
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
    print("Finished parsing %i images of '%s' to target '%s'." %
          (img_no, source_bag, target_dest))


if __name__ == '__main__':
    source_bag = '/home/lukas/Documents/Datasets/large_flat_dataset/' \
                 'run1.bag'
    target_dest = '/home/lukas/Documents/Datasets/large_flat_dataset/run1'
    extract_bag(source_bag, target_dest)
