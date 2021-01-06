#!/usr/bin/env python

import os
import csv

import rosbag
from cv_bridge import CvBridge
import cv2

COLOR_TOPIC = '/airsim_drone/Scene_cam'


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
    img_no_and_ts = []
    img_no = 0
    cv_bridge = CvBridge()
    bag = rosbag.Bag(source_bag)
    for _, msg, _ in bag.read_messages(topics=[COLOR_TOPIC]):
        image_id = "%06d" % img_no
        img_no_and_ts.append([image_id, msg.header.stamp])
        img_no = img_no + 1
        image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv2.imwrite(os.path.join(target_dest, image_id + "_color.png"), image)
    bag.close()

    # Write timestamp decoding.
    with open(os.path.join(target_dest, "timestamps.csv"), 'w') as csvfile:
        writer = csv.writer(csvfile,
                            delimiter=',',
                            quotechar='|',
                            quoting=csv.QUOTE_MINIMAL)
        writer.writerow(["ImageID", "TimeStamp"])
        for row in img_no_and_ts:
            writer.writerow(row)
    print("Finished parsing %i images of '%s' to target '%s'." %
          (img_no, source_bag, target_dest))


if __name__ == '__main__':
    source_bag = '/home/lukas/Documents/PanopticMapping/Data/flat_dataset/' \
                 'run1.bag'
    target_dest = '/home/lukas/Documents/PanopticMapping/Data/flat_dataset/run1'
    extract_bag(source_bag, target_dest)
