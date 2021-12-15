#!/usr/bin/env python3

import json
import os
import csv

import numpy as np
import cv2

SOURCE_DIR = '/home/lukas/Documents/Datasets/3RScan'
INDEX_FILE = '3RScan.json'


def scan_exists(scan_id):
    return os.path.isdir(os.path.join(SOURCE_DIR, scan_id))


def list_matching_scans():
    with open(os.path.join(SOURCE_DIR, INDEX_FILE)) as json_file:
        index = json.load(json_file)
        for i in range(478):
            info = index[i]
            if not scan_exists(info['reference']):
                continue

            print("Ref scan %i (%s) exists. Rescans:" % (i, info['reference']))
            num_rescans_max = len(info['scans'])
            num_rescans = 0
            for j in range(len(info['scans'])):
                scan = info['scans'][j]
                if scan_exists(scan['reference']):
                    num_rescans = num_rescans + 1
                    print("  %i (%s) exists." % (j, scan['reference']))
                else:
                    print("  -%i (%s) failed." % (j, scan['reference']))
            print("  Found %i/%i rescans." % (num_rescans, num_rescans_max))


def produce_instance_images(scan_names, create_images):
    # Setup
    if scan_names is str:
        scan_names = [scan_names]

    # Prepare label list
    header = [
        "InstanceID", "ClassID", "PanopticID", "MeshID", "InfraredID", "R",
        "G", "B", "Name", "RIOGlobalID"
    ]
    labels = {}  # instance id to label list above
    name_counter = {}
    current_instance_id = 0

    # Parse scans
    for scan in scan_names:
        print("Total labels: %i" % len(labels))
        scan_dir = os.path.join(SOURCE_DIR, scan)
        if not os.path.isdir(scan_dir):
            print("Warning: Cannot fid dir '%s'" % scan_dir)
            continue

        # Read objects
        object_index = None
        with open(os.path.join(SOURCE_DIR, 'objects.json')) as json_file:
            indices = json.load(json_file)
            for index in indices['scans']:
                if index['scan'] == scan:
                    object_index = index['objects']
                    break
        if object_index is None:
            print("Warning: Could not find scan '%s' in object list." % scan)
            continue

        # Write objects to label list
        color_to_id = {}
        print("Found %i objects in scan '%s'." % (len(object_index), scan))
        for obj in object_index:
            h = obj['ply_color'].lstrip('#')
            color = [int(h[i:i + 2], 16) for i in (0, 2, 4)]

            rio_global_id = int(obj['global_id'])
            current_instance_id = current_instance_id + 1
            instance_id = current_instance_id
            nyu_label = int(obj['nyu40'])
            is_instance = int(nyu_label not in [1, 2, 9, 22, 38])
            name = obj['label']
            if name not in name_counter:
                name_counter[name] = 0
            else:
                name_counter[name] = name_counter[name] + 1
            name = name + "_%i" % name_counter[name]
            labels[instance_id] = [
                instance_id, nyu_label, is_instance, 0, 0, color[0], color[1],
                color[2], name, rio_global_id
            ]
            color_code = color[0] * 256 * 256 + color[1] * 256 + color[2]
            if color_code in color_to_id:
                print("Warning: color '%s' of rio global label %i already "
                      "exists." % (h, rio_global_id))
            color_to_id[color_code] = instance_id

        # Parse images
        frame = 0
        while create_images:
            label_file = os.path.join(SOURCE_DIR, scan, "rendered",
                                      "frame-%06d.rendered.labels.png" % frame)
            if not os.path.isfile(label_file):
                print("Finished parsing scan '%s' after frame %i" %
                      (scan, frame))
                break

            img_in = cv2.imread(label_file)
            w = np.shape(img_in)[0]
            h = np.shape(img_in)[1]
            img_out = np.zeros((w, h))
            found_codes = 0
            for x in range(w):
                for y in range(h):
                    color_code = img_in[x, y, 2] * 256 * 256 + img_in[x, y, 1]\
                        * 256 + img_in[x, y, 0]  # bgr
                    if color_code in color_to_id:
                        found_codes = found_codes + 1
                        img_out[x, y] = color_to_id[color_code]
            out_file = os.path.join(
                SOURCE_DIR, scan, "rendered",
                "frame-%06d.rendered.panlabels.png" % frame)
            cv2.imwrite(out_file, img_out)
            frame = frame + 1

    # Write label list
    label_out_file = os.path.join(SOURCE_DIR, "labels.csv")
    with open(label_out_file, 'w') as csvfile:
        writer = csv.writer(csvfile,
                            delimiter=',',
                            quotechar='|',
                            quoting=csv.QUOTE_MINIMAL)
        writer.writerow(header)
        for key in labels:
            writer.writerow(labels[key])


if __name__ == "__main__":
    list_matching_scans()

    # create_images = True
    # produce_instance_images([
    #     '0cac7578-8d6f-2d13-8c2d-bfa7a04f8af3',
    #     '2451c041-fae8-24f6-9213-b8b6af8d86c1',
    #     'ddc73793-765b-241a-9ecd-b0cebb7cf916',
    #     'ddc73795-765b-241a-9c5d-b97744afe077',
    #     '1776ad7e-4db7-2333-89e1-66854e82170c',
    #     '1776ad82-4db7-2333-89e4-d73159ac81d0',
    #     '1776ad86-4db7-2333-8935-240e44ccb16d',
    #     '1776ad84-4db7-2333-8aa7-2cc9126d5f71',
    #     'f62fd5f8-9a3f-2f44-8b1e-1289a3a61e26',
    #     '20c9939d-698f-29c5-85c6-3c618e00061f'
    # ], create_images)
