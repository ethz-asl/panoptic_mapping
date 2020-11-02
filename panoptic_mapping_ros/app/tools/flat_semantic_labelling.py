#!/usr/bin/env python

import csv
import operator
import collections

import airsim
import numpy as np
from scipy import misc


def create_label_ids(ir_correction_file):
    """
    Create the class and instance labels for the objects in the flat dataset.
    """
    # Read infrared label mapping (because airsims infrared values are screwed)
    instance_to_mesh_id = []  # inst_to_mesh_id[inst_id] = [mesh_id, ir_color]
    with open(ir_correction_file) as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='"')
        last_id = -1
        for row in reader:
            if row[0] != 'MeshID':
                if int(row[1]) > last_id:
                    last_id = int(row[1])
                    instance_to_mesh_id.append([int(row[0]), last_id])

    # labels for the flat test dataset
    label_ids = {}
    mesh_ids = {}
    class_ids = {}
    counter = [0]
    class_counter = [0]
    letters = [
        "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n",
        "o", "p", "q", "r", "s", "t", "u"
    ]

    def set_label(name, label_id=None, increment_class=True, count=1):
        for i in range(count):
            full_name = name + "_" + letters[i]
            if label_id is None:
                label_id = counter[0]
                counter[0] = counter[0] + 1
            label_ids[full_name] = label_id
            if label_id < len(instance_to_mesh_id):
                mesh_ids[full_name] = instance_to_mesh_id[label_id]
            else:
                print "Warning: id '%i' is larger than the maximum supported " \
                      "id count" % label_id
            if increment_class:
                class_counter[0] = class_counter[0] + 1
            class_ids[full_name] = class_counter[0]
            increment_class = False  # for multiple count

    # Background classes
    set_label("SM_Ceiling")
    set_label("SM_Curtains")
    set_label("SM_Floor")
    set_label("SM_Walls")
    set_label("SM_Windows_glass")
    set_label("SM_Windows_Windows")
    set_label("SM_TV_Wall")

    panoptic_count = counter[0] - 1
    # Instances
    set_label("SM_Bed")
    set_label("SM_Bed_lamp", count=2)
    set_label("SM_Bed_table", count=2)
    set_label("SM_Bottle")
    set_label("SM_Ceiling_lamp", count=12)
    set_label("SM_Chair", count=2)
    set_label("SM_Kitchen_Chair_a", increment_class=False)
    set_label("SM_Coffee_table")
    set_label("SM_Cup_", count=3)
    set_label("SM_Decor", count=2)
    set_label("SM_Digital_Clock")
    set_label("SM_Dimmer", count=2)
    set_label("SM_Door", count=2)
    set_label("SM_Floor_Lamp")
    set_label("SM_Journal", count=3)
    set_label("SM_Kitchen")
    set_label("SM_Picture", count=7)
    set_label("SM_Plant")
    set_label("SM_Plate", count=3)
    set_label("SM_Remote")
    set_label("SM_Sofa")
    set_label("SM_Stack_of_Books", count=6)
    set_label("SM_Table")
    set_label("SM_Table_Decor")
    set_label("SM_Tumblr", count=2)
    set_label("SM_TV")
    set_label("SM_Wall_Clock")
    set_label("SM_Coffee_Machine")
    set_label("SM_Nightstand")

    print("Created a labelling with %i instances and %i classes." %
          (counter[0] - 1, class_counter[0] - 1))
    return label_ids, class_ids, mesh_ids, panoptic_count


def apply_labels(mesh_labels):
    """
    Set the segmentation id for all object in unreal based on the info in
    'mesh_labels'
    """
    client = airsim.MultirotorClient()
    success = 0
    client.confirmConnection()
    # reset the labels of everything to invisible
    client.simSetSegmentationObjectID(r"[\w]*", -1, True)
    for key in mesh_labels:
        if client.simSetSegmentationObjectID(key, mesh_labels[key][0], False):
            success = success + 1
            print "Successfully set label for '%s'." % key
        else:
            print "Failed to set label for '%s'." % key
    print "Applied %i labels." % success


def get_available_meshes(comparison_labels=None):
    """
    Print the names of all meshes in the unreal world. If comparison_labels is
    set print how many of these are uniquely
    matched with the existing names.
    """
    client = airsim.MultirotorClient()
    client.confirmConnection()
    names = client.simListSceneObjects(r"[\w]*")
    names = [str(name) for name in names]
    counts = []
    print "Available mesh names: ", sorted(names)
    for name in sorted(names):
        print "Found Mesh '%s'" % name
    if comparison_labels is not None:
        for key in comparison_labels:
            matches = [name == key for name in names]
            counts.append(np.sum(np.array(matches)))
        print "Label names found: ", counts
        print "Unique labels matched: %.1f percent" \
              % (np.mean(np.array(counts) == 1) * 100)


def get_infrared_correction(target_file):
    """
    Compute the transfer dynamics from segmentation id in unreal and the value
    in the infrared image and save it to file.
    """
    client = airsim.MultirotorClient()
    client.confirmConnection()
    counter = 0
    print "Computing infrared corrections ..."
    with open(target_file, 'w') as csvfile:
        writer = csv.writer(csvfile,
                            delimiter=',',
                            quotechar='|',
                            quoting=csv.QUOTE_MINIMAL)
        writer.writerow(["MeshID", "InfraRedID"])
        for i in range(256):
            client.simSetSegmentationObjectID(r"[\w]*", i, True)
            responses = client.simGetImages([
                airsim.ImageRequest("Id_cam", airsim.ImageType.Infrared, False,
                                    False)
            ])
            response = responses[0]
            img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
            writer.writerow([i, img1d[0]])
            if i * 100 / 256 > counter:
                counter = counter + 5
                print "%i percent done ..." % counter
    print "Saved infrared corrections in '%s'." % target_file


def export_labels(labels, classes, mesh_ids, panoptic_count, out_file_name):
    """
    Save label data to file.
    """
    color_palette = misc.imread(
        "/home/lukas/programs/AirSim/Unreal/Plugins/AirSim/Content/HUDAssets/"
        "seg_color_pallet.png")
    with open(out_file_name, 'w') as csvfile:
        writer = csv.writer(csvfile,
                            delimiter=',',
                            quotechar='|',
                            quoting=csv.QUOTE_MINIMAL)
        writer.writerow([
            "InstanceID",
            "ClassID",
            "PanopticID",
            "MeshID",
            "InfraredID",
            "R",
            "G",
            "B",
            "Name",
        ])
        sorted_labels = sorted(labels.items(), key=operator.itemgetter(1))
        sorted_labels = collections.OrderedDict(sorted_labels)
        for key in sorted_labels:
            instance_id = labels[key]
            mesh_id = mesh_ids[key]
            writer.writerow([
                instance_id, classes[key],
                int(instance_id <= panoptic_count), mesh_id[0], mesh_id[1],
                color_palette[0, mesh_id[0] * 4, 0],
                color_palette[0, mesh_id[0], 1], color_palette[0, mesh_id[0],
                                                               2], key
            ])
        writer.writerow([255, 255, 1, 255, 255, 80, 80, 80, "Unknown"])
    print "Saved labels in '%s'." % out_file_name


if __name__ == "__main__":
    get_ir_corrections = False
    apply_mesh_labels = True
    export_mesh_labels = True

    ir_file = "/home/lukas/Documents/PanopticMapping/Data/" \
              "infrared_corrections.csv"
    label_file = "/home/lukas/Documents/PanopticMapping/Data/labels.csv"

    if get_ir_corrections:
        get_infrared_correction(ir_file)
    if apply_mesh_labels or export_mesh_labels:
        f_labels, f_classes, f_mesh_ids, f_panoptic_count = create_label_ids(
            ir_file)
    if apply_mesh_labels:
        apply_labels(f_mesh_ids)
    if export_mesh_labels:
        export_labels(f_labels, f_classes, f_mesh_ids, f_panoptic_count,
                      label_file)

    # get_available_meshes(labels)
