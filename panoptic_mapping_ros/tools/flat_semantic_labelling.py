#!/usr/bin/env python

import airsim
import numpy as np
import csv
from scipy import misc
import rospkg


def create_label_ids(ir_correction_file):
    """
    Create the class and instance labels for the objects in the flat dataset.
    """
    # Read infrared label mapping (because airsims infrared values are screwed up)
    instance_to_mesh_id = []  # instance_to_mesh_id[inst_id] = [mesh_id, ir_color]
    with open(ir_correction_file) as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='"')
        last_id = -1
        instance_id = 0
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

    def set_label(name, id=None, increment_class=True, class_counter=class_counter,
                  counter=counter, label_ids=label_ids, class_ids=class_ids):
        if id is None:
            id = counter[0]
            counter[0] = counter[0] + 1
        label_ids[name] = id
        if id < len(instance_to_mesh_id):
            mesh_ids[name] = instance_to_mesh_id[id]
        else:
            print("Warning: id '%i' is out larger than the maximum supported id count")
        if increment_class:
            class_counter[0] = class_counter[0] + 1
        class_ids[name] = class_counter[0]

    # Background classes
    letters = ["a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o", "p", "q", "r", "s", "t", "u"]
    set_label("SM_Ceiling_a")
    set_label("SM_Curtains_a")
    set_label("SM_Floor_a")
    set_label("SM_Walls_a")
    set_label("SM_Windows_glass_a")
    set_label("SM_Windows_Windows_a")
    set_label("SM_TV_Wall_a")

    panoptic_count = counter[0] - 1
    # Instances
    set_label("SM_Bed_a")
    set_label("SM_Bed_lamp_a")
    set_label("SM_Bed_lamp_b", increment_class=False)
    set_label("SM_Bed_table_a")
    set_label("SM_Bed_table_b", increment_class=False)
    set_label("SM_Bottle_a")
    set_label("SM_Ceiling_lamp_a")
    [set_label("SM_Ceiling_lamp_%s" % letters[x], increment_class=False) for x in range(1, 12)]
    set_label("SM_Chair_a")
    set_label("SM_Chair_b", increment_class=False)
    set_label("SM_Kitchen_Chair_a", increment_class=False)
    set_label("SM_Coffee_table_a")
    [set_label("SM_Cup_%s" % letters[x], increment_class=False) for x in range(3)]
    set_label("SM_Decor_a")
    set_label("SM_Decor_b", increment_class=False)
    set_label("SM_Digital_Clock_a")
    set_label("SM_Dimmer_a")
    set_label("SM_Dimmer_b", increment_class=False)
    set_label("SM_Door_a")
    set_label("SM_Door_b", increment_class=False)
    set_label("SM_Floor_Lamp_a")
    set_label("SM_Journal_a")
    [set_label("SM_Journal_%s" % letters[x], increment_class=False) for x in range(1, 3)]
    set_label("SM_Kitchen_a")
    set_label("SM_Picture_a")
    [set_label("SM_Picture_%s" % letters[x], increment_class=False) for x in range(1, 7)]
    set_label("SM_Plant_a")
    set_label("SM_Plate_a")
    [set_label("SM_Plate_%s" % letters[x], increment_class=False) for x in range(1, 3)]
    set_label("SM_Remote_a")
    set_label("SM_Sofa_a")
    set_label("SM_Stack_of_Books_a")
    [set_label("SM_Stack_of_Books_%s" % letters[x], increment_class=False) for x in range(1, 6)]
    set_label("SM_Table_a")
    set_label("SM_Table_Decor_a")
    set_label("SM_Tumblr_a")
    set_label("SM_Tumblr_b", increment_class=False)
    set_label("SM_TV_a")
    set_label("SM_Wall_Clock_a")
    set_label("SM_Coffee_Machine_a")
    set_label("SM_Nightstand_a")

    print("Created a labelling with %i instances and %i classes." % (counter[0] - 1, class_counter[0] - 1))
    return label_ids, class_ids, mesh_ids, panoptic_count


def apply_labels(mesh_labels):
    """
    Set the segmentation id for all object in unreal based on the info in 'mesh_labels'
    """
    client = airsim.MultirotorClient()
    success = 0
    client.confirmConnection()
    # reset the labels of everything to invisible
    client.simSetSegmentationObjectID("[\w]*", -1, True)
    for key in mesh_labels:
        if client.simSetSegmentationObjectID(key, mesh_labels[key][0], False):
            success = success + 1
            print("Successfully set label for '%s'." % key)
        else:
            print("Failed to set label for '%s'." % key)
    print("Applied %i labels." % success)


def get_available_meshes(comparison_labels=None):
    """
    Print the names of all meshes in the unreal world. If comparison_labels is set print how many of these are uniquely
    matched with the existing names.
    """
    client = airsim.MultirotorClient()
    client.confirmConnection()
    names = client.simListSceneObjects("[\w]*")
    names = [str(name) for name in names]
    counts = []
    print("Available mesh names: ", sorted(names))
    for name in sorted(names):
        print("Found Mesh '%s'" % name)
        # print("Mesh '%s' has id '%i'" % (name, client.simGetSegmentationObjectID(name)))
    if comparison_labels is not None:
        for key in comparison_labels:
            matches = [name == key for name in names]
            counts.append(np.sum(np.array(matches)))
        print("Label names found: ", counts)
        print("Unique labels matched: %.1f percent" % (np.mean(np.array(counts) == 1) * 100))


def get_infrared_correction(target_file):
    """
    Compute the transfer dynamics from segmentation id in unreal and the value in the infrared image and save it to file.
    """
    client = airsim.MultirotorClient()
    client.confirmConnection()
    counter = 0
    print("Computing infrared corrections ...")
    with open(target_file, 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(["MeshID", "InfraRedID"])
        for i in range(256):
            client.simSetSegmentationObjectID("[\w]*", i, True)
            responses = client.simGetImages([airsim.ImageRequest("Id_cam", airsim.ImageType.Infrared, False, False)])
            response = responses[0]
            img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
            writer.writerow([i, img1d[0]])
            if i * 100 / 256 > counter:
                counter = counter + 5
                print("%i percent done ..." % counter)
    print("Saved infrared corrections in '%s'." % target_file)


def export_labels(labels, classes, mesh_ids, panoptic_count, out_file_name):
    """
    Save label data to file.
    """
    color_palette = misc.imread(
        "/home/lukas/programs/AirSim/Unreal/Plugins/AirSim/Content/HUDAssets/seg_color_pallet.png")
    with open(out_file_name, 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(["InstanceID", "ClassID", "PanopticID", "MeshID", "InfraredID", "R", "G", "B", "Name", ])
        keys = [key for key in labels]
        keys = sorted(keys, key=lambda x: labels(x))
        for key in keys:
            id = labels[key]
            mesh_id = mesh_ids[key]
            writer.writerow([id, classes[key], int(id <= panoptic_count), mesh_id[0], mesh_id[1],
                             color_palette[0, mesh_id[0] * 4, 0], color_palette[0, mesh_id[0], 1],
                             color_palette[0, mesh_id[0], 2], key])
        writer.writerow([255, 255, 1, 255, 255, 80, 80, 80, "Unknown"])
    print("Saved labels in '%s'." % out_file_name)


if __name__ == "__main__":
    get_infrared_correction("/home/lukas/Documents/PanopticMapping/Data/infrared_corrections.csv");
    labels, classes, mesh_ids, panoptic_count = create_label_ids("/home/lukas/Documents/PanopticMapping/Data/infrared_corrections.csv")
    # get_available_meshes(labels)
    apply_labels(mesh_ids)
    export_labels(labels, classes, mesh_ids, panoptic_count, "/home/lukas/Documents/PanopticMapping/Data/labels.csv")