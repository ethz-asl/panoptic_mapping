#!/usr/bin/env python

import airsim
import numpy as np
import csv
from scipy import misc


def create_label_ids():
    # labels for the flat test dataset
    label_ids = {}
    class_ids = {}
    counter = [0]
    class_counter = [0]

    def set_label(name, id=None, increment_class=True, class_counter=class_counter,
                  counter=counter, label_ids=label_ids, class_ids=class_ids):
        if id is None:
            id = counter[0]
            counter[0] = counter[0] + 1
        label_ids[name] = id
        if increment_class:
            class_counter[0] = class_counter[0] + 1
        class_ids[name] = class_counter[0]

    # Background classes
    letters = ["a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l"]
    set_label("SM_Ceiling_a")
    set_label("SM_Curtains_a")
    set_label("SM_Floor_a")
    set_label("SM_Kitchen_a")
    set_label("SM_Walls_a")
    set_label("SM_Windows_glass_a")
    set_label("SM_Windows_Windows_a")

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
    set_label("SM_Cup_a")
    [set_label("SM_Cup_%s" % letters[x-1], increment_class=False) for x in range(1, 3)]
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
    set_label("SM_TV_Wall_a")
    set_label("SM_Wall_Clock_a")
    set_label("SM_Coffee_Machine_a")
    set_label("SM_Nightstand_a")

    print("Created a labelling with %i instances and %i classes." % (counter[0] - 1, class_counter[0] - 1))
    return label_ids, class_ids, panoptic_count


def apply_labels(labels):
    client = airsim.MultirotorClient()
    success = 0
    client.confirmConnection()
    for key in labels:
        if client.simSetSegmentationObjectID(key, labels[key], False):
            success = success + 1
            print("Successfully set label '%s'." % key)
        else:
            print("Failed to set label '%s'." % key)
    print("Applied %i labels." % success)


def get_available_meshes(comparison_labels=None):
    client = airsim.MultirotorClient()
    client.confirmConnection()
    names = client.simListSceneObjects("[\w]*")
    names = [str(name) for name in names]
    counts = []
    print("Available mesh names: ", sorted(names))
    if comparison_labels is not None:
        for key in comparison_labels:
            matches = [name == key for name in names]
            counts.append(np.sum(np.array(matches)))
        print("Label names found: ", counts)
        print("Unique labels matched: %.1f percent" % np.mean(np.array(counts) == 1) * 100)


def export_labels(labels, classes, panoptic_count, out_file_name):
    color_palette = misc.imread("/home/lukas/programs/AirSim/Unreal//Plugins/AirSim/Content/HUDAssets/seg_color_pallet.png")
    with open(out_file_name, 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(["InstanceID", "ClassID", "PanopticID", "R", "G", "B", "MeshName"])
        for key in labels:
            id = labels[key]
            writer.writerow([id, classes[key], int(id <= panoptic_count), color_palette[0, id*4, 0],
                             color_palette[0, id*4, 1], color_palette[0, id*4, 2], key])
    print("Saved labels in '%s'." % out_file_name)


if __name__ == "__main__":
    labels, classes, panoptic_count = create_label_ids()
    #get_available_meshes(labels)
    apply_labels(labels)
    #export_labels(labels, classes, panoptic_count, "/home/lukas/catkin_ws/src/panoptic_mapping/data/flat_labels.csv")
