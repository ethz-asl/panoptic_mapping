#!/usr/bin/env python

import csv

import airsim
import numpy as np
from scipy import misc


def create_label_ids(ir_correction_file):
    """
    Create the class and instance labels for the objects in the flat dataset.
    """
    # Read infrared label mapping (because airsims infrared values are screwed)
    mesh_ids = []
    ir_ids = []  # These match in count and order.
    with open(ir_correction_file) as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='"')
        last_id = -1
        for row in reader:
            if row[0] != 'MeshID':
                if int(row[1]) > last_id:
                    last_id = int(row[1])
                    mesh_ids.append(int(row[0]))
                    ir_ids.append(last_id)

    # labels for the flat test dataset
    labels = []  # {InstanceID, ClassID, PanopticID, MeshID, InfraredID, Name}

    # NOTE: These are lists so they are mutable in set label.
    id_counter = [0]
    class_counter = [0]
    panotpic_id = 0
    letters = [
        "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n",
        "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z"
    ]

    def set_label(name, increment_class=True, count=1):
        for i in range(count):
            if i >= len(letters):
                print("Warning: can only write %i (request is %i) suffixes "
                      "using 'letters'." % (len(letters), i))
            full_name = name + "_" + letters[i]
            label = {}

            # Instance ID
            label_id = id_counter[0]
            id_counter[0] = id_counter[0] + 1
            label["InstanceID"] = label_id
            label["Name"] = full_name

            # IR and mesh
            if label_id < len(mesh_ids):
                label["MeshID"] = mesh_ids[label_id]
                label["InfraredID"] = ir_ids[label_id]
            else:
                print("Warning: id '%i' is larger than the maximum supported "
                      "id count of %i." % (label_id, len(mesh_ids)))
            # Class and Panoptic
            label["ClassID"] = class_counter[0]
            if increment_class:
                class_counter[0] = class_counter[0] + 1
            label["PanopticID"] = panotpic_id
            increment_class = False  # for multiple count
            labels.append(label)

    # Background classes
    panotpic_id = 0
    set_label("SM_Ceiling")
    set_label("SM_Curtains")
    set_label("SM_Floor")
    set_label("SM_Walls")
    set_label("SM_Windows_glass")
    set_label("SM_Windows_Windows")
    set_label("SM_TV_Wall")

    # Instances
    panotpic_id = 1
    set_label("SM_Bed")
    set_label("SM_Bed_lamp", count=2)
    set_label("SM_Bed_table", count=2)
    set_label("SM_Bottle")
    set_label("SM_Ceiling_lamp", count=12)
    set_label("SM_Chair", count=2)
    set_label("SM_Kitchen_Chair", increment_class=False)
    set_label("SM_Coffee_table")
    set_label("SM_Cup", count=3)
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
          (id_counter[0], class_counter[0]))
    return labels


def apply_labels(labels):
    """
    Set the segmentation id for all object in unreal based on the info in
    'mesh_labels'
    """
    client = airsim.MultirotorClient()
    success = 0
    client.confirmConnection()
    # Reset the labels of everything to invisible.
    client.simSetSegmentationObjectID(r"[\w]*", -1, True)
    for label in labels:
        key = label["Name"]
        if client.simSetSegmentationObjectID(key, label["MeshID"], False):
            success = success + 1
            print "Successfully set label for '%s'." % key
        else:
            print "Failed to set label for '%s'." % key
    print "Applied %i/%i labels." % (success, len(labels))


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
    print "Available mesh names: "
    for name in sorted(names):
        print str(name)
    if comparison_labels is not None:
        for label in comparison_labels:
            matches = [name == label["Name"] for name in names]
            counts.append(np.sum(np.array(matches)))
        # TODO(schmluk): These last parts are not cleaned up, change these if
        #                the function is needed.
        print "Comparison Label names found in the scene: ", counts
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


def export_labels(labels, out_file_name):
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
        for label in labels:
            writer.writerow([
                label["InstanceID"], label["ClassID"], label["PanopticID"],
                label["MeshID"], label["InfraredID"],
                color_palette[0, label["MeshID"] * 4,
                              0], color_palette[0, label["MeshID"] * 4, 1],
                color_palette[0, label["MeshID"] * 4, 2], label["Name"]
            ])
        writer.writerow([255, 255, 1, 255, 255, 80, 80, 80, "Unknown"])
    print "Saved labels in '%s'." % out_file_name


if __name__ == "__main__":
    # Args.
    get_ir_corrections = False
    apply_mesh_labels = True
    export_mesh_labels = True

    ir_file = "/home/lukas/Documents/PanopticMapping/Data/" \
              "infrared_corrections.csv"
    label_file = "/home/lukas/Documents/PanopticMapping/Data/labels.csv"

    # Run.
    if get_ir_corrections:
        get_infrared_correction(ir_file)
    f_labels = create_label_ids(ir_file)
    if apply_mesh_labels:
        apply_labels(f_labels)
    if export_mesh_labels:
        export_labels(f_labels, label_file)

    # Tools.
    # get_available_meshes(f_labels)
