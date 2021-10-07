#!/usr/bin/env python3

import csv

import airsim
import numpy as np
import imageio


def get_ir_ids(ir_correction_file):
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
    return mesh_ids, ir_ids


def create_label_ids_flat(ir_correction_file):
    """
    Create the class and instance labels for the objects in the flat dataset.
    """
    # labels for the flat test dataset
    labels = [
    ]  # {InstanceID, ClassID, PanopticID, MeshID, InfraredID, Name, Size}

    mesh_ids, ir_ids = get_ir_ids(ir_correction_file)

    # NOTE: These are lists so they are mutable in set label.
    id_counter = [0]
    class_counter = [0]
    panotpic_id = 0
    letters = [
        "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n",
        "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z"
    ]

    def set_label(name, size="M", increment_class=True, count=1, new_id=True):
        for i in range(count):
            if i >= len(letters):
                print("Warning: can only write %i (request is %i) suffixes "
                      "using 'letters'." % (len(letters), i))
            full_name = name + "_" + letters[i]
            label = {}

            # Instance ID
            label_id = id_counter[0]
            if new_id:
                id_counter[0] = id_counter[0] + 1
            label["InstanceID"] = label_id
            label["Name"] = full_name
            label["Size"] = size

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
    set_label("SM_Floor")
    set_label("SM_Walls")
    set_label("SM_Windows_glass")
    set_label("SM_Windows_Windows")
    set_label("SM_TV_Wall")

    # Instances
    panotpic_id = 1
    set_label("SM_Bed", "L", count=2)
    set_label("SM_Bed_lamp", count=2)
    set_label("SM_Bed_table", count=2)
    set_label("SM_Ceiling_lamp", count=12)
    set_label("SM_Chair", count=2)
    set_label("SM_Office_Chair_base", increment_class=False, new_id=False)
    set_label("SM_Office_Chair_seat", increment_class=False)
    set_label("SM_Coffee_table")
    set_label("SM_Cup", "S", count=3)
    set_label("SM_Decor", "S", count=2)
    set_label("SM_Digital_Clock")
    set_label("SM_Dimmer", count=2)
    set_label("SM_Door", count=2)
    set_label("SM_Floor_Lamp")
    set_label("SM_Journal", "S", count=3)
    set_label("SM_Kitchen", "L")
    set_label("SM_Picture", count=7)
    set_label("SM_Plant", "S")
    set_label("SM_Plate", "S", count=3)
    set_label("SM_Remote", "S")
    set_label("SM_Sofa", "L")
    set_label("SM_Stack_of_Books", "S", count=6)
    set_label("SM_Table", count=2)
    set_label("SM_Table_Decor", "S")
    set_label("SM_Tumblr", "S", count=2)
    set_label("SM_TV")
    set_label("SM_Wall_Clock")
    set_label("SM_Coffee_Machine", "S")
    id_counter[0] = id_counter[0] + 1
    set_label("SM_Laptop",
              size="S",
              count=2,
              increment_class=False,
              new_id=False)

    print("Created a labeling with %i instances and %i classes." %
          (id_counter[0], class_counter[0]))
    return labels


class Labler(object):
    def __init__(self, ir_correction_file):
        self.panoptic_id = 0
        self.instance_counter = 0
        self.mesh_ids, self.ir_ids = get_ir_ids(ir_correction_file)
        self.labels = [
        ]  # {InstanceID, ClassID, PanopticID, MeshID, InfraredID, Name}
        self.meshes_to_label = {}  # {mesh_name: id}
        self.class_labels = {}  # {class_name: [id, instance counter]}
        self.class_counter = 0

    def set_panoptic_label(self, label):
        self.panoptic_id = label  # 0 = background, 1 = instance

    def get_labels(self):
        return self.labels

    def get_meshes_to_label(self):
        return self.meshes_to_label

    def add_instance(self, mesh_names, class_name):
        """ mesh names: list of all meshnames to include, 
        class_name: string of class name """
        # Instance
        label_id = self.instance_counter
        self.instance_counter = self.instance_counter + 1
        label = {"InstanceID": label_id}

        # Class and Name
        if class_name in self.class_labels:
            self.class_labels[class_name][
                1] = self.class_labels[class_name][1] + 1
        else:
            self.class_labels[class_name] = [self.class_counter, 0]
            self.class_counter = self.class_counter + 1
        label["Name"] = class_name + "_" + str(
            self.class_labels[class_name][1])
        label["ClassID"] = self.class_labels[class_name][0]

        # IR and mesh
        if label_id < len(self.mesh_ids):
            label["MeshID"] = self.mesh_ids[label_id]
            label["InfraredID"] = self.ir_ids[label_id]
        else:
            print("Warning: id '%i' is larger than the maximum supported "
                  "id count of %i." % (label_id, len(self.mesh_ids)))
            label["MeshID"] = -1
            label["InfraredID"] = -1

        # Panoptic
        label["PanopticID"] = self.panoptic_id

        # Write
        self.labels.append(label)
        for m in mesh_names:
            self.meshes_to_label[m] = label_id


def create_label_ids_large_flat(ir_correction_file):
    labler = Labler(ir_correction_file)

    # Background
    labler.add_instance([
        "Walls_StaticMesh", "SM_frame_door09_25", "SM_frame_door10",
        "SM_frame_door11", "SM_frame_door12", "SM_frame_door13",
        "SM_frame_door14", "SM_frame_door15", "SM_frame_door16",
        "SM_frame_door17"
    ], "wall")
    labler.add_instance(["Floor_StaticMesh"], "floor")
    labler.add_instance(["Ceiling_StaticMesh"], "ceiling")
    labler.add_instance(["pr_background_mountain_a_4_summer_16", "Ground_195"],
                        "landscape")
    labler.add_instance([
        "SM_window07_frame_14", "SM_window07_leaf01_22",
        "SM_window26_leaf01_8", "SM_window26_frame2_134", "SM_window26_frame3"
    ] + ["SM_window07_frame%i" % i for i in range(2, 8)] +
                        ["SM_window07_leaf%i" % i for i in range(2, 15)] +
                        ["SM_window26_leaf%i" % i
                         for i in range(2, 5)], "window")

    # Instances
    labler.set_panoptic_label(1)

    # Kitchen
    labler.add_instance([
        "SM_Kitchen_KNOXHULT_Exhaust_Hood3",
        "SM_Kitchen_KNOXHULT_FloorCabinet_Boxes3",
        "SM_Kitchen_KNOXHULT_FloorCabinet_Doors3",
        "SM_Kitchen_KNOXHULT_FloorCabinet_TableTop3",
        "SM_Kitchen_KNOXHULT_Lamps3", "SM_Kitchen_KNOXHULT_Sink3",
        "SM_Kitchen_KNOXHULT_Tile_4", "SM_Kitchen_KNOXHULT_WallCabinet_Boxes3",
        "SM_Kitchen_KNOXHULT_WallCabinet_Doors3"
    ], "kitchen")
    labler.add_instance(["SM_Refregerator_01_140"], "refregerator")
    labler.add_instance(
        ["SM_Stand3", "SM_Spice_jar11", "SM_Spice_jar12", "SM_Spice_jar13"],
        "spices")
    for i in range(11, 14):
        labler.add_instance(["SM_Jar_%i" % i], "jar")
    for i in range(14, 17):
        labler.add_instance(["SM_Box_%i" % i], "box")
    labler.add_instance(["SM_Plate_%i" % i for i in range(17, 21)], "plates")
    labler.add_instance(["SM_Tool_3"], "tool")
    labler.add_instance(["SM_Tools3"], "tool")
    labler.add_instance(["SM_Stand_3"], "stand")
    labler.add_instance(["SM_Cuttingboard_3"], "cuttingboard")
    labler.add_instance(["SM_Cuttingboard_4"], "cuttingboard")
    labler.add_instance(["SM_Cloth_3"], "cloth")
    labler.add_instance(["SM_Knives_Stand_Hivla_3"], "knives")
    labler.add_instance(["SM_Bowl_%i" % i for i in range(13, 17)], "bowls")
    labler.add_instance(["SM_Jar3", "SM_Spice_jar14", "SM_Spice_jar15"],
                        "spices")
    labler.add_instance(["SM_Plate_%i" % i for i in range(21, 25)], "plates")
    labler.add_instance(["SM_Cup_3"], "cup")
    labler.add_instance(["SM_Yukke_Watch3"], "clock")

    # Dining Room
    labler.add_instance(["SM_Table_Lisabo_01_26"], "table")
    for s in ["_8", "2_11", "4_17", "5_20", "6_23", 7]:
        labler.add_instance(["SM_Table_chair%s" % s], "chair")
    labler.add_instance(["SM_Light_Hektar_3"], "lamp")
    labler.add_instance(["SM_Light_Hektar_4"], "lamp")
    labler.add_instance(["SM_Cup_01_149"], "cup")
    labler.add_instance(["SM_Cup_02_143"], "cup")
    labler.add_instance(["ChineseFoodBox_A_141"], "food")
    labler.add_instance(["SM_Stack_of_Books_260"], "books")
    labler.add_instance(["SM_Cup_146"], "cup")

    # Living Room
    labler.add_instance(["SM_Carpet_5"], "carpet")
    labler.add_instance([
        "SM_Sofa_01_Fold_32", "SM_Pillow_Sofa_03_38", "SM_Pillow_Sofa_4",
        "SM_Pillow_Sofa_4"
    ], "sofa")
    labler.add_instance(["SM_Pillow_03_43"], "pillow")
    labler.add_instance(["SM_Pillow_04_49"], "pillow")
    labler.add_instance(["Armchair_dense_27"], "chair")
    labler.add_instance(["EdithLivingRoomTable_18"], "table")
    labler.add_instance(["SM_Remote_110"], "remote")
    labler.add_instance(["SM_Table_Decor_21"], "decor")
    labler.add_instance(["SM_Decor_24"], "decor")
    labler.add_instance(["SM_Picture_68"], "picture")
    labler.add_instance(["SM_Picture_71"], "picture")
    labler.add_instance(["SM_Wall_TV_2"], "tv")
    labler.add_instance([
        "SM_TVSet_SM_MailUnitBdvE4102", "SM_TVSet_SM_SurroundSpeakers5",
        "SM_TVSet_SM_SurroundSpeakers6", "SM_TVSet_SM_centerpeaker2"
    ], "soundsystem")
    labler.add_instance([
        "SM_RackKallax_2x5", "SM_RackKallax_2x6", "SM_RackKallax_2x7",
        "SM_BoxDoorKallax_12", "SM_BoxDoorKallax_14", "SM_BoxDoorKallax_23",
        "SM_BoxDoorKallax_24", "SM_BoxTiena_5", "SM_BoxDrena_9",
        "SM_BoxDrena_8", "SM_Box9", "SM_Box8_151"
    ], "shelf")

    # Entry Area
    labler.add_instance(["SM_MainDoor_30"], "door")
    labler.add_instance(["SM_Rack_4x2_92"], "shelf")
    labler.add_instance(["SM_Stack_of_Books_101"], "books")
    labler.add_instance(["SM_Stack_of_Books_98"], "books")
    labler.add_instance(["SM_Stack_of_Books_95"], "books")
    labler.add_instance(["SM_Plant_107"], "plant")
    labler.add_instance(["SM_Decor_104"], "decor")
    labler.add_instance(["Shoe_shelf_Hemnes3", "SM_Coat_hanger_2"] +
                        ["SM_Hanger_%i" % i for i in range(23, 27)],
                        "wardrobe")
    labler.add_instance(["SM_Picture_140x100_74"], "picture")

    # Toilet
    labler.add_instance(
        ["SM_Toilet_02_229", "SM_ToiletPaper_02_235", "SM_Toiletbrush_03_232"],
        "toilet")
    labler.add_instance(["SM_Washbasin_2", "SM_BathroomSinkMixer_5"], "basin")
    labler.add_instance(["SM_Mirror_06_8"], "mirror")

    # Washing Room
    labler.add_instance(["SM_WashingMachine_38"], "washingmachine")
    labler.add_instance(["DawnDresser_a_161"], "dresser")
    labler.add_instance(
        ["SM_Door_80_7", "SM_handleMetal11", "SM_handleMetal12"], "door")
    labler.add_instance(
        ["SM_Door_80_8", "SM_handleMetal13", "SM_handleMetal14"], "door")
    labler.add_instance(["Fiddle_a_144"], "fiddle")
    labler.add_instance(["WateringCan_152"], "wateringcan")

    # Floor
    labler.add_instance(["DresserWood_b_123"], "dresser")
    labler.add_instance(["DawnDresser_a2_167"], "dresser")
    labler.add_instance(["EdithPhoto_a_170"], "picture")
    labler.add_instance(["FloorBottle_c_179"], "decor")

    # Bath
    labler.add_instance(["SM_Toilet_03_68"], "toilet")
    labler.add_instance(["SM_Washbasin_03_188", "SM_BathroomSinkMixer_02_105"],
                        "basin")
    labler.add_instance(["SM_Mirror_01_76"], "mirror")
    labler.add_instance([
        "SM_Cosmetics_4_01_120", "SM_Cosmetics_04_01_111",
        "SM_Cosmetics_3_01_123"
    ], "cosmetics")
    labler.add_instance([
        "SM_ToothbrushHolder_02_126", "SM_Cosmetics_04_02_114",
        "SM_Cosmetics_1_01_117"
    ], "cosmetics")
    labler.add_instance(["SM_RecycleBin_01_108"], "bin")
    labler.add_instance(["SM_Branas_102"], "basket")
    labler.add_instance(["SM_BathCarpet_01_27"], "carpet")
    labler.add_instance(["SM_Bath_01_71", "SM_Shower_01_SM_Shower_01_79"],
                        "bath")
    labler.add_instance(["SM_Branas_01_98"], "basket")
    labler.add_instance(["SM_WashingMachine_01_92"], "washingmachine")

    # Bedroom
    labler.add_instance(["SM_P1sh12_%s" % i for i in [158, 174, 181, 184]] +
                        ["SM_Hanger_%i" % i for i in range(14, 23)], "shelf")
    labler.add_instance(["SM_Laundry_Basket_01_187"], "basket")
    labler.add_instance(["SM_Pillow_%i" % i for i in range(8, 12)], "pillows")
    labler.add_instance(["SM_Carpet_11"], "carpet")
    labler.add_instance(["SM_Bed_03_59"], "bed")
    labler.add_instance(["SM_Bed_table_62"], "table")
    labler.add_instance(["SM_Journal_263"], "books")
    labler.add_instance(["SM_WallLamp_01_58"], "lamp")
    labler.add_instance(["SM_Nightstand_01_197"], "nightstand")
    labler.add_instance(["SM_TableLamp_02_61"], "lamp")
    labler.add_instance(["SM_Digital_Clock_266"], "clock")
    labler.add_instance(["SM_Picture_194"], "picture")
    labler.add_instance([
        "SM_RackKallax_4x5_189", "SM_BoxDoorKallax_25", "SM_BoxFiella_6",
        "SM_BoxDoorKallax_26"
    ], "shelf")
    labler.add_instance(["SM_GrassBox_7", "SM_GrassBox_8"], "plant")
    labler.add_instance(["SM_Curtain_3"], "curtain")
    labler.add_instance(["SM_Curtain_02_44"], "curtain")

    # Guest Room
    labler.add_instance(["EdithRoomBed_a_129"], "bed")
    labler.add_instance(["SM_Nightstand_220"], "nightstand")
    labler.add_instance(["SM_Picture_120x120_138"], "picture")
    labler.add_instance(["SM_Sofa_3"], "chair")
    labler.add_instance(
        ["SM_Notebook_01_keyboard_223", "SM_Notebook_01_top_226"], "pc")
    labler.add_instance(
        ["SM_Door_80_6", "SM_handleMetal10", "SM_handleMetal9"], "door")
    labler.add_instance(["SM_Picture_140x101_117"], "picture")
    labler.add_instance(["SM_Metal_Rack_2"], "shelf")
    labler.add_instance(["SM_BoxFiella_7"], "box")
    labler.add_instance(["SM_BoxTiena_7", "SM_BoxTiena_7"], "box")
    labler.add_instance([
        "SM_Pen_Stand_Tiera_01_v4", "SM_Marker_01_SM_Marker_01_Cap5",
        "SM_Marker_01_SM_Marker_01_Cap5", "SM_Marker_01_SM_Marker_9",
        "SM_Marker_01_SM_Marker_01_Cap6", "SM_Marker_01_SM_Marker_7",
        "SM_Marker_01_SM_Marker_01_Cap4"
    ], "pens")
    labler.add_instance(["SM_Pen_Stand_Tiera_01_v5"], "pens")
    labler.add_instance(["SM_GrassBox_9"], "plant")

    # Office
    labler.add_instance(["SM_Door_80_5", "SM_handleMetal8", "SM_handleMetal7"],
                        "door")
    labler.add_instance(["SM_Picture_01_49"], "picture")
    labler.add_instance(["SM_Picture_62"], "picture")
    labler.add_instance(["SM_Table_3"], "table")
    labler.add_instance(["SM_Office_Chair_43"], "chair")
    labler.add_instance(["SM_Keyboard_01_213"], "keyboard")
    labler.add_instance(["SM_Printer_01_210"], "pc")
    labler.add_instance(["SM_Monitor_02_40"], "monitor")
    labler.add_instance(["SM_Floor_Lamp_207"], "lamp")
    labler.add_instance(["SM_Table_02_200"], "table")
    labler.add_instance(["SM_Office_Chair_01_46"], "chair")
    labler.add_instance(["SM_Monitor_01_37"], "monitor")
    labler.add_instance(["SM_Keyboard_2"], "keyboard")
    labler.add_instance(["SM_TableLamp_01_52"], "lamp")

    # Lamps
    for i in range(3, 16):
        labler.add_instance(["SM_Light_ALENG%i" % i], "lamp")
    for s in ["4_9", 5, 6, 7, 8]:
        labler.add_instance(["SM_Light_MEZOSPHERE%s" % s], "lamp")

    print("Created a labeling with %i instances and %i classes." %
          (len(labler.get_labels()), labler.class_counter))
    return labler.get_labels(), labler.get_meshes_to_label()


def apply_labels_flat(labels):
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
            print("Successfully set label for '%s'." % key)
        else:
            print("Failed to set label for '%s'." % key)
    print("Applied %i/%i labels." % (success, len(labels)))


def apply_labels_large_flat(meshes_to_label):
    """
    Set the segmentation id for all object in unreal based on the info in
    'mesh_labels'
    """
    client = airsim.MultirotorClient()
    success = 0
    client.confirmConnection()
    # Reset the labels of everything to invisible.
    client.simSetSegmentationObjectID(r"[\w]*", -1, True)
    for mesh_name in meshes_to_label:
        mesh_id = meshes_to_label[mesh_name]
        if client.simSetSegmentationObjectID(mesh_name, mesh_id, False):
            success = success + 1
            print("Successfully set label for '%s'." % mesh_name)
        else:
            print("Failed to set label for '%s'." % mesh_name)
    print("Applied %i/%i labels." % (success, len(meshes_to_label)))


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
    print("Available mesh names: ")
    for name in sorted(names):
        print(str(name))
    if comparison_labels is not None:
        for label in comparison_labels:
            matches = [name == label["Name"] for name in names]
            counts.append(np.sum(np.array(matches)))
        # TODO(schmluk): These last parts are not cleaned up, change these if
        #                the function is needed.
        print("Comparison Label names found in the scene: ", counts)
        print("Unique labels matched: %.1f percent" %
              (np.mean(np.array(counts) == 1) * 100))


def get_infrared_correction(target_file):
    """
    Compute the transfer dynamics from segmentation id in unreal and the value
    in the infrared image and save it to file.
    """
    client = airsim.MultirotorClient()
    client.confirmConnection()
    counter = 0
    print("Computing infrared corrections ...")
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
                print("%i percent done ..." % counter)
    print("Saved infrared corrections in '%s'." % target_file)


def export_labels(labels, out_file_name):
    """
    Save label data to file.
    """
    color_palette = imageio.imread(
        "/home/lukas/programs/AirSim/Unreal/Plugins/AirSim"
        "/Content/HUDAssets/seg_color_pallet.png")
    with open(out_file_name, 'w') as csvfile:
        writer = csv.writer(csvfile,
                            delimiter=',',
                            quotechar='|',
                            quoting=csv.QUOTE_MINIMAL)
        writer.writerow([
            "InstanceID", "ClassID", "PanopticID", "MeshID", "InfraredID", "R",
            "G", "B", "Name", "Size"
        ])
        previous_id = None
        for label in labels:
            if label["InstanceID"] != previous_id:
                previous_id = label["InstanceID"]
                writer.writerow([
                    label["InstanceID"], label["ClassID"], label["PanopticID"],
                    label["MeshID"], label["InfraredID"],
                    color_palette[0, label["MeshID"] * 4,
                                  0], color_palette[0, label["MeshID"] * 4, 1],
                    color_palette[0, label["MeshID"] * 4,
                                  2], label["Name"], label["Size"]
                ])
    print("Saved %i labels in '%s'." % (len(labels), out_file_name))


def main_flat(get_ir_corrections, apply_mesh_labels, export_mesh_labels):
    ir_file = "/home/lukas/Documents/Datasets/flat_dataset/" \
              "infrared_corrections.csv"
    label_file = "/home/lukas/Documents/Datasets/flat_dataset/labels.csv"
    if get_ir_corrections:
        get_infrared_correction(ir_file)
    f_labels = create_label_ids_flat(ir_file)
    if apply_mesh_labels:
        apply_labels_flat(f_labels)
    if export_mesh_labels:
        export_labels(f_labels, label_file)


def main_large_flat(get_ir_corrections, apply_mesh_labels, export_mesh_labels):
    ir_file = "/home/lukas/Documents/Datasets/large_flat_dataset/" \
              "infrared_corrections.csv"
    label_file = "/home/lukas/Documents/Datasets/large_flat_dataset/labels.csv"
    if get_ir_corrections:
        get_infrared_correction(ir_file)
    f_labels, meshes = create_label_ids_large_flat(ir_file)
    if apply_mesh_labels:
        apply_labels_large_flat(meshes)
    if export_mesh_labels:
        export_labels(f_labels, label_file)


if __name__ == "__main__":
    # Args.
    get_ir_corrections = False
    apply_mesh_labels = True
    export_mesh_labels = True

    # Run
    main_flat(get_ir_corrections, apply_mesh_labels, export_mesh_labels)
    # main_large_flat(get_ir_corrections, apply_mesh_labels, export_mesh_labels)

    # Tools.
    # get_available_meshes(f_labels)
