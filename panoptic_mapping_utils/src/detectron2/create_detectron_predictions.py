#!/usr/bin/python
# /home/lukas/anaconda3/envs/detectron/bin/python

# import some common libraries
import numpy as np
import os
import json
import cv2
import time
import csv

import detectron2
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.data import MetadataCatalog, DatasetCatalog
from detectron2.utils.visualizer import Visualizer

from dataclasses import dataclass


@dataclass
class Params:
    target_path: str = '/home/lukas/Documents/Datasets/flat_dataset/run1'
    model: str = 'COCO-PanopticSegmentation/panoptic_fpn_R_101_3x.yaml'
    output_label_file: str = ''  # Leave empty to not write labels.


def create_labels(meta_data, output_file: str = ""):
    sizes = []
    if (output_file):
        with open(output_file, 'w') as csvfile:
            writer = csv.writer(csvfile,
                                delimiter=',',
                                quotechar='|',
                                quoting=csv.QUOTE_MINIMAL)
            writer.writerow(
                ["InstanceID", "ClassID", "PanopticID", "Name", "Size"])
            writer.writerow([0, 0, 0, "Unknown", "M"])
            id = 1
            for label in meta_data.stuff_classes:
                writer.writerow([id, id, 0, label, ''])
                id += 1
            for i, label in enumerate(meta_data.thing_classes):
                writer.writerow([id, id, 1, label, ''])  # sizes[i]])
                id += 1
        return len(meta_data.stuff_classes), "Saved %i labels in '%s'." % (
            id, output_file)
    else:
        return len(meta_data.stuff_classes), ""


def create_predictions(params: Params):
    # Verify.
    if not os.path.isdir(params.target_path):
        print("Error: Directory '%s' does not exist." % params.target_path)
        return

    # Setup model.
    print("Setting up Detectron2 model... ", end="", flush="True")
    cfg = get_cfg()
    cfg.merge_from_file(model_zoo.get_config_file(params.model))
    cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(params.model)
    cfg.MODEL.DEVICE = 'cpu'
    predictor = DefaultPredictor(cfg)
    print("done!")

    # Setup labels.
    print("Setting up labels... ", end="", flush="True")
    meta_data = MetadataCatalog.get(cfg.DATASETS.TRAIN[0])
    label_offset, msg = create_labels(meta_data, params.output_label_file)
    print("done!")
    if msg:
        print(msg)

    # Get files to parse.
    files = [
        o for o in os.listdir(params.target_path)
        if os.path.isfile(os.path.join(params.target_path, o))
        and o.endswith('_color.png')
    ]
    times = []

    # Run inference.
    msg = "Predicting %i images... " % len(files)
    for i, im_file in enumerate(files):
        print(msg + '%.1f%%' % (i / len(files) * 100, ), end='\r', flush=True)
        im = cv2.imread(os.path.join(params.target_path, im_file))

        # Predict.
        t1 = time.perf_counter()
        panoptic_seg, segments_info = predictor(im)["panoptic_seg"]
        t2 = time.perf_counter()
        times.append(t2 - t1)

        # Write output.
        file_id = im_file[:6]
        id_img = panoptic_seg.numpy()
        cv2.imwrite(
            os.path.join(params.target_path, file_id + "_predicted.png"),
            id_img)

        for segment_info in segments_info:
            if segment_info['isthing']:
                segment_info['category_id'] += label_offset
            segment_info['category_id'] += 1  # Compensate for unknown class.
        with open(os.path.join(params.target_path, file_id + "_labels.json"),
                  'w') as json_file:
            json.dump(segments_info, json_file)
    print(msg + "done!")

    # Finish.
    times = np.array(times, dtype=float) * 1000
    print("Average inference time was %.1f +/- %.1f ms per frame." %
          (np.mean(times), np.std(times)))
    print("Finished parsing '%s'." % params.target_path)


if __name__ == '__main__':
    # Params.
    params = Params()
    params.model = "COCO-PanopticSegmentation/panoptic_fpn_R_101_3x.yaml"
    params.target_path = '/home/lukas/Documents/Datasets/flat_dataset/run1'
    params.output_label_file = '/home/lukas/Documents/Datasets/flat_dataset/detectron_labels.csv'

    # Run.
    create_predictions(params)
