import argparse
import csv
import json
import time
from pathlib import Path
from typing import List
from zipfile import ZipFile

import numpy as np
from PIL import Image

from scannetv2_to_nyu40 import SCANNETV2_TO_NYU40

_LABEL_DIR = "label-filt"
_INSTANCE_DIR = "instance-filt"
_DETECTRON_LABEL_DIR = "label-detectron"
_NYU40_STUFF_CLASSES = [1, 2, 22]
_DEFAULT_THING_SCORE = 0.9
_TIMESTAMPS_FILE = "timestamps.csv"
_FRAME_PERIOD_NS = 33333333


def make_predicted_map_and_segment_labels(
    instance_map: np.ndarray,
    labels: np.ndarray,
    stuff_classes: List[int],
):
    predicted_map = np.zeros_like(instance_map)
    segment_labels = []

    next_new_segment_id = 1
    next_new_instance_id = 0
    for class_id in np.unique(labels):
        if class_id == 0:
            continue

        class_mask = labels == class_id

        if class_id in stuff_classes:
            area = np.count_nonzero(class_mask)
            predicted_map[class_mask] = next_new_segment_id
            segment_labels.append(
                {
                    "id": next_new_segment_id,
                    "isthing": False,
                    "category_id": int(class_id),
                    "area": int(area),
                }
            )
            next_new_segment_id += 1
            continue

        for instance_id in np.unique(instance_map[class_mask]):
            if instance_id == 0:
                continue
            instance_mask = instance_map == instance_id
            area = np.count_nonzero(instance_mask)

            predicted_map[instance_mask] = next_new_segment_id

            segment_labels.append(
                {
                    "id": next_new_segment_id,
                    "isthing": True,
                    "score": _DEFAULT_THING_SCORE,
                    "category_id": int(class_id),
                    "instance_id": next_new_instance_id,
                    "area": int(area),
                }
            )

            next_new_segment_id += 1
            next_new_instance_id += 1

    return predicted_map, segment_labels


def convert_labels_to_nyu40(labels: np.ndarray):
    return np.vectorize(SCANNETV2_TO_NYU40.get)(labels).astype(np.uint8)


def create_detectron_labels_from_groundtruth(
    scan_dir_path: Path,
    instance_dir: str,
    label_dir: str,
    out_dir: str,
):
    assert scan_dir_path.is_dir()

    instance_dir_path = scan_dir_path / instance_dir
    if not instance_dir_path.exists():
        instance_archive_glob_pattern = f"*{instance_dir}.zip"
        # Try to extract the archive
        try:
            instance_archive_path = list(
                scan_dir_path.glob(instance_archive_glob_pattern)
            )[0]
        except IndexError:
            print("Instance archive or dir not found!")
            return
        instance_archive = ZipFile(str(instance_archive_path))
        instance_archive.extractall(path=str(scan_dir_path))

    label_dir_path = scan_dir_path / label_dir
    if not label_dir_path.exists():
        # Try to extract the archive
        label_archive_glob_pattern = f"*{label_dir}.zip"
        try:
            label_archive_path = list(scan_dir_path.glob(label_archive_glob_pattern))[0]
        except IndexError:
            print("Instance archive or dir not found!")
            return
        label_archive = ZipFile(str(label_archive_path))
        label_archive.extractall(path=str(scan_dir_path))

    label_detectron_dir_path = scan_dir_path /out_dir
    label_detectron_dir_path.mkdir(exist_ok=True)
    num_frames = 0
    for labels_file_path in label_dir_path.glob("*.png"):
        instance_file_path = instance_dir_path / labels_file_path.name

        labels = np.array(Image.open(labels_file_path))
        labels_nyu40 = convert_labels_to_nyu40(labels)

        instance_map = np.array(Image.open(instance_file_path))

        predicted_map, segment_labels = make_predicted_map_and_segment_labels(
            instance_map,
            labels_nyu40,
            _NYU40_STUFF_CLASSES,
        )

        predicted_map_file_path = label_detectron_dir_path / (
            instance_file_path.stem + "_predicted.png"
        )
        Image.fromarray(predicted_map).save(predicted_map_file_path)

        segment_labels_file_path = label_detectron_dir_path / (
            instance_file_path.stem + "_labels.json"
        )
        with segment_labels_file_path.open("w") as f:
            json.dump(segment_labels, f)

        num_frames += 1

    # Generate timestamps file
    next_frame_time = time.time_ns()
    timestamps_file_path = scan_dir_path / _TIMESTAMPS_FILE
    with open(timestamps_file_path, "w") as f:
        writer = csv.DictWriter(f, fieldnames=["FrameID", "TimeStamp"])
        writer.writeheader()
        for i in range(num_frames):
            writer.writerow(
                {
                    "FrameID": i,
                    "TimeStamp": next_frame_time,
                }
            )
            next_frame_time += _FRAME_PERIOD_NS


def _parse_args():
    desc = "Create labels compatible with the panoptic mapping framework for the given scan."

    parser = argparse.ArgumentParser(description=desc)

    parser.add_argument(
        "scan_dir_path",
        type=lambda p: Path(p).expanduser(),
        help="Path to the scan directory.",
    )

    parser.add_argument(
        "--instance-dir",
        type=str,
        default=_INSTANCE_DIR,
        help="Name of the subdirectory containing instance maps.",
    )

    parser.add_argument(
        "--label-dir",
        type=str,
        default=_LABEL_DIR,
        help="Name of the subdirectory containing semantic labels.",
    )

    parser.add_argument(
        "--out-dir",
        type=str,
        default=_DETECTRON_LABEL_DIR,
        help="Name of the subdirectory where the generated labels will be saved.",
    )

    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    create_detectron_labels_from_groundtruth(
        scan_dir_path=args.scan_dir_path,
        instance_dir=args.instance_dir,
        label_dir=args.label_dir,
        out_dir=args.out_dir,
    )
