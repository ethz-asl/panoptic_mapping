import argparse
import csv
import json
import re
import time
import tempfile
from pathlib import Path
from typing import List, Optional
from zipfile import ZipFile

import numpy as np
from PIL import Image

from scannetv2_to_nyu40 import SCANNETV2_TO_NYU40

_LABEL_DIR_NAME = "label-filt"
_INSTANCE_DIR_NAME = "instance-filt"
_LABEL_ARCHIVE_NAME_PATTERN = "{}_2d-label-filt.zip"
_INSTANCE_ARCHIVE_NAME_PATTERN = "{}_2d-instance-filt.zip"
_LABEL_DETECTRON_DIR = "label-detectron"
_NYU40_STUFF_CLASSES = [1, 2, 22]
_DEFAULT_THING_SCORE = 0.9
_TIMESTAMPS_FILE = "timestamps.csv"
_FRAME_PERIOD_NS = 33333333
_PANOPTIC_LABELS_CHOICES = ["groundtruth", "deeplab"]


def get_scan_number_of_color_frames(scan_dir_path: Path):

    scan_info_file_path = scan_dir_path / (scan_dir_path.stem + ".txt")
    assert (
        scan_info_file_path.exists()
    ), f"Scene info file not found for {scan_dir_path.stem}"
    with scan_info_file_path.open("r") as f:
        for line in f:
            if re.search("numColorFrames", line):
                num_color_frames = int(line.rstrip("\n").split(" ")[2])
                return num_color_frames


def make_detectron_stuff_label(
    segment_id,
    class_id,
    area,
):
    return {
        "id": int(segment_id),
        "isthing": False,
        "category_id": int(class_id),
        "area": int(area),
    }


def make_detectron_thing_label(
    segment_id,
    class_id,
    instance_id,
    area,
    score=_DEFAULT_THING_SCORE,
):
    return {
        "id": int(segment_id),
        "isthing": True,
        "category_id": int(class_id),
        "instance_id": int(instance_id),
        "area": int(area),
        "score": float(score),
    }


def make_predicted_map_and_segment_labels(
    instance_map: np.ndarray,
    labels: np.ndarray,
    instance_scores: List = None,
):
    predicted_map = np.zeros_like(instance_map)
    segment_labels = []

    next_new_segment_id = 1
    next_new_instance_id = 0
    for class_id in np.unique(labels):
        if class_id == 0:
            continue

        class_mask = labels == class_id

        if class_id in _NYU40_STUFF_CLASSES:
            area = np.count_nonzero(class_mask)
            predicted_map[class_mask] = next_new_segment_id
            segment_labels.append(
                make_detectron_stuff_label(next_new_segment_id, class_id, area)
            )
            next_new_segment_id += 1
            continue

        for instance_id in np.unique(instance_map[class_mask]):
            if instance_id == 0:
                continue
            instance_mask = instance_map == instance_id
            area = np.count_nonzero(instance_mask)

            predicted_map[instance_mask] = next_new_segment_id

            if instance_scores is not None:
                instance_score = next(
                    (
                        x["score"]
                        for x in instance_scores
                        if x["class_id"] == class_id and x["instance_id"] == instance_id
                    ),
                    0.0,
                )
            else:
                instance_score = _DEFAULT_THING_SCORE

            segment_labels.append(
                make_detectron_thing_label(
                    next_new_segment_id,
                    class_id,
                    next_new_instance_id,
                    area,
                    instance_score,
                )
            )

            next_new_segment_id += 1
            next_new_instance_id += 1

    return predicted_map, segment_labels


def convert_labels_to_nyu40(labels: np.ndarray):
    return np.vectorize(SCANNETV2_TO_NYU40.get)(labels).astype(np.uint8)


def create_panoptic_labels_from_grountruth(scan_dir_path: Path):

    with tempfile.TemporaryDirectory() as tmpdir:
        temp_dir_path = Path(tmpdir)
        # Extract instance id maps
        instance_archive_path = scan_dir_path / _INSTANCE_ARCHIVE_NAME_PATTERN.format(
            scan_dir_path.name
        )
        instance_archive = ZipFile(str(instance_archive_path))
        instance_archive.extractall(path=str(temp_dir_path))
        instance_dir_path = temp_dir_path / _INSTANCE_DIR_NAME

        # Extract semantic id maps
        label_archive_path = scan_dir_path / _LABEL_ARCHIVE_NAME_PATTERN.format(
            scan_dir_path.name
        )
        label_dir_path = temp_dir_path / _LABEL_DIR_NAME
        label_archive = ZipFile(str(label_archive_path))
        label_archive.extractall(path=str(temp_dir_path))

        label_detectron_dir_path = scan_dir_path / _LABEL_DETECTRON_DIR
        label_detectron_dir_path.mkdir(exist_ok=True)
        for labels_file_path in label_dir_path.glob("*.png"):
            instance_file_path = instance_dir_path / labels_file_path.name

            labels = np.array(Image.open(labels_file_path))
            labels_nyu40 = convert_labels_to_nyu40(labels)

            instance_map = np.array(Image.open(instance_file_path))

            predicted_map, segment_labels = make_predicted_map_and_segment_labels(
                instance_map,
                labels_nyu40,
                None,
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


def create_panoptic_labels_from_deeplab_predictions(
    scan_dir_path: Path,
    predicted_labels_dir: str,
):
    predicted_labels_dir_path = scan_dir_path / predicted_labels_dir
    assert predicted_labels_dir_path.is_dir()

    label_detectron_dir_path = scan_dir_path / _LABEL_DETECTRON_DIR
    label_detectron_dir_path.mkdir(exist_ok=True)

    for panoptic_map_file_path in predicted_labels_dir_path.glob("*.png"):
        # Load the predicted panoptic map
        panoptic_map = np.array(Image.open(panoptic_map_file_path))
        semantic_map = panoptic_map[:, :, 0]
        instance_map = panoptic_map[:, :, 1]

        # Load the instance scores file
        instance_scores_file_path = predicted_labels_dir_path / (
            panoptic_map_file_path.stem + "_instance_scores.json"
        )
        with open(instance_scores_file_path, "r") as j:
            instance_scores = json.load(j)
        predicted_map, segment_labels = make_predicted_map_and_segment_labels(
            instance_map, semantic_map, instance_scores
        )

        predicted_map_file_path = label_detectron_dir_path / (
            panoptic_map_file_path.stem + "_predicted.png"
        )
        Image.fromarray(predicted_map).save(predicted_map_file_path)

        segment_labels_file_path = label_detectron_dir_path / (
            panoptic_map_file_path.stem + "_labels.json"
        )
        with segment_labels_file_path.open("w") as f:
            json.dump(segment_labels, f)


def create_timestamps_file(scan_dir_path: Path):
    # Generate timestamps file
    next_frame_time = time.time_ns()
    timestamps_file_path = scan_dir_path / _TIMESTAMPS_FILE
    num_frames = get_scan_number_of_color_frames(scan_dir_path)
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


def prepare_scan(
    scan_dir_path: Path,
    panoptic_labels: str,
    predicted_labels_dir: Optional[str],
):
    assert scan_dir_path.is_dir()

    # Create timestamps file
    create_timestamps_file(scan_dir_path)

    # Create panoptic labels
    if panoptic_labels == "groundtruth":
        create_panoptic_labels_from_grountruth(scan_dir_path)
    else:
        assert (
            predicted_labels_dir is not None
        ), "The name of directory containing predicted labels must be provided"
        create_panoptic_labels_from_deeplab_predictions(
            scan_dir_path,
            predicted_labels_dir,
        )


def _parse_args():
    desc = "Create labels compatible with the panoptic mapping framework for the given scan."

    parser = argparse.ArgumentParser(description=desc)

    parser.add_argument(
        "scan_dir_path",
        type=lambda p: Path(p).absolute(),
        help="Path to the scan directory.",
    )

    parser.add_argument(
        "--panoptic_labels",
        type=str,
        choices=_PANOPTIC_LABELS_CHOICES,
        default="groundtruth",
        help="Where panoptic labels should be generated from.",
    )

    parser.add_argument(
        "--predicted_labels_dir",
        type=str,
        default=None,
        help="Name of the subdirectory containing the predicted labels.",
    )

    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    prepare_scan(
        scan_dir_path=args.scan_dir_path,
        panoptic_labels=args.panoptic_labels,
        predicted_labels_dir=args.predicted_labels_dir,
    )
