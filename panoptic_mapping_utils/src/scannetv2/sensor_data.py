"""
The source code in this file is based on:
https://github.com/ScanNet/ScanNet/blob/master/SensReader/python/SensorData.py

Copyright 2017 
Angela Dai, Angel X. Chang, Manolis Savva, Maciej Halber, Thomas Funkhouser, 
Matthias Niessner

Permission is hereby granted, free of charge, to any person obtaining a copy of 
this software and associated documentation files (the "Software"), to deal in 
the Software without restriction, including without limitation the rights to 
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies 
of the Software, and to permit persons to whom the Software is furnished to do 
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all 
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
SOFTWARE.
"""

import os
import struct
import csv
from typing import Tuple

import numpy as np
import zlib
import imageio
import cv2
import png

COMPRESSION_TYPE_COLOR = {-1: "unknown", 0: "raw", 1: "png", 2: "jpeg"}
COMPRESSION_TYPE_DEPTH = {
    -1: "unknown",
    0: "raw_ushort",
    1: "zlib_ushort",
    2: "occi_ushort",
}


def format_frame_number_with_leading_zeros(frame: int) -> str:
    """
    Formats the given number as a 5-digit with leading zeros
    """
    return "{:05d}".format(frame)


def adjust_intrinsic_matrix(
    intrinsic_matrix: np.ndarray,
    original_image_size: Tuple[int, int],
    new_image_size: Tuple[int, int],
):
    rx = new_image_size[0] / original_image_size[0]
    ry = new_image_size[1] / original_image_size[1]
    S = np.diag([rx, ry, 1, 1])
    return S.dot(intrinsic_matrix)


class RGBDFrame:
    def load(self, file_handle):
        self.camera_to_world = np.asarray(
            struct.unpack("f" * 16, file_handle.read(16 * 4)), dtype=np.float32
        ).reshape(4, 4)
        self.timestamp_color = struct.unpack("Q", file_handle.read(8))[0]
        self.timestamp_depth = struct.unpack("Q", file_handle.read(8))[0]
        self.color_size_bytes = struct.unpack("Q", file_handle.read(8))[0]
        self.depth_size_bytes = struct.unpack("Q", file_handle.read(8))[0]
        self.color_data = b"".join(
            struct.unpack(
                "c" * self.color_size_bytes, file_handle.read(self.color_size_bytes)
            )
        )
        self.depth_data = b"".join(
            struct.unpack(
                "c" * self.depth_size_bytes, file_handle.read(self.depth_size_bytes)
            )
        )

    def decompress_depth(self, compression_type):
        if compression_type == "zlib_ushort":
            return self.decompress_depth_zlib()
        else:
            raise

    def decompress_depth_zlib(self):
        return zlib.decompress(self.depth_data)

    def decompress_color(self, compression_type):
        if compression_type == "jpeg":
            return self.decompress_color_jpeg()
        else:
            raise

    def decompress_color_jpeg(self):
        return imageio.imread(self.color_data)


class IMUFrame:
    def load(self, file_handle):
        self.rotation_rate = np.asarray(
            struct.unpack("d" * 3, file_handle.read(3 * 8)), dtype=np.float64
        )
        self.acceleration = np.asarray(
            struct.unpack("d" * 3, file_handle.read(3 * 8)), dtype=np.float64
        )
        self.magnetic_field = np.asarray(
            struct.unpack("d" * 3, file_handle.read(3 * 8)), dtype=np.float64
        )
        self.attitude = np.asarray(
            struct.unpack("d" * 3, file_handle.read(3 * 8)), dtype=np.float64
        )
        self.gravity = np.asarray(
            struct.unpack("d" * 3, file_handle.read(3 * 8)), dtype=np.float64
        )
        self.timestamp = struct.unpack("Q", file_handle.read(8))[0]


class SensorData:
    def __init__(self, filename):
        self.version = 4
        self.load(filename)

    def load(self, filename):
        with open(filename, "rb") as f:
            version = struct.unpack("I", f.read(4))[0]
            assert self.version == version
            strlen = struct.unpack("Q", f.read(8))[0]
            self.sensor_name = b"".join(struct.unpack("c" * strlen, f.read(strlen)))
            self.intrinsic_color = np.asarray(
                struct.unpack("f" * 16, f.read(16 * 4)), dtype=np.float32
            ).reshape(4, 4)
            self.extrinsic_color = np.asarray(
                struct.unpack("f" * 16, f.read(16 * 4)), dtype=np.float32
            ).reshape(4, 4)
            self.intrinsic_depth = np.asarray(
                struct.unpack("f" * 16, f.read(16 * 4)), dtype=np.float32
            ).reshape(4, 4)
            self.extrinsic_depth = np.asarray(
                struct.unpack("f" * 16, f.read(16 * 4)), dtype=np.float32
            ).reshape(4, 4)
            self.color_compression_type = COMPRESSION_TYPE_COLOR[
                struct.unpack("i", f.read(4))[0]
            ]
            self.depth_compression_type = COMPRESSION_TYPE_DEPTH[
                struct.unpack("i", f.read(4))[0]
            ]
            self.color_width = struct.unpack("I", f.read(4))[0]
            self.color_height = struct.unpack("I", f.read(4))[0]
            self.depth_width = struct.unpack("I", f.read(4))[0]
            self.depth_height = struct.unpack("I", f.read(4))[0]
            self.depth_shift = struct.unpack("f", f.read(4))[0]
            num_frames = struct.unpack("Q", f.read(8))[0]
            self.frames = []
            for i in range(num_frames):
                frame = RGBDFrame()
                frame.load(f)
                self.frames.append(frame)

            num_imu_frames = struct.unpack("Q", f.read(8))[0]
            self.imu_frames = []
            for i in range(num_imu_frames):
                imu_frame = IMUFrame()
                imu_frame.load(f)
                self.imu_frames.append(imu_frame)

    def export_depth_images(self, output_path, image_size=None, frame_skip=1):
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        print(
            "exporting", len(self.frames) // frame_skip, " depth frames to", output_path
        )
        for f in range(0, len(self.frames), frame_skip):
            depth_data = self.frames[f].decompress_depth(self.depth_compression_type)
            depth = np.fromstring(depth_data, dtype=np.uint16).reshape(
                self.depth_height, self.depth_width
            )
            if image_size is not None:
                depth = cv2.resize(
                    depth,
                    (image_size[0], image_size[1]),
                    interpolation=cv2.INTER_NEAREST,
                )
            formatted_frame_no = format_frame_number_with_leading_zeros(f)
            with open(
                os.path.join(output_path, formatted_frame_no + ".png"), "wb"
            ) as fh:  # write 16-bit
                writer = png.Writer(
                    width=depth.shape[1], height=depth.shape[0], bitdepth=16
                )
                depth = depth.reshape(-1, depth.shape[1]).tolist()
                writer.write(fh, depth)

    def export_color_images(self, output_path, image_size=None, frame_skip=1):
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        print(
            "exporting", len(self.frames) // frame_skip, "color frames to", output_path
        )
        for f in range(0, len(self.frames), frame_skip):
            color = self.frames[f].decompress_color(self.color_compression_type)
            if image_size is not None:
                color = cv2.resize(
                    color,
                    (image_size[0], image_size[1]),
                    interpolation=cv2.INTER_AREA,
                )
            formatted_frame_no = format_frame_number_with_leading_zeros(f)
            imageio.imwrite(
                os.path.join(output_path, formatted_frame_no + ".jpg"), color
            )

    def save_mat_to_file(self, matrix, filename):
        with open(filename, "w") as f:
            for line in matrix:
                np.savetxt(f, line[np.newaxis], fmt="%f")

    def export_poses(self, output_path, frame_skip=1):
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        print(
            "exporting", len(self.frames) // frame_skip, "camera poses to", output_path
        )
        for f in range(0, len(self.frames), frame_skip):
            formatted_frame_no = format_frame_number_with_leading_zeros(f)
            self.save_mat_to_file(
                self.frames[f].camera_to_world,
                os.path.join(output_path, formatted_frame_no + ".txt"),
            )

    def export_intrinsics(self, output_path, image_size=None):
        """Export camera intrinsics as .txt file

        If image_size is passed, adjust the camera intrinsics to match the new image size
        """
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        print("exporting camera intrinsics to", output_path)
        if image_size is not None:
            # Adjust intrinsics matrix
            adjusted_intrinsic_color = adjust_intrinsic_matrix(
                self.intrinsic_color,
                (self.color_width, self.color_height),
                (image_size[0], image_size[1]),
            )
            self.save_mat_to_file(
                adjusted_intrinsic_color,
                os.path.join(
                    output_path,
                    "intrinsic_color.txt",
                ),
            )

            adjusted_intrinsic_depth = adjust_intrinsic_matrix(
                self.intrinsic_depth,
                (self.depth_width, self.depth_height),
                (image_size[0], image_size[1]),
            )

            self.save_mat_to_file(
                adjusted_intrinsic_depth,
                os.path.join(output_path, "intrinsic_depth.txt"),
            )

        else:
            self.save_mat_to_file(
                self.intrinsic_color, os.path.join(output_path, "intrinsic_color.txt")
            )
            self.save_mat_to_file(
                self.intrinsic_depth, os.path.join(output_path, "intrinsic_depth.txt")
            )
        self.save_mat_to_file(
            self.extrinsic_color, os.path.join(output_path, "extrinsic_color.txt")
        )
        self.save_mat_to_file(
            self.extrinsic_depth, os.path.join(output_path, "extrinsic_depth.txt")
        )

    def export_imu_timestamps(self, output_file_path):
        field_names = ["FrameID", "TimeStamp"]
        with open(output_file_path, "w") as f:
            writer = csv.DictWriter(f, fieldnames=field_names)
            writer.writeheader()
            for i in range(len(self.imu_frames)):
                writer.writerow(
                    {"FrameID": i, "TimeStamp": self.imu_frames[i].timestamp}
                )
