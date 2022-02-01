"""
The source code in this file is based on:
https://github.com/ScanNet/ScanNet/blob/master/SensReader/python/reader.py

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


import argparse
import os, sys

from sensor_data import SensorData

# params
parser = argparse.ArgumentParser()
# data paths
parser.add_argument(
    "--filename",
    required=True,
    help="path to sens file to read",
)
parser.add_argument(
    "--output_path",
    help="path to output folder",
)
parser.add_argument(
    "--export_depth",
    dest="export_depth",
    action="store_true",
)
parser.add_argument(
    "--export_color",
    dest="export_color",
    action="store_true",
)
parser.add_argument(
    "--export_pose",
    dest="export_pose",
    action="store_true",
)
parser.add_argument(
    "--export_intrinsic",
    dest="export_intrinsic",
    action="store_true",
)
parser.add_argument(
    "--export_timestamps",
    dest="export_timestamps",
    action="store_true",
)
parser.add_argument(
    "--image_size",
    dest="image_size",
    type=int,
    nargs=2,
    help="Size of the exported color images as WIDTH HEIGHT",
)
parser.set_defaults(
    output_path=None,
    export_depth=False,
    export_color=False,
    export_pose=False,
    export_intrinsic=False,
    export_timestamps=False,
    image_size=None,
)

opt = parser.parse_args()
print(opt)


def main():
    # If not specified use the same directory as the .sens file
    if opt.output_path is None:
        opt.output_path = os.path.dirname(opt.filename)
    os.makedirs(opt.output_path, exist_ok=True)
    # load the data
    sys.stdout.write("loading %s..." % opt.filename)
    sd = SensorData(opt.filename)
    sys.stdout.write("loaded!\n")
    if opt.export_color:
        sd.export_color_images(
            os.path.join(opt.output_path, "color"),
            image_size=opt.image_size,
        )
    if opt.export_depth:
        sd.export_depth_images(os.path.join(opt.output_path, "depth"), opt.image_size)
    if opt.export_pose:
        sd.export_poses(os.path.join(opt.output_path, "pose"))
    if opt.export_intrinsic:
        sd.export_intrinsics(os.path.join(opt.output_path, "intrinsic"), opt.image_size)
    if opt.export_timestamps:
        sd.export_imu_timestamps(os.path.join(opt.output_path, "timestamps.csv"))


if __name__ == "__main__":
    main()
