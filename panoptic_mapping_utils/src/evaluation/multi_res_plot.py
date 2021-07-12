import numpy as np
from matplotlib import pyplot as plt
from panoptic_data_reader import read_panoptic_mapper_data_log
import os

# Params
keys = ['Reconstruction Error [cm]', 'Map Size [MB]']  # x, y
output_dir = '/home/lukas/Pictures/pan'
output_name = 'multi_res'
store_output = True

# name: [mean absolute distance [cm], map size [MB]
# Note: supereight no color integration, no semantic integration.
# supereight resolutions are roughly ~8,4,2,1 cm
data = {
    'supereight 256': [3.228, 7.2],  # 5.872 (without cropping)
    'supereight 512': [2.119, 34.7],  # 3.559
    'supereight 1024': [1.409, 190.6],  # 2.870
    'supereight 2048': [1.058, 1100.6],  # 2.546
    'panmap single_tsdf 20cm': [2.332, 0.6],
    'panmap single_tsdf 10cm': [1.859, 4.5],
    'panmap single_tsdf 5cm': [1.409, 19.7],
    'panmap single_tsdf 3cm': [1.056, 77.7],
    'voxblox 20cm': [3.114, 1.5],
    'voxblox 10cm': [2.284, 7.0],
    'voxblox 5cm': [1.832, 39.5],
    'voxblox 2cm': [1.205, 247.8],
    'panmap GT': [],
    'panmap Detectron': [],
}

# Plot
for d in dirs:
    data = read_panoptic_mapper_data_log(d)
    x = data[keys[0]]
    x = x - x[0]
    y = data[keys[1]]
    plt.plot(x, y)

plt.xlabel(keys[0])
plt.ylabel(keys[1])
plt.legend(names)

# Print

if store_output:
    plt.savefig(os.path.join(output_dir, output_name))
plt.show()
