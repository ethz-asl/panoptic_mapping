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
    'supereight 256': [10.161, 7.2],
    'supereight 512': [3.493, 34.7],
    'supereight 1024': [2.774, 190.6],
    'supereight 2048': [0, 1100.6],
    'voxblox 20cm': [],
    'voxblox 10cm': [],
    'voxblox 5cm': [],
    'voxblox 2cm': [],
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
