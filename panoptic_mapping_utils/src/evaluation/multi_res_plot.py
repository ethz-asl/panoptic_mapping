import numpy as np
from matplotlib import pyplot as plt
from panoptic_data_reader import read_panoptic_mapper_data_log
import os

# Params
keys = ['Map Size [MB]', 'Reconstruction Error [cm]']  # x, y
output_dir = '/home/lukas/Pictures/pan'
output_name = 'multi_res'
store_output = True

# name: [mean absolute distance [cm], map size [MB], unknown points (of 3201033)
data = {
    'supereight 10cm': [19.48, 8.9],
    'supereight 5cm': [20.51, 37.4],
    'supereight 2cm': [1.409, 190.6],
    'supereight 1cm': [1.058, 1100.6],
    'supereight_dense 10cm': [17.6, 3.6],
    'supereight_dense 5cm': [18.25, 22.5],
    'supereight_dense 2cm': [1.409, 190.6],
    'supereight_dense 1cm': [1.058, 1100.6],
    'panmap single_tsdf 20cm': [2.332, 0.6],
    'panmap single_tsdf 10cm': [1.859, 4.5],
    'panmap single_tsdf 5cm': [1.409, 19.7],
    'panmap single_tsdf 3cm': [1.056, 77.7],
    'voxblox 20cm': [7.076, 0.8, 371626],
    'voxblox 10cm': [3.051, 3.7, 578613],
    'voxblox 5cm': [1.505, 21.0, 744807],
    'voxblox 2cm': [0.753, 236.3, 933362],
    'voxblox_uni 20cm': [3.114, 0.8],
    'voxblox_uni 10cm': [2.284, 3.7],
    'voxblox_uni 5cm': [1.832, 21.1],
    'voxblox_uni 2cm': [1.205, 237.2],
    # 'panmap multi w/ GT': [0, 0],
    # 'panmap multi w/ Detectron': [0, 0],
}

# Plot
keylist = sorted(data.keys())
for k in keylist:
    if k.startswith('supereight'):
        style = 'ok'
    elif k.startswith('panmap single_tsdf'):
        style = 'ob'
    elif k.startswith('voxblox'):
        style = 'og'
    elif k.startswith('panmap multi'):
        style = 'ob'
    plt.scatter(data[k][1], data[k][0], c=style[1], marker=style[0])

plt.semilogx()
plt.xlabel(keys[0])
plt.ylabel(keys[1])
plt.legend(keylist)

# Print

if store_output:
    plt.savefig(os.path.join(output_dir, output_name))
plt.show()
