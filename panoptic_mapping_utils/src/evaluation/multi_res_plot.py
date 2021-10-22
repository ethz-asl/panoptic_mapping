import numpy as np
from matplotlib import pyplot as plt
from panoptic_data_reader import read_panoptic_mapper_data_log
import collections
import os

# Params
keys = ['Map Size [MB]', 'Reconstruction Error [cm]', 'Coverage [%]']  # x, y
output_dir = '/home/lukas/Pictures/pan'
output_name = 'multi_res'
store_output = True
plot_error = True

# name: [mean absolute distance [cm], map size [MB], unknown points (of 3201033)
data = {
    'supereight 10cm': [3.899, 8.9, 353298],
    'supereight 5cm': [2.051, 37.4, 517256],
    'supereight 2cm': [0.962, 226.8, 733611],
    'supereight 1cm': [0.657, 899.1, 832092],
    'supereight_dense 10cm': [3.90, 3.6, 353360],
    'supereight_dense 5cm': [2.051, 22.5, 517316],
    'supereight_dense 2cm': [0.961, 136.3, 734710],
    'supereight_dense 1cm': [0.663, 540.2, 847184],
    'voxblox 20cm': [7.076, 0.8, 371626],
    'voxblox 10cm': [3.051, 3.7, 578613],
    'voxblox 5cm': [1.505, 21.0, 744807],
    'voxblox 2cm': [0.753, 236.3, 933362],
    'voxblox_uni 20cm': [10.664, 0.8, 336338],
    'voxblox_uni 10cm': [4.395, 3.7, 546224],
    'voxblox_uni 5cm': [2.098, 21.1, 716764],
    'voxblox_uni 2cm': [0.903, 237.2, 902464],
    'panmap single_tsdf 20cm': [6.194, 0.8, 665660],
    'panmap single_tsdf 10cm': [2.792, 5.0, 707937],
    'panmap single_tsdf 5cm': [1.417, 29.2, 888050],
    # 'panmap single_tsdf 2cm': [1.056, 77.7]
    # 'panmap multi w/ GT': [0, 0],
    # 'panmap multi w/ Detectron': [0, 0],
}

# Plot
keylist = sorted(data.keys())
styles = {
    'panmap single_tsdf ': 'xb',
    'supereight ': 'ok',
    'supereight_dense ': 'xk',
    'voxblox ': 'og',
    'voxblox_uni ': 'xg'
}  # , 'panmap multi ': 'ob'}
styles = collections.OrderedDict(sorted(styles.items()))
for k in keylist:
    for s in styles.keys():
        if k.startswith(s):
            style = styles[s]
            text = k[len(s):]
            break
    x = data[k][1]
    y = data[k][0]
    if not plot_error:
        y = (3201033.0 - data[k][2]) / 32010.33
    plt.scatter(x, y, c=style[1], marker=style[0])
    plt.annotate(text, (x, y))

# Axes
plt.semilogx()
plt.xlabel(keys[0])
if plot_error:
    plt.ylabel(keys[1])
else:
    plt.ylabel(keys[2])
    plt.ylim((60, 100))

# Legend
artists = []
labels = []
for s in styles.keys():
    artists.append(plt.scatter(0, 0, c=styles[s][1], marker=styles[s][0]))
    labels.append(s)
plt.legend(artists, labels)

# Save
if plot_error:
    output_name += "_error"
else:
    output_name += "_coverage"
if store_output:
    plt.savefig(os.path.join(output_dir, output_name))
plt.show()
