import os
import csv
import numpy as np
from matplotlib import pyplot as plt

# Params
keys = [
    'MeanGTError [m]', 'StdGTError [m]', 'GTRMSE [m]', 'TotalPoints [1]',
    'UnknownPoints [1]', 'TruncatedPoints [1]', 'GTInliers [1]',
    'MeanMapError [m]', 'StdMapError [m]', 'MapRMSE [m]', 'MapInliers [1]',
    'MapOutliers [1]'
]
output_dir = '/home/lukas/Documents/PanopticMapping/Results/exp2/'
output_name = 'exp2'

# Args
coverage = True
show = True

# Auto Args
if coverage:
    key = 13
else:
    key = 0  # 0 MAD, 13: Coverage
store_output = True
use_percentage = False

# input_path = '/home/lukas/Documents/PanopticMapping/Exp2/'
# input_dirs = [
#     'single_with_map/pan_1', 'single/pan_1',
#     'fusion/pan_1', 'gt/pan_1', 'detectron/pan_1'
# ]
# previous data
input_path = '/home/lukas/Documents/PanopticMapping/Exp2/previous_data/'
input_dirs = [
    'SingleTsdf/run2_with_map', 'SingleTsdf/run2_no_map',
    'longterm_pcl/longterm_pcl', 'GT/prob', 'Detectron/prob'
]
legend = [
    'Monolithic with map', 'Monolithic no map', 'Long-term fusion',
    'Ours (ground truth)', 'Ours (detectron)'
]
labels = keys + ["Coverage [1]", "Coverage [%]"]
styles = ['g-', 'g--', 'k-.', 'b-', 'b--']

# Read data
data = {}  # {dir: {field: values}}
for d in input_dirs:
    with open(os.path.join(input_path, d, 'evaluation_data.csv'),
              'r') as read_obj:
        csv_reader = csv.reader(read_obj)
        fields = []
        values = []
        for row in csv_reader:
            if row[0] == "MeanGTError [m]":
                fields = row
                values = [[] for i in range(len(row))]
            else:
                for i, r in enumerate(row):
                    values[i].append(r)
        datum = {}
        for i, f in enumerate(fields):
            datum[f] = np.array(values[i], dtype=float)
        data[d] = datum

# Plot
plt.rcParams.update({'font.size': 12})
fig, ax = plt.subplots(2)
key = 0
for i, d in enumerate(input_dirs):
    if key < 12:
        y = data[d][keys[key]]
    elif key == 12:
        # Coverage
        y = data[d]['TotalPoints [1]'] - data[d]['UnknownPoints [1]']
    elif key == 13:
        # Coverage with inliers
        y = data[d]['GTInliers [1]'] / data[d]['TotalPoints [1]']
    if use_percentage:
        y = y / 31165.62
        str_list = list(labels[key])
        str_list[-2] = "%"
        labels[key] = "".join(str_list)
    y = y[1:] * 100
    x = (np.arange(len(y)) + 1) * 10
    ax[0].plot(x, y, styles[i])

key = 13
for i, d in enumerate(input_dirs):
    if key < 12:
        y = data[d][keys[key]]
    elif key == 12:
        # Coverage
        y = data[d]['TotalPoints [1]'] - data[d]['UnknownPoints [1]']
    elif key == 13:
        # Coverage with inliers
        y = data[d]['GTInliers [1]'] / data[d]['TotalPoints [1]']
    if use_percentage:
        y = y / 31165.62
        str_list = list(labels[key])
        str_list[-2] = "%"
        labels[key] = "".join(str_list)
    y = y[1:] * 100
    x = (np.arange(len(y)) + 1) * 10
    ax[1].plot(x, y, styles[i])

ax[0].label_outer()
ax[1].set_xlabel("Frame Number")
ax[0].set_ylabel('Average Error [cm]')
ax[1].set_ylabel('Coverage [%]')

# Legend
# ax[1].legend(legend,
#              loc='upper center',
#              bbox_to_anchor=(0.45, -.2),
#              ncol=3,
#              fontsize=11)
ax[1].legend(legend, fontsize=11)
plt.tight_layout()

# Axes
# plt.xlabel("Frame Number")
# plt.ylabel(labels[key])
# if not coverage:
#     plt.ylabel('Average Reconstruction Error [cm]')

# Legend
# if coverage:
#     plt.legend(legend)
# plt.tight_layout()

# Save
plt.gcf().set_size_inches(6, 5, forward=True)
if store_output:
    # output_path = os.path.join(
    #     output_dir, output_name + "_" + labels[key].split(" ", 1)[0])
    # if key in [1, 2, 3] and use_percentage:
    #     output_path += "_perc"
    # output_path += ".jpg"
    plt.savefig(output_dir + "/exp2_both.jpg")
if show:
    plt.show()
