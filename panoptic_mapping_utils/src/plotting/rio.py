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
output_dir = '/home/lukas/Documents/PanopticMapping/Results/rio/'
output_name = 'rio_comp'

# Args
scene_id = 0
run_id = 3  # 0:3 2:1
coverage = True
is_local = True
show = True
store_output = True

# Auto Args
input_path = '/home/lukas/Documents/PanopticMapping/Rio/scene_%i' % scene_id
if coverage:
    if is_local:
        key = 13  # 7: Map MAD, 12: Observed Coverage, 0 GT MAD, 13 Inlier Coverage
        use_percentage = False
        local = 0  # 0: local, 1: global
    else:
        key = 12
        use_percentage = True
        local = 1
else:
    if is_local:
        key = 0
        local = 0  # 0: local, 1: global
    else:
        key = 7
        local = 1
    use_percentage = False

input_dirs = [
    'single_with_map/pan_', 'single/pan_', 'fusion/pan_', 'gt/pan_',
    'detectron/pan_'
]
input_dirs = [x + '%i' % run_id for x in input_dirs]
legend = [
    'Monolithic with map', 'Monolithic no map', 'Long-term fusion',
    'Ours (ground truth)', 'Ours (detectron)'
]
labels = keys + ["Coverage [1]", "Coverage [%]"]
styles = ['g-', 'g--', 'k-.', 'b-', 'b--']
eval_files = ['eval_local.csv', 'eval_complete.csv']

# Read data
data = {}  # {dir: {field: values}}
for d in input_dirs:
    with open(os.path.join(input_path, d, eval_files[local]), 'r') as read_obj:
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
        y = y / data[d]['TotalPoints [1]'][0]
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
        y = y / data[d]['TotalPoints [1]'][0]
        str_list = list(labels[key])
        str_list[-2] = "%"
        labels[key] = "".join(str_list)
    y = y[1:] * 100
    x = (np.arange(len(y)) + 1) * 10
    ax[1].plot(x, y, styles[i])

# Axes
# plt.xlabel("Frame Number")
# plt.ylabel(labels[key])
# if not coverage:
#     plt.ylabel('Average Reconstruction Error [cm]')

# # Legend
# if coverage:
#     plt.legend(legend)
# plt.tight_layout()
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

# Save
plt.gcf().set_size_inches(6, 5, forward=True)
if store_output:
    # output_path = os.path.join(
    #     output_dir, output_name + "_" + labels[key].split(" ", 1)[0])
    # if is_local:
    #     output_path += "_loc"
    # else:
    #     output_path += "_glob"

    # output_path += ".jpg"
    plt.savefig(output_dir + "/rio_local_both_scene%s.jpg" % scene_id)
if show:
    plt.show()
