import os
import csv
import numpy as np
from matplotlib import pyplot as plt

# Params
keys = [
    'MeanGTError [m]', 'StdGTError [m]', 'GTRMSE [m]', 'TotalPoints [1]',
    'UnknownPoints [1]', 'TruncatedPoints [1]', 'GTInliers [1]',
    'MeanMapError [m]', 'StdMapError [m]', 'MapRMSE[m]', 'MapInliers [1]',
    'MapOutliers [1]'
]
output_dir = '/home/lukas/Documents/PanopticMapping/Results/rio/'
output_name = 'rio'
store_output = True

setting = 1  # 0: Local eval (exp2), 1: Global eval
scene_id = 2  # 0: 4 days, (1), 2: 2 days

# 0 MAD, 6: GTInliers, 12: Coverage (observed), 13: Coverage (inliers)
if setting == 0:
    key = [0, 13]  # 7-12 (2), 0-13 (1)
    datasets = [1, 1]  # 1: local, 2: complete
    use_percentage = [False, False]
else:
    key = [7, 12]
    datasets = [2, 2]
    use_percentage = [False, False]

input_path = '/home/lukas/Documents/PanopticMapping/Rio/'
no_runs = [4, 4, 2][scene_id]  # 0:4, 1:4, 2:2
run_range = range(0, no_runs)
input_dirs = [[
    "scene_%i/single_with_map/pan_%i" % (scene_id, i) for i in run_range
], ["scene_%i/single/pan_%i" % (scene_id, i) for i in run_range],
              ["scene_%i/fusion/pan_%i" % (scene_id, i) for i in run_range],
              ["scene_%i/gt/pan_%i" % (scene_id, i) for i in run_range],
              ["scene_%i/detectron/pan_%i" % (scene_id, i) for i in run_range]]
legend = [
    'Monolithic with map', 'Monolithic no map', 'Long-term fusion',
    'Ours (ground truth)', 'Ours (detectron)'
]
labels = keys + ["Coverage [1]", "Coverage [%]"]
styles = ['g-', 'g--', 'k-.', 'b-', 'b--']

# Read data
read_data_dirs = {}
for d in input_dirs:
    for d2 in d:
        read_data_dirs[d2] = d2

data_1 = {}  # {dir: {field: values}}
for d in read_data_dirs:
    with open(os.path.join(input_path, d, 'eval_local.csv'), 'r') as read_obj:
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
        data_1[d] = datum

data_2 = {}  # {dir: {field: values}}
for d in read_data_dirs:
    with open(os.path.join(input_path, d, 'eval_complete.csv'),
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
        data_2[d] = datum

# Plot
plt.rcParams.update({'font.size': 11})
fig, ax = plt.subplots(2)
data = data_1
if datasets[0] == 2:
    data = data_2
for i, d in enumerate(input_dirs):
    y = []
    for d2 in d:
        if key[0] < 12:
            [y.append(x) for x in data[d2][keys[key[0]]]]
        elif key[0] == 12:
            # Coverage
            y += [(data[d2]['TotalPoints [1]'][j] -
                   data[d2]['UnknownPoints [1]'][j]) /
                  data[d2]['TotalPoints [1]'][j]
                  for j in range(len(data[d2]['TotalPoints [1]']))]
        elif key[0] == 13:
            # Coverage with inliers
            y += [
                data[d2]['GTInliers [1]'][j] / data[d2]['TotalPoints [1]'][j]
                for j in range(len(data[d2]['TotalPoints [1]']))
            ]
        if use_percentage[0]:
            y = [y2 / data[d2]['TotalPoints [1]'][0] * 100.0 for y2 in y]
            str_list = list(labels[key[0]])
            str_list[-2] = "%"
            labels[key[0]] = "".join(str_list)
    y = np.array(y) * 100.0
    x = (np.arange(len(y)) + 1) * 10
    ax[0].plot(x, y, styles[i])

data = data_1
if datasets[1] == 2:
    data = data_2
for i, d in enumerate(input_dirs):
    y = []
    for d2 in d:
        if key[1] < 12:
            [y.append(x) for x in data[d2][keys[key[1]]]]
        elif key[1] == 12:
            # Coverage
            y += [(data[d2]['TotalPoints [1]'][j] -
                   data[d2]['UnknownPoints [1]'][j]) /
                  data[d2]['TotalPoints [1]'][j]
                  for j in range(len(data[d2]['TotalPoints [1]']))]
        elif key[1] == 13:
            # Coverage with inliers
            y += [
                data[d2]['GTInliers [1]'][j] / data[d2]['TotalPoints [1]'][j]
                for j in range(len(data[d2]['TotalPoints [1]']))
            ]
        if use_percentage[1]:
            y = [y2 / data[d2]['TotalPoints [1]'][0] * 100.0 for y2 in y]
            str_list = list(labels[key[1]])
            str_list[-2] = "%"
            labels[key[1]] = "".join(str_list)
    y = np.array(y) * 100.0
    x = (np.arange(len(y)) + 1) * 10
    ax[1].plot(x, y, styles[i])

# Day intersections
days = []
for d2 in input_dirs[0]:
    days.append(len(data[d2][keys[0]]) * 10)
    if len(days) > 1:
        days[-1] = days[-1] + days[-2]

for x in days[:-1]:
    ax[0].axvline(x + 10, color='k', ls='--', lw=1)
    ax[1].axvline(x + 10, color='k', ls='--', lw=1)

ax[0].label_outer()
ax[1].set_xlabel("Frame Number")
ax[0].set_ylabel('Average Error [cm]')
ax[1].set_ylabel('Coverage [%]')

# Legend
# ax[1].legend(legend,
#  loc='upper center',
#  bbox_to_anchor=(0.45, -.2),
#  ncol=3,
#  fontsize=11)
plt.tight_layout()

# Save
plt.gcf().set_size_inches(7, 3.5, forward=True)
if store_output:
    suffix = "loc"
    if setting == 1:
        suffix = "glob"
    output_path = os.path.join(output_dir,
                               output_name + "_%i_%s.jpg" % (scene_id, suffix))
    plt.savefig(output_path)
plt.show()
