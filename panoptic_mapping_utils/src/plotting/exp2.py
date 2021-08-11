import os
import csv
import numpy as np
from matplotlib import pyplot as plt

# Params
keys = [
    'MeanError [m]', 'StdError [m]', 'RMSE [m]', 'TotalPoints [1]',
    'UnknownPoints [1]', 'TruncatedPoints [1]'
]
output_dir = '/home/lukas/Documents/PanopticMapping/Results/exp2/'
output_name = 'exp2'

key = 2  # 0: MeanError, 1: Coverage, 2: Correct Points, 3: Incorrect Points
store_output = True
use_percentage = False

input_path = '/home/lukas/Documents/PanopticMapping/Exp2/SingleTsdf'
input_dirs = ['run2_with_map', 'run2_no_map']
legend = input_dirs
labels = [
    'MeanError [m]', 'Coverage [1]', 'Correct Points [1]',
    'Incorrect Points [1]'
]
styles = ['b-', 'g--', 'k-.']

# Read data
data = {}  # {dir: {field: values}}
for d in input_dirs:
    with open(os.path.join(input_path, d, 'evaluation_data.csv'),
              'r') as read_obj:
        csv_reader = csv.reader(read_obj)
        fields = []
        values = []
        for row in csv_reader:
            if row[0] == "MeanError [m]":
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
x = np.arange(len(values[0])) * 10
for i, d in enumerate(input_dirs):
    y = data[d][keys[key]]
    if key == 1:
        # Coverage
        y = 3116562 - data[d]['UnknownPoints [1]']
    elif key == 2:
        # Correct Points
        y = 3116562 - (data[d]['UnknownPoints [1]'] +
                       data[d]['TruncatedPoints [1]'])
    elif key == 3:
        # Incorrect Points
        y = data[d]['TruncatedPoints [1]']
    if key in [1, 2, 3] and use_percentage:
        y = y / 31165.62
        str_list = list(labels[key])
        str_list[-2] = "%"
        labels[key] = "".join(str_list)

    plt.plot(x, y, styles[i])

# Axes
plt.xlabel("Frame Number")
plt.ylabel(labels[key])

# Legend
plt.legend(legend)

# Save
if store_output:
    output_path = os.path.join(
        output_dir, output_name + "_" + labels[key].split(" ", 1)[0])
    if key in [1, 2, 3] and use_percentage:
        output_path += "_perc"
    output_path += ".jpg"
    plt.savefig(output_path)
plt.show()
