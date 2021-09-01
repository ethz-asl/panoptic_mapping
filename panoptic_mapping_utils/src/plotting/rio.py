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


def plot(coverage, show):
    if coverage:
        key = 12
        use_percentage = key == 12
    else:
        key = 7  # 7: Map MAD, 12: Coverage
        use_percentage = False
    store_output = True

    input_path = '/home/lukas/Documents/PanopticMapping/Rio/scene_2'
    input_dirs = [
        'single_with_map/pan_1', 'single/pan_1', 'fusion/pan_1', 'gt/pan_1',
        'detectron/pan_1'
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
            data[d] = datum

    # Plot
    plt.rcParams.update({'font.size': 12})
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
        plt.plot(x, y, styles[i])

    # Axes
    plt.xlabel("Frame Number")
    plt.ylabel(labels[key])
    if not coverage:
        plt.ylabel('Average Reconstruction Error [cm]')

    # Legend
    if coverage:
        plt.legend(legend)
    plt.tight_layout()

    # Save
    plt.gcf().set_size_inches(7, 3.5, forward=True)
    if store_output:
        output_path = os.path.join(
            output_dir, output_name + "_" + labels[key].split(" ", 1)[0])
        if key in [1, 2, 3] and use_percentage:
            output_path += "_perc"
        output_path += ".jpg"
        plt.savefig(output_path)
    if show:
        plt.show()


if __name__ == "__main__":
    plot(False, True)