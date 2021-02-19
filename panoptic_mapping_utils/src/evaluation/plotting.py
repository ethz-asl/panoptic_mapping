import numpy as np
from matplotlib import pyplot as plt
from panoptic_data_reader import read_panoptic_mapper_data_log
import os

# Params
t1 = '/home/lukas/Documents/PanopticMapping/Data/saved_runs/run1_gt.csv'
t2 = '/home/lukas/Documents/PanopticMapping/Data/saved_runs/run1_gtseg_track.csv'
t3 = '/home/lukas/Documents/PanopticMapping/Data/saved_runs/run1_gtseg_track_noclear.csv'

keys = ['Timestamp [s]', ' NoSubmaps [1]']
key = keys[1]
output_dir = '/home/lukas/Pictures/pan'
output_name = 'number_of_submaps'
store_output = True

# Plot

data = read_panoptic_mapper_data_log(t1)
x = data['Timestamp [s]']
x = x - x[0]
y = data[key]
plt.plot(x, y)

data = read_panoptic_mapper_data_log(t2)
x = data['Timestamp [s]']
x = x - x[0]
y = data[key]
plt.plot(x, y)

data = read_panoptic_mapper_data_log(t3)
x = data['Timestamp [s]']
x = x - x[0]
y = data[key]
plt.plot(x, y)

plt.xlabel("Time [s]")
plt.ylabel("Number of submaps [1]")
plt.legend(["Ground Truth Associations", "Tracked", "Tracked no clearing"])

if store_output:
    plt.savefig(os.path.join(output_dir, output_name))
plt.show()
