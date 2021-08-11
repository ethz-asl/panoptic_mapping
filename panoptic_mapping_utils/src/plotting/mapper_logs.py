import os
from matplotlib import pyplot as plt
from panoptic_data_reader import read_panoptic_mapper_data_log

# Params
dirs = [
    '/home/lukas/Documents/PanopticMapping/Data/saved_logs/run1_gt.csv',
    '/home/lukas/Documents/PanopticMapping/Data/saved_logs/'
    'run1_detectron_int.csv'
]
names = ["Ground Truth Associations", "Detectron"]
keys = ['Timestamp [s]', 'NoSubmaps [1]']  # x, y
output_dir = '/home/lukas/Pictures/pan'
output_name = 'number_of_submaps_int'
store_output = True

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

if store_output:
    plt.savefig(os.path.join(output_dir, output_name))
plt.show()
