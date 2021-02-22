import numpy as np
import csv
import os
import warnings


def read_panoptic_mapper_data_log(file_name):
    """ A stored data log of the panoptic mapper and converts it into a
    dictionary of numpy arrays if numeric or lists of strings otherwise. """
    result = {}
    if not os.path.isfile(file_name):
        warnings.warn("Target file '%s' does not exist." % file_name)
        return result

    with open(file_name, newline='') as f:
        reader = csv.reader(f)
        data = list(reader)
        for col in range(len(data[0])):
            header = data[0][col]
            values = []
            for row in range(1, len(data)):
                values.append(data[row][col])
            try:
                result[header] = np.array(values, dtype=float)
            except:
                result[header] = values
    return result
