import glob
import os
import json
import numpy as np
import pandas as pd
from copy import deepcopy

# Huge thanks to https://stackoverflow.com/a/49677241/13399661
class NumpyEncoder(json.JSONEncoder):
    """ Special json encoder for numpy types """
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

def list_string_to_matrix(data: list):
    # if the last element is empty string, remove it
    if data[-1] == '':
        data = data[:-1]
    # the matrix's elements are separated by space
    data = [i.split(' ') for i in data]
    # remove unnecessary empty strings
    data = [[j for j in i if j != ''] for i in data]
    # convert the strings to floats
    data = [[float(j) for j in i] for i in data]
    # convert this to numpy array
    data = np.array(data)
    return data

def list_string_to_matrix_v2(data: str):
    # In this version the data is a str, each row is separated by a newline character
    # each element in a row is separated by a space
    data = data.split('\n')
    # if the last element is empty string, remove it
    if data[-1] == '':
        data = data[:-1]
    # the matrix's elements are separated by space
    data = [i.split(' ') for i in data]
    # remove unnecessary empty strings
    data = [[j for j in i if j != ''] for i in data]
    # convert the strings to floats
    data = [[float(j) for j in i] for i in data]
    # sometimes here an element is an empty list, remove it
    data = [i for i in data if i != []]
    # convert this to numpy array
    data = np.array(data)
    return data

def analyse(json_file: str):
    data = json.load(open(json_file, 'r'))
    # first get the data structure by reading one iteration
    # copy the data to a new dictionary
    del data['integer_bits_standard']
    del data['fractional_bits_standard']
    del data['integer_bits_gravity']
    del data['fractional_bits_gravity']
    del data['integer_bits_fd']
    del data['fractional_bits_fd']
    # remove the small_resolution keys
    for key1 in data.keys():
        # key1 is the iteration number
        for key2 in data[key1].keys():
            # key2 is the joint index
            for key3 in data[key1][key2].keys():
                data[key1][key2][key3] = list_string_to_matrix_v2(data[key1][key2][key3]['big_resolution'])

    data_template = data['0'].copy()
    
    # now we can analyse the data
    # sum all values over iterations to the template
    for key1 in data.keys():
        for key2 in data[key1].keys():
            for key3 in data[key1][key2].keys():
                data_template[key2][key3] += data[key1][key2][key3]
    
    # calculate the mean
    for key1 in data_template.keys():
        for key2 in data_template[key1].keys():
            data_template[key1][key2] /= len(data.keys())

    # save means
    os.makedirs('experiment_means', exist_ok=True)
    with open('experiment_means/means.json', 'w') as f:
        json.dump(data_template, f, indent=4, cls=NumpyEncoder)
    print('The mean data is saved to experiment_means/means.json')

    range_template = deepcopy(data['0'].copy())
    # calculate the range
    for key1 in data.keys():
        for key2 in data[key1].keys():
            for key3 in data[key1][key2].keys():
                range_template[key2][key3] = {
                    "min": np.Inf,
                    "max": np.NINF
                }

    for key1 in data.keys():
        for key2 in data[key1].keys():
            for key3 in data[key1][key2].keys():
                try:
                    range_template[key2][key3]['min'] = np.minimum(range_template[key2][key3]['min'], np.min(data[key1][key2][key3]))
                    range_template[key2][key3]['max'] = np.maximum(range_template[key2][key3]['max'], np.max(data[key1][key2][key3]))
                except:
                    print("Error in key1: ", key1, " key2: ", key2, " key3: ", key3)
                    print("Data: ", data[key1][key2][key3])
                    print("Range: ", range_template[key2][key3])

    # save the data
    os.makedirs('experiment_ranges', exist_ok=True)
    with open('experiment_ranges/ranges.json', 'w') as f:
        json.dump(range_template, f, indent=4, cls=NumpyEncoder)
    print('The range data is saved to experiment_ranges/ranges.json')
    




if __name__ == "__main__":
    summary_files = glob.glob('experiment_summaries/*')
    # I assume that there will be only one summary file as I'll only analyse the ranges for double precision
    if len(summary_files) != 1:
        raise ValueError('There should be only one summary file')
    
    analyse(summary_files[0])
    










