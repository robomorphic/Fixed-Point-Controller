import glob
import os
import json
import numpy as np
import pandas as pd
from copy import deepcopy
from matplotlib import pyplot as plt

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

    # remove "fixed_point_Minv_output" and original_data_Minv_output keys from each element if it exists
    for key1 in data.keys():
        # does "fixed_point_Minv_output" exist?
        if "fixed_point_Minv_output" in data[key1]:
            del data[key1]["fixed_point_Minv_output"]
        if "original_data_Minv_output" in data[key1]:
            del data[key1]["original_data_Minv_output"]


    # remove the small_resolution keys
    for key1 in data.keys():
        # key1 is the iteration number
        for key2 in data[key1].keys():
            # key2 is the joint index
            for key3 in data[key1][key2].keys():
                data[key1][key2][key3] = list_string_to_matrix_v2(data[key1][key2][key3]['big_resolution'])

    # for some reason first key "0" can give incorrect output, so I'll remove it
    del data['0']

    data_template = data['1'].copy()

    # After here every key and data is a mess
    # I'll convert this to a dataframe to make it easier to analyse
    data_df = pd.DataFrame(columns=['iteration', 'joint', 'variable', 'value'])
    # For an efficient dataframe, I'll create a list of dictionaries
    data_list = []
    for key1 in data.keys():
        for key2 in data[key1].keys():
            for key3 in data[key1][key2].keys():
                data_list.append({
                    'iteration': int(key1),
                    'joint': int(key2),
                    'variable': key3,
                    'value': data[key1][key2][key3]
                })
    data_df = pd.DataFrame(data_list)


    # save it
    os.makedirs('experiment_dataframes', exist_ok=True)
    data_df.to_csv('experiment_dataframes/data.csv', index=False)
    print('The dataframe is saved to experiment_dataframes/data.csv')

    # we need to calculate the means of each key
    # So, firstly collect the unique list of "variable" column values
    variables = data_df['variable'].unique()
    # Then, collect unique variables of "joint_id"
    joints = data_df['joint'].unique()

    for joint in joints:
        for variable in variables:
            # get the data for this joint and variable
            data_temp = data_df[(data_df['joint'] == joint) & (data_df['variable'] == variable)]
            # calculate the mean
            mean = np.mean(data_temp['value'].values)
            # save it to data_template
            data_template[str(joint)][variable] = mean

    # save means
    os.makedirs('experiment_means', exist_ok=True)
    with open('experiment_means/means.json', 'w') as f:
        json.dump(data_template, f, indent=4, cls=NumpyEncoder)
    print('The mean data is saved to experiment_means/means.json')

    # Now the problem is sometimes I may want to debug or see the data myself
    # But it is a bit hard
    # So, For every joint and every variable, I'll save the data to a separate file
    for joint in joints:
        for variable in variables:
            data_temp = data_df[(data_df['joint'] == joint) & (data_df['variable'] == variable)]
            # save it
            os.makedirs('experiment_debug_data', exist_ok=True)
            # if there are illegal characters in the variable name, replace them
            variable = variable.replace('/', '_')
            variable = variable.replace(' ', '')
            # also while printing numpy arrays to csv, sometimes only the first element is printed
            # so, I'll convert the numpy array to a list
            data_temp['value'] = data_temp['value'].apply(lambda x: x.tolist())
            data_temp.to_csv(f'experiment_debug_data/joint_{joint}_variable_{variable}.csv', index=False)
            # Just for some debugging, print the sum of the values for this joint and variable
            #print(f'Joint {joint}, Variable {variable}, Sum: {np.sum(data_temp["value"].values)}')

    range_template = deepcopy(data['1'].copy())
    # calculate the range
    for key1 in data.keys():
        for key2 in data[key1].keys():
            for key3 in data[key1][key2].keys():
                range_template[key2][key3] = {
                    "min": np.Inf,
                    "max": np.NINF
                }
    
    for joint in joints:
        for variable in variables:
            data_temp = data_df[(data_df['joint'] == joint) & (data_df['variable'] == variable)]
            # calculate the range
            # iterate over every row            
            for index, row in data_temp.iterrows():
                min_value = np.min(row['value'])
                max_value = np.max(row['value'])
                range_template[str(joint)][variable]['min'] = min(range_template[str(joint)][variable]['min'], min_value)
                range_template[str(joint)][variable]['max'] = max(range_template[str(joint)][variable]['max'], max_value)
            

    # save the data
    os.makedirs('experiment_ranges', exist_ok=True)
    with open('experiment_ranges/ranges.json', 'w') as f:
        json.dump(range_template, f, indent=4, cls=NumpyEncoder)
    print('The range data is saved to experiment_ranges/ranges.json')
    
    """
    # now get the data for each variable, and plot the distribution over the variable's values for each iteration
    summed_data = {}
    for key1 in data.keys():
        for key2 in data[key1].keys():
            for key3 in data[key1][key2].keys():
                if key3 not in summed_data:
                    summed_data[key3] = []
                #summed_data[key3].append(data[key1][key2][key3])
                # some data here are matrices or vectors, we need to flatten them
                # and add each element to the list as a separate element
                summed_data[key3].extend(data[key1][key2][key3].flatten())


    # plot the data
    os.makedirs('experiment_plots', exist_ok=True)
    plot_number = 0
    for key in summed_data.keys():
        summed_data[key] = np.array(summed_data[key])
        # plot the data as a histogram, representing the number of occurences of each value
        # make the histogram's scale logarithmic
        plt.hist(summed_data[key], bins=100, log=True)
        plt.title(key)
        plt.xlabel('Value')
        plt.ylabel('Number of occurences')
        plt.savefig(f'experiment_plots/{plot_number}.png')
        plt.close()
        plot_number += 1

    # now collect data only for jdata.Dinv()_Minv_element matrix, and plot the distribution of every element independently
    Dinv_Minv_elements = []
    # initialize
    for i in range(6):
        Dinv_Minv_elements.append([])

    # key2 will be the index of Dinv_Minv_element
    for key1 in data.keys():
        for key2 in data[key1].keys():
            Dinv_Minv_elements[int(key2) - 1].append(data[key1][key2]['jdata.Dinv()_Minv_element'].flatten())
    
    #print('Dinv_Minv_elements: ', Dinv_Minv_elements)
    # plot the data
    for i in range(6):
        Dinv_Minv_elements[i] = np.array(Dinv_Minv_elements[i])
        plt.hist(Dinv_Minv_elements[i], bins=100, log=True)
        plt.title(f'Dinv_Minv_element_{i}')
        plt.xlabel('Value')
        plt.ylabel('Number of occurences')
        plt.savefig(f'experiment_plots/Minv__{plot_number}.png')
        plt.close()
        plot_number += 1    

    print('All plots are saved to experiment_plots folder')
    """

    # load data again because the small_resolution keys were removed
    data = json.load(open(json_file, 'r'))
    # first get the data structure by reading one iteration
    # copy the data to a new dictionary
    del data['integer_bits_standard']
    del data['fractional_bits_standard']
    del data['integer_bits_gravity']
    del data['fractional_bits_gravity']
    del data['integer_bits_fd']
    del data['fractional_bits_fd']

    # remove "fixed_point_Minv_output" and original_data_Minv_output keys from each element if it exists
    for key1 in data.keys():
        # does "fixed_point_Minv_output" exist?
        if "fixed_point_Minv_output" in data[key1]:
            del data[key1]["fixed_point_Minv_output"]
        if "original_data_Minv_output" in data[key1]:
            del data[key1]["original_data_Minv_output"]

    for key1 in data.keys():
        # key1 is the iteration number
        for key2 in data[key1].keys():
            # key2 is the joint index
            for key3 in data[key1][key2].keys():
                for key4 in data[key1][key2][key3].keys():
                    data[key1][key2][key3][key4] = list_string_to_matrix_v2(data[key1][key2][key3][key4])

    # Convert the data to a dataframe
    data_df = pd.DataFrame(columns=['iteration', 'joint', 'variable', 'value'])
    # For an efficient dataframe, I'll create a list of dictionaries
    data_list = []
    for key1 in data.keys():
        for key2 in data[key1].keys():
            for key3 in data[key1][key2].keys():
                data_list.append({
                    'iteration': int(key1),
                    'joint': int(key2),
                    'variable': key3,
                    'value': data[key1][key2][key3]
                })
    data_df = pd.DataFrame(data_list)

    # now, I'll do a different thing. 
    # Read every element in every joint, if it is a matrix or vector, just flatten it
    # Then, calculate the difference between small_resolution and big_resolution in ratios
    # Then, store these ratios in a similar dictionary structure
    # Then, firstly print the mean, min, max of these ratios, and print it
    # Then, plot the histogram of these ratios

    # I'll use the data_template to get the structure
    ratios_template = deepcopy(data_template)
    for key1 in data.keys():
        for key2 in data[key1].keys():
            for key3 in data[key1][key2].keys():
                ratios_template[key2][key3] = []

    for joint in joints:
        for variable in variables:
            data_temp = data_df[(data_df['joint'] == joint) & (data_df['variable'] == variable)]
            # calculate the ratio
            for index, row in data_temp.iterrows():
                ratio_temp = (row['value']['small_resolution'] - row['value']['big_resolution']) / row['value']['big_resolution']
                # if zero division occurred, append nothing
                if np.isnan(ratio_temp).any():
                    # remove nan values
                    ratio_temp = ratio_temp[~np.isnan(ratio_temp)]
                # remove inf values
                ratio_temp = ratio_temp[~np.isinf(ratio_temp)]
                ratios_template[str(joint)][variable].extend(ratio_temp.flatten())

    
    # save ratios_template for debugging
    os.makedirs('experiment_ratios', exist_ok=True)
    with open('experiment_ratios/ratios_template.json', 'w') as f:
        json.dump(ratios_template["1"], f, indent=4, cls=NumpyEncoder)
    
    # calculate the mean, min, max
    mean_ratios = deepcopy(data_template)
    min_ratios = deepcopy(data_template)
    max_ratios = deepcopy(data_template)
    for key2 in ratios_template.keys():
        for key3 in ratios_template[key2].keys():
            try:
                mean_ratios[key2][key3] = np.mean(ratios_template[key2][key3])
                min_ratios[key2][key3] = np.min(ratios_template[key2][key3])
                max_ratios[key2][key3] = np.max(ratios_template[key2][key3])
            except Exception as e:
                pass
                # some keys are empty, ignore them

    # save the data
    os.makedirs('experiment_ratios', exist_ok=True)
    with open('experiment_ratios/mean_ratios.json', 'w') as f:
        json.dump(mean_ratios, f, indent=4, cls=NumpyEncoder)

    with open('experiment_ratios/min_ratios.json', 'w') as f:
        json.dump(min_ratios, f, indent=4, cls=NumpyEncoder)

    with open('experiment_ratios/max_ratios.json', 'w') as f:
        json.dump(max_ratios, f, indent=4, cls=NumpyEncoder)

    print('The mean, min, max ratios are saved to experiment_ratios folder')

    # plot the data
    plot_number = 0
    for key2 in ratios_template.keys():
        for key3 in ratios_template[key2].keys():
            plt.hist(ratios_template[key2][key3], bins=100, log=True)
            plt.title(f'{key2}_{key3}')
            plt.xlabel('Ratio')
            plt.ylabel('Number of occurences')
            # file name is key2_key3.png
            try:
                plt.savefig(f'experiment_ratios/{key2}_{key3}.png')
            except:
                # a key has a special character, replace it
                plt.savefig(f'experiment_ratios/{key2}_Dinv.png')
            plt.close()
            plot_number += 1



if __name__ == "__main__":
    summary_files = glob.glob('experiment_summaries/*')
    # I assume that there will be only one summary file as I'll only analyse the ranges for double precision
    if len(summary_files) != 1:
        raise ValueError('There should be only one summary file')
    print("Analyzing the summary file: ", summary_files[0])
    
    analyse(summary_files[0])
    










