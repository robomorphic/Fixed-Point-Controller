import glob
import os
import json
import numpy as np
import pandas as pd




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

# This function finds out the sparsity of the data, and returns the sparse versions
# For diagonal matrices, it will return a vector with the diagonal elements
# Otherwise, it is not implemented yet
def matrix_vector_simplification(data):
    # is data a diagonal matrix?
    if np.all(data == np.diag(np.diag(data))):
        return np.diag(data)
    else:
        print("Not implemented yet, data is not a diagonal matrix")
        print("data: ", data)
        return data

def Minv_analysis(json_file: str):
    json_data = json.load(open(json_file))
    json_data_keys = list(json_data.keys())
    bigger_resolution_integer_bits = json_data["integer_bits_standard"]
    bigger_resolution_fractional_bits = json_data["fractional_bits_standard"]
    lower_resolution_integer_bits = json_data["integer_bits_fd"]
    lower_resolution_fractional_bits = json_data["fractional_bits_fd"]

    # I'll only read a few keys, delete the rest
    # iterate over keys in json_data
    for i in json_data_keys:
        # if the key is not a number, skip
        if not i.isdigit():
            # delete the key
            del json_data[i]
            continue
        # now delete the keys that are not fixed_point_Minv_output and original_data_Minv_output
        json_data_i_keys = list(json_data[i].keys())
        for j in json_data_i_keys:
            if j != 'fixed_point_Minv_output' and j != 'original_data_Minv_output':
                del json_data[i][j]

    # now we are ready for process
    json_data_keys = list(json_data.keys())
    for i in json_data_keys:
        # get the fixed point and original data
        fixed_point = json_data[i]['fixed_point_Minv_output']
        original_data = json_data[i]['original_data_Minv_output']
        fixed_point = list_string_to_matrix(fixed_point)
        original_data = list_string_to_matrix(original_data)
        # simplify the matrices
        fixed_point = matrix_vector_simplification(fixed_point)
        original_data = matrix_vector_simplification(original_data)
        # store the simplified matrices
        json_data[i]['fixed_point_Minv_output'] = fixed_point
        json_data[i]['original_data_Minv_output'] = original_data

    # now, analyze
    # before that, put everything on a dataframe for easier analysis
    df = pd.DataFrame(json_data).T

    # calculate the difference between fixed_point and original_data
    df['diff'] = df['fixed_point_Minv_output'] - df['original_data_Minv_output']
    # calculate the error in percentage
    df['error'] = df['diff'] / df['original_data_Minv_output'] * 100

    #print("df: ", df)

    # calculate the mean for each column in error
    #print("mean: ", df['error'].mean())

    # plot the distribution of the error
    errors = []
    for i in range(6):
        errors.append([])
        # append the first element from each element of the column
        for j in df['error']:
            errors[i].append(j[i])

    # plot the errors
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(3, 2)
    # set name
    fig.suptitle('Error distributions for Minv output matrices\n' + 'big resolution: ' + str(bigger_resolution_integer_bits) + '.' + str(bigger_resolution_fractional_bits) + ' lower resolution: ' + str(lower_resolution_integer_bits) + '.' + str(lower_resolution_fractional_bits))
    for i in range(3):
        for j in range(2):
            # print the counts of the bins in the plot, but do not write 0s
            counts, bins, patches = axs[i, j].hist(errors[i*2+j], bins=100)
            for count, bin in zip(counts, bins):
                if count != 0:
                    axs[i, j].text(bin, count, str(int(count)), fontsize=3)

            # open autoscale for x
            axs[i, j].autoscale(enable=True, axis='x', tight=True)

            axs[i, j].set_title('error in element ' + str(i*2+j), fontsize=8)
            # set x y labels
            axs[i, j].set_xlabel('error percentage', fontsize=8)
            axs[i, j].set_ylabel('frequency', fontsize=8)
            # set x y values font size
            axs[i, j].tick_params(axis='both', which='major', labelsize=8)

    # save the fig to experiment_data/latest_folder/figs
    fig_folder = 'experiment_summaries/figs'
    os.makedirs(fig_folder, exist_ok=True)

    # put more distance between subplots
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    fig.savefig(
        fig_folder + f'/Minv_error_distribution_{bigger_resolution_integer_bits}_{bigger_resolution_fractional_bits}_{lower_resolution_integer_bits}_{lower_resolution_fractional_bits}.png',
        dpi=900
        )

    # now repeat the same, but calculate rmse and plot
    # calculate the rmse for each column in error
    #print("rmse: ", np.sqrt(np.mean(df['error']**2)))

# Assume all inputs will be matrices
def joint_by_joint_key_analysis_matrix(json_file: str, key: str):
    json_data = json.load(open(json_file))
    bigger_resolution_integer_bits = json_data["integer_bits_standard"]
    bigger_resolution_fractional_bits = json_data["fractional_bits_standard"]
    lower_resolution_integer_bits = json_data["integer_bits_fd"]
    lower_resolution_fractional_bits = json_data["fractional_bits_fd"]

    del json_data["integer_bits_standard"]
    del json_data["fractional_bits_standard"]
    del json_data["integer_bits_fd"]
    del json_data["fractional_bits_fd"]
    del json_data["integer_bits_gravity"]
    del json_data["fractional_bits_gravity"]

    json_data_keys = list(json_data.keys())

    # delete the keys that are fixed_point_Minv_output and original_data_Minv_output
    # iterate over keys in json_data
    for i in json_data_keys:
        #print("i: ", i)
        # these keys are iteration counts
        json_sub_data_keys = list(json_data[i].keys())
        # these keys are joint indexes
        for j in json_sub_data_keys:
            # if the key is not the "key", delete it
            #print("j: ", j)
            if j == "fixed_point_Minv_output" or j == "original_data_Minv_output":
                del json_data[i][j]
                continue

            json_sub_sub_data_keys = list(json_data[i][j].keys())
            for k in json_sub_sub_data_keys:
                if k != key:
                    del json_data[i][j][k]
    
    # let's flatten the json even more
    # now we have a json like this
    # 0:
    #   0:
    #     key:
    #       small_resolution: matrix
    #       big_resolution: matrix
    # make it
    # 0_0:
    #   key:
    #     small_resolution: matrix
    #     big_resolution: matrix

    # iterate over keys in json_data
    for i in json_data_keys:
        # these keys are iteration counts
        json_sub_data_keys = list(json_data[i].keys())
        # these keys are joint indexes
        for j in json_sub_data_keys:
            #print("j: ", j)
            # these keys are the key
            json_sub_sub_data_keys = list(json_data[i][j].keys())
            for k in json_sub_sub_data_keys:
                #print("k: ", k)
                # change the key to i_j
                json_data[i + '_' + j] = json_data[i].pop(j)

    # delete the original keys
    for i in json_data_keys:
        del json_data[i]

    with open('/Users/alp/trial.json', 'w') as f:
        json.dump(json_data, f, indent=4)

    # the rest can be much more simpler, and closer to Minv_analysis

    json_data_keys = list(json_data.keys())
    for i in json_data_keys:
        fixed_point = json_data[i][key]['small_resolution']
        original_data = json_data[i][key]['big_resolution']
        fixed_point = list_string_to_matrix_v2(fixed_point)
        original_data = list_string_to_matrix_v2(original_data)
        # store the matrices
        json_data[i]['small_resolution'] = fixed_point
        json_data[i]['big_resolution'] = original_data
        del json_data[i][key]

    # now, analyze
    # before that, put everything on a dataframe for easier analysis
    df = pd.DataFrame(json_data).T

    # calculate the difference between fixed_point and original_data
    df['diff'] = df['small_resolution'] - df['big_resolution']
    # calculate the error in percentage
    df['error'] = df['diff'] / df['big_resolution'] * 100

    #print("df: ", df)

    # calculate the mean for each column in error
    #print("mean: ", df['error'].mean())

    # plot the distribution of the error
    errors = []
    for i in range(6):
        errors.append([])
        for j in range(6):
            errors[i].append([])
            # append the first element from each element of the column
            for k in df['error']:
                #isnan
                if k[i][j] != 0 and not np.isnan(k[i][j]):
                    errors[i][j].append(k[i][j])
                

    # plot the errors
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(6, 6)
    # set name
    fig.suptitle('Error distributions for ' + key + ' matrices\n' + 'big resolution: ' + str(bigger_resolution_integer_bits) + '.' + str(bigger_resolution_fractional_bits) + ' lower resolution: ' + str(lower_resolution_integer_bits) + '.' + str(lower_resolution_fractional_bits))
    for i in range(6):
        for j in range(6):
            counts, bins, patches = axs[i, j].hist(errors[i][j], bins=100)
            axs[i, j].set_title('error in element ' + str(i) + ',' + str(j), fontsize=3)
            # set x y labels
            #axs[i, j].set_xlabel('error percentage', fontsize=8)
            #axs[i, j].set_ylabel('frequency', fontsize=8)
            # set x y values font size
            axs[i, j].tick_params(axis='both', which='major', labelsize=4)

    # save the fig to experiment_data/latest_folder/figs
    fig_folder = 'experiment_summaries/figs'
    os.makedirs(fig_folder, exist_ok=True)

    # put more distance between subplots
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    fig.savefig(
        fig_folder + f'/{key}_error_distribution_{bigger_resolution_integer_bits}_{bigger_resolution_fractional_bits}_{lower_resolution_integer_bits}_{lower_resolution_fractional_bits}.png',
        dpi=900
        )

    # plot the means for the matrix
    means = []
    for i in range(6):
        means.append([])
        for j in range(6):
            means[i].append(np.mean(errors[i][j]))

    fig, axs = plt.subplots(1, 1)
    # set name
    fig.suptitle('Mean error for ' + key + ' matrices\n' + 'big resolution: ' + str(bigger_resolution_integer_bits) + '.' + str(bigger_resolution_fractional_bits) + ' lower resolution: ' + str(lower_resolution_integer_bits) + '.' + str(lower_resolution_fractional_bits)
    )
    # plot the means
    axs.imshow(means, cmap='hot', interpolation='nearest')
    # save the fig to experiment_data/latest_folder/figs
    fig_folder = 'experiment_summaries/figs'
    os.makedirs(fig_folder, exist_ok=True)

    # put more distance between subplots
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    # put a scale
    cbar = fig.colorbar(axs.imshow(means, cmap='hot', interpolation='nearest'))

    # write the values on the cells
    #for i in range(6):
    #    for j in range(6):
    #        axs.text(j, i, str(round(means[i][j], 2)), ha='center', va='center', color='black')

    fig.savefig(
        fig_folder + f'/{key}_mean_error_{bigger_resolution_integer_bits}_{bigger_resolution_fractional_bits}_{lower_resolution_integer_bits}_{lower_resolution_fractional_bits}.png',
        dpi=900
        )

    
        

if __name__ == '__main__':
    summary_files = glob.glob('experiment_summaries/*')
    for summary_file in summary_files:
        print("summary_file: ", summary_file)
        if 'figs' in summary_file:
            continue # this is a folder
        #Minv_analysis(summary_file)
        joint_by_joint_key_analysis_matrix(summary_file, "Yaba")

