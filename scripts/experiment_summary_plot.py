import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import glob
import json
from tqdm import tqdm
import json

def float_close_to_n(x, n):
    return abs(x - n) < 1e-6

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

def matrix_to_plot(
    data: list,
    integer_bits_original: int,
    fractional_bits_original: int,
    integer_bits_fd: int,
    fractional_bits_fd: int
    ):
    print("data", data)
    # data1 is the original one, data2 is the fixed point one
    # the difference will be shown as ratio, so that we can see the relative error
    # first, calculate the difference
    differences = []
    for i in tqdm(range(len(data))):
        diff = data[i][1] - data[i][0]
        differences.append(diff)
    
    print("differences", differences)

    # then, calculate the ratio
    ratios = []
    for i in tqdm(range(len(differences))):
        ratio = differences[i] / data[i][0]
        ratios.append(ratio)

    print("ratios", ratios)

    # convert nan's to 0
    for i in tqdm(range(len(ratios))):
        ratios[i] = np.nan_to_num(ratios[i])

    # average the ratios
    avg_ratio = np.mean(ratios, axis=0)

    print(avg_ratio)

    median_ratio = np.median(ratios, axis=0)
    # plot the ratio
    # the colors will be white for 0
    # the colors will be red for negative until -1
    # the colors will be blue for positive until 1
    # create colormap
    cmap = plt.cm.coolwarm
    # plot the matrix, write the values in diagonal cells
    plt.matshow(avg_ratio, cmap=cmap)
    for i in range(avg_ratio.shape[0]):
        for j in range(avg_ratio.shape[1]):
            plt.text(j, i, f"{avg_ratio[i, j]:.2f}", ha='center', va='center')

    #plt.xlim(-0.5, 5.5)
    #plt.ylim(-0.5, 5.5)
    # add legend
    plt.colorbar()
    plt.title(f"Average relative error of Minv matrix\nOriginal: {integer_bits_original}.{fractional_bits_original}, Fixed point: {integer_bits_fd}.{fractional_bits_fd}")
    # save
    plt.savefig(f"experiment_data/{run_id}/avg_ratio.png")

    # now median
    # plot the matrix, write the values in diagonal cells
    plt.matshow(median_ratio, cmap=cmap)
    for i in range(median_ratio.shape[0]):
        for j in range(median_ratio.shape[1]):
            plt.text(j, i, f"{median_ratio[i, j]:.2f}", ha='center', va='center')

    #plt.xlim(-0.5, 5.5)
    #plt.ylim(-0.5, 5.5)
    # add legend
    plt.colorbar()
    plt.title(f"Median relative error of Minv matrix\nOriginal: {integer_bits_original}.{fractional_bits_original}, Fixed point: {integer_bits_fd}.{fractional_bits_fd}")
    plt.savefig(f"experiment_data/{run_id}/median_ratio.png")


def plot(data: dict, run_id: str):
    integer_bits_standard = data["integer_bits_standard"]
    fractional_bits_standard = data["fractional_bits_standard"]
    integer_bits_gravity = data["integer_bits_gravity"]
    fractional_bits_gravity = data["fractional_bits_gravity"]
    integer_bits_fd = data["integer_bits_fd"]
    fractional_bits_fd = data["fractional_bits_fd"]

    # not good, but I'll store each data in a list
    all_data = []
    # The rest is just loop index from 1 to some unknown N
    i = 0
    while True:
        if str(i) not in data:
            break
        fixed_point_Minv_output = data[str(i)]["fixed_point_Minv_output"]
        original_data_Minv_output = data[str(i)]["original_data_Minv_output"]
        # now, plot the difference between the two
        # the difference will be shown as ratio, so that we can see the relative error
        # this is important because the absolute error is not very informative

        # first preprocess
        fixed_point_Minv_output = list_string_to_matrix(fixed_point_Minv_output)
        original_data_Minv_output = list_string_to_matrix(original_data_Minv_output)

        # if there are any 0 in diagonals of original_data_Minv_output, do not append
        for j in range(original_data_Minv_output.shape[0]):
            if float_close_to_n(original_data_Minv_output[j, j], 0):
                print(f"Found 0 in {i}")
                continue
        
        for j in range(fixed_point_Minv_output.shape[0]):
            if float_close_to_n(fixed_point_Minv_output[j, j], 0):
                print(f"Found 0 in {i}")
                continue

        all_data.append([original_data_Minv_output, fixed_point_Minv_output])
        
        i += 1
        
    # then plot
    matrix_to_plot(
        all_data,
        integer_bits_standard,
        fractional_bits_standard,
        integer_bits_fd,
        fractional_bits_fd)


def plot_each_iter_of_a_run(run: str):
    with open(run + '/experiment_summary.json', 'r') as f:
        data = json.load(f)
    # iterate over fixed_Minv and original_Minv of each iteration
    i = 0
    png_list = []
    while True:
        print(i)
        # if data[str(i)] exists
        if str(i) in data:
            fixed_Minv = data[str(i)]["fixed_point_Minv_output"]
            original_Minv = data[str(i)]["original_data_Minv_output"]
            fixed_Minv = list_string_to_matrix(fixed_Minv)
            original_Minv = list_string_to_matrix(original_Minv)
            #print("fixed_Minv", fixed_Minv)
            #print("original_Minv", original_Minv)

            difference = np.array(fixed_Minv) - np.array(original_Minv)
            #print("difference", difference)
            ratio = difference / np.array(original_Minv)
            #print("ratio", ratio)

            # plot it, save it to run+i/ratio.png
            cmap = plt.cm.coolwarm
            plt.matshow(ratio, cmap=cmap)
            for k in range(ratio.shape[0]):
                for j in range(ratio.shape[1]):
                    plt.text(j, k, f"{ratio[k, j]:.2f}", ha='center', va='center')
            plt.colorbar()
            plt.title(f"Relative error of Minv matrix\nIteration {i}")
            plt.savefig(f"{run}/ABA/{i}/ratio.png")
            #png_list.append(f"file {run}/ABA/{i}/ratio.png")
            png_list.append(f"file ./ABA/{i}/ratio.png")
            # clear pyplot
            plt.close()
            
            i += 1
        else:
            break
    
    # save png_list
    with open(f"{run}/png_list.txt", 'w') as f:
        for png in png_list:
            f.write(png + '\n')

    
    # save all of these figures into an mp4 file
    # ffmpeg -f concat -safe 0 -r 1 -i f"{run}/ABA/png_list.txt" -c:v libx264 -vf "fps=25,format=yuv420p" f"{run}/ABA/ABA.mp4"
    import os
    os.system(f"ffmpeg -f concat -safe 0 -r 25 -i {run}/png_list.txt -c:v libx264 -vf 'fps=25,format=yuv420p' {run}/ABA.mp4")
    
        


# I'll assume that I need to read and plot *every* run in the experiment data
# each run is a different folder
# the experiment data can be read from experiment_summary.json file, please do not read anything else here
# the raw data is already pretty complicated, I don't want to make it more complicated
all_runs = glob.glob("experiment_data/*")
for run in all_runs:
    with open(run + '/experiment_summary.json', 'r') as f:
        data = json.load(f)
    run_id = run.split('/')[-1]
    #plot(data, run_id)
    plot_each_iter_of_a_run(run)







