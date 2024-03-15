import glob
import os

# read model_output folder, get all folder names choose the latest one
folders = glob.glob('model_output/*')
latest_folder = max(folders, key=os.path.getctime)

experiment_info = {}

# the folder should have two files and one folder
# files are model_modified.txt and model_original.txt
# for now we can just read the model_modified, and check the integer bits and fractional bits, and report them
# the folder should be "ABA"
# each folder in ABA will include 5 files
# fixed_point.txt and original_data.txt are the Minv matrices, need to read them
# the other 3 files are pass1-2-3.txt, which are the output of the 3 passes
# we will read them carefully
# I will firstly read EVERYTHING in these 3 files, later we can decide

def read_bit_info():
    # read the model_modified.txt
    with open(latest_folder + '/model_smaller.txt', 'r') as f:
        model_modified = f.read()
        # get the first 6 lines of the file
        model_modified = model_modified.split('\n')
        model_modified = model_modified[:6]
        # get the integer bits and fractional bits
        for line in model_modified:
            line = line.split(" ")
            line[0] = line[0][:-1]
            experiment_info[line[0]] = line[1]

def read_pass_joint_info(folder, folder_index, lines, line_index, is_currently_original=False):
    # pass1[line_index] is the starting line
    # pass1[line_index] is "Joint: {joint_index}"
    joint_index = int(lines[line_index].split(" ")[1])
    if joint_index not in experiment_info[folder_index]:
        experiment_info[folder_index][joint_index] = {}
    # skip the 2nd line
    # after 2, the rest of the lines are structured as follows:
    # {name_of_var}:
    # {value}
    # {name_of_2ndvar}:
    # {value}
    # we will read the file line by line, and store the values in a dictionary
    fixed_original_key = "big_resolution" if is_currently_original else "small_resolution"
    curr_var = ""
    print("folder_index: " + str(folder_index))
    for i in range(line_index + 1, len(lines)):
        print("i: " + str(i))
        print(lines[i])
        # if the line starts with "Joint: ", then we have finished reading the joint
        if "Joint: " in lines[i]: 
            print("Joint finished" + str(folder_index))
            break
        # if the line ends with ": ", then it is the name of the variable
        elif lines[i][-2:] == ": ":
            curr_var = lines[i][:-2]
            print("curr_var: ")
            print(curr_var)
            # experiment_info[folder_index][joint_index][curr_var] = {} if not exists
            #print curr keys
            if not is_currently_original:
                experiment_info[folder_index][joint_index][curr_var] = {}
            experiment_info[folder_index][joint_index][curr_var][fixed_original_key] = ""
        # otherwise, it is the value of the variable
        # keep appending the value to the dictionary
        else:
            print("appending")
            experiment_info[folder_index][joint_index][curr_var][fixed_original_key] += lines[i] + "\n"



def read_pass1_info(folder, folder_index):
    with open(folder + '/pass1.txt', 'r') as f:
        pass1 = f.read()
        pass1 = pass1.split('\n')
        # split the pass1 into 12 parts, each starts with "Joint: {joint_index}"
        joint_start_lines = [i for i in range(len(pass1)) if "Joint: " in pass1[i]]
        for i in range(int(len(joint_start_lines)/2)):
            read_pass_joint_info(folder, folder_index, pass1, joint_start_lines[i], False)
        for i in range(int(len(joint_start_lines)/2), len(joint_start_lines)):
            read_pass_joint_info(folder, folder_index, pass1, joint_start_lines[i], True)

def read_pass2_info(folder, folder_index):
    with open(folder + '/pass2.txt', 'r') as f:
        pass2 = f.read()
        pass2 = pass2.split('\n')
        joint_start_lines = [i for i in range(len(pass2)) if "Joint: " in pass2[i]]
        for i in range(int(len(joint_start_lines)/2)):
            read_pass_joint_info(folder, folder_index, pass2, joint_start_lines[i], False)
        for i in range(int(len(joint_start_lines)/2), len(joint_start_lines)):
            read_pass_joint_info(folder, folder_index, pass2, joint_start_lines[i], True)

def read_pass3_info(folder, folder_index):
    with open(folder + '/pass3.txt', 'r') as f:
        pass3 = f.read()
        pass3 = pass3.split('\n')
        # split the pass1 into 12 parts, each starts with "Joint: {joint_index}"
        joint_start_lines = [i for i in range(len(pass3)) if "Joint: " in pass3[i]]
        print(joint_start_lines)
        for i in range(int(len(joint_start_lines)/2)):
            print("giving start_index: " + str(joint_start_lines[i]))
            read_pass_joint_info(folder, folder_index, pass3, joint_start_lines[i], False)
        for i in range(int(len(joint_start_lines)/2), len(joint_start_lines)):
            read_pass_joint_info(folder, folder_index, pass3, joint_start_lines[i], True)



def read_aba_info():
    # read the ABA folder
    folders = glob.glob(latest_folder + '/ABA/*')
    for folder in folders:
        folder_index = int(folder.split('/')[-1])
        experiment_info[folder_index] = {}
        # read the fixed_point.txt
        with open(folder + '/fixed_point_data.txt', 'r') as f:
            fixed_point = f.read()
            fixed_point = fixed_point.split('\n')
            experiment_info[folder_index]['fixed_point_Minv_output'] = fixed_point
        # read the original_data.txt
        with open(folder + '/original_data.txt', 'r') as f:
            original_data = f.read()
            original_data = original_data.split('\n')
            experiment_info[folder_index]['original_data_Minv_output'] = original_data
        # read the pass1.txt
        read_pass1_info(folder, folder_index)
        # read the pass2.txt
        read_pass2_info(folder, folder_index)
        # read the pass3.txt
        read_pass3_info(folder, folder_index)


read_bit_info()
read_aba_info()

# write it into a json file with indentation

import json
with open(latest_folder + '/experiment_summary.json', 'w') as f:
    json.dump(experiment_info, f, indent=4)
