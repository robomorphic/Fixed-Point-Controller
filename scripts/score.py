import pandas as pd
import numpy as np

import glob

import traj_panda as traj_panda

PARENT_DIRECTORY = 'exp/09-25'


def calculate_goal(time):
    # find the minimum time data that is bigger than time
    # Slow :(
    index = 0
    for i, data in enumerate(traj_panda.time_array):
        if data > time:
            index = i
            break

    # the goal is between index-1 and index
    # we'll do a simple linear interpolation
    prev_pos = np.array(traj_panda.position_array[index-1])
    next_pos = np.array(traj_panda.position_array[index])
    prev_time = traj_panda.time_array[index-1]
    next_time = traj_panda.time_array[index]

    # calculate the time difference
    time_diff = next_time - prev_time
    time_ratio = (time - prev_time) / time_diff

    # calculate the position difference
    pos_diff = next_pos - prev_pos
    goal = prev_pos + pos_diff * time_ratio

    return goal

def calculate_score(row):
    time = row['time']
    goal = calculate_goal(time)
    position = [row['q_1'], row['q_2'], row['q_3'], row['q_4'], row['q_5'], row['q_6']]

    #print("time: ", time)
    #print("goal: ", goal)
    #print("position: ", position)

    # calculate the distance
    #print("goal: ", goal)
    #print("position: ", position)
    distance = np.linalg.norm(goal - position)

    return distance

    
def calculate_score_from_folder(int_bits, frac_bits):
    folder = f'{PARENT_DIRECTORY}/{int_bits}_{frac_bits}'
    try:
        data = pd.read_csv(f'{folder}/data.csv')
    except Exception as e:
        print(e)
        return float('inf') # there may be an error because the df is empty
    # is data empty
    if data.empty:
        return float('inf')

    # calculate the score and print the sum
    data['score'] = data.apply(calculate_score, axis=1)
    return data['score'].sum()/len(data)

def calculate_score_from_folder(gravity_int_bits, gravity_frac_bits, fd_int_bits, fd_frac_bits):
    folder = f'{PARENT_DIRECTORY}/{gravity_int_bits}_{gravity_frac_bits}_{fd_int_bits}_{fd_frac_bits}'
    print("reading from ", folder)
    try:
        data = pd.read_csv(f'{folder}/data.csv')
    except Exception as e:
        print(e)
        return float('inf') # there may be an error because the df is empty
    # is data empty
    if data.empty:
        return float('inf')

    print('data read')

    # calculate the score and print the sum
    data['score'] = data.apply(calculate_score, axis=1)
    return data['score'].sum()/len(data)


if __name__ == "__main__":

    folders = glob.glob(f'{PARENT_DIRECTORY}/*')

    print(folders)

    # every folder has data.csv, which we will read and score here

    for folder in folders:
        data = pd.read_csv(f'{folder}/data.csv')
        # is data empty
        if data.empty:
            print(folder, "inf")
            continue

        # calculate the score and print the sum
        data['score'] = data.apply(calculate_score, axis=1)
        print(folder, data['score'].sum()/len(data))
        