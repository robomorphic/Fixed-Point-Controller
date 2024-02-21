import pandas as pd
import numpy as np

import glob

import traj

PARENT_DIRECTORY = 'exp/02-19'


def calculate_goal(time):
    # find the minimum time data that is bigger than time
    # Slow :(
    index = 0
    for i, data in enumerate(traj.time_array):
        if data > time:
            index = i
            break

    # the goal is between index-1 and index
    # we'll do a simple linear interpolation
    prev_pos = np.array(traj.position_array[index-1])
    next_pos = np.array(traj.position_array[index])
    prev_time = traj.time_array[index-1]
    next_time = traj.time_array[index]

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
    distance = np.linalg.norm(goal - position)

    return distance


folders = glob.glob(f'{PARENT_DIRECTORY}/*')

print(folders)

# every folder has data.csv, which we will read and score here

for folder in folders:
    data = pd.read_csv(f'{folder}/data.csv')

    print(data.head())


    # calculate the score and print the sum
    data['score'] = data.apply(calculate_score, axis=1)
    print(folder, data['score'].sum()/len(data))
    
    




