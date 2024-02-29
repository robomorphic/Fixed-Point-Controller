import glob

EXPERIMENT_DIRECTORY = "exp/02-25/"

folders = glob.glob(f'{EXPERIMENT_DIRECTORY}/*')

print(folders)

# remove the {EXPERIMENT_DIRECTORY} part
folders = [folder[len(EXPERIMENT_DIRECTORY):] for folder in folders]

print(folders)
# separate the int_bits and frac_bits
folders = [folder.split('_') for folder in folders]

GRAVITY_INT = '9'
GRAVITY_FRAC = '8'

# convert all to ints
# filter the list so that only the ones with two 8's are left
folders = [[int(x) for x in folder] for folder in folders if folder[0] == GRAVITY_INT and folder[1] == GRAVITY_FRAC]

print(folders)

# now remove the first two elements from each folder
folders = [folder[2:] for folder in folders]

folders = sorted(folders)

print(folders)

min_int_bits = folders[0][0]
max_int_bits = folders[-1][0]
min_frac_bits = folders[0][1]
max_frac_bits = folders[-1][1]

# now read all and calculate the scores, then heatmap
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from score import calculate_score_from_folder

#total_scores = [[calculate_score_from_folder(int_bits, frac_bits) for frac_bits in range(min_frac_bits, max_frac_bits+1)] for int_bits in range(min_int_bits, max_int_bits+1)]
print("total scores")
total_scores = np.array([[calculate_score_from_folder(GRAVITY_INT, GRAVITY_FRAC, int_bits, frac_bits) for frac_bits in range(min_frac_bits, max_frac_bits+1)] for int_bits in range(min_int_bits, max_int_bits+1)])
print('end')
print(total_scores)

fig, ax = plt.subplots()
cax = ax.matshow(total_scores)

# Add colorbar
fig.colorbar(cax)

# We want to show all ticks...
ax.set_xticks(np.arange(len(range(min_frac_bits, max_frac_bits+1))))
ax.set_yticks(np.arange(len(range(min_int_bits, max_int_bits+1))))

# ... and label them with the respective list entries
ax.set_xticklabels(range(min_frac_bits, max_frac_bits+1))
ax.set_yticklabels(range(min_int_bits, max_int_bits+1))

# Rotate the tick labels and set their alignment.
plt.setp(ax.get_xticklabels(), rotation=45, ha="right",
         rotation_mode="anchor")

# Loop over data dimensions and create text annotations, but write the score with 3 decimal points
for i in range(len(range(min_int_bits, max_int_bits+1))):
    for j in range(len(range(min_frac_bits, max_frac_bits+1))):
        text = ax.text(j, i, f'{total_scores[i, j]:.3f}',
                       ha="center", va="center", color="w")

# name the x and y axis
ax.set_xlabel("Forward Dynamics Fractional Bits")
ax.set_ylabel("Forward Dynamics Integer Bits")

ax.set_title(f"Avg Euclidean Distance in Joint Space for Gravity Integer Bits: {GRAVITY_INT}, Gravity Fractional Bits: {GRAVITY_FRAC}")
fig.tight_layout()

plt.show()






