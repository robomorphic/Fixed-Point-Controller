import glob

EXPERIMENT_DIRECTORY = "exp/04-16/"

folders = glob.glob(f'{EXPERIMENT_DIRECTORY}/*')

print(folders)

# remove the exp/02-20/ part
folders = [folder[len(EXPERIMENT_DIRECTORY):] for folder in folders]

print(folders)
# separate the int_bits and frac_bits
folders = [folder.split('_') for folder in folders]

# convert all to ints
folders = [[int(x) for x in folder] for folder in folders]

print(folders)

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

total_scores = np.array([[calculate_score_from_folder(int_bits, frac_bits) for frac_bits in range(min_frac_bits, max_frac_bits+1)] for int_bits in range(min_int_bits, max_int_bits+1)])


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
ax.set_xlabel("Fractional Bits")
ax.set_ylabel("Integer Bits")

ax.set_title("Avg Euclidean Distance in Joint Space")
fig.tight_layout()

plt.show()






