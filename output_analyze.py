

# There are files with names output0.txt, output1.txt, output2.txt, ..., output4.txt.
# Each file is like:  
# qpos -2.897 -1.763 -2.897 -3.072 -2.897 0.4537
# Minv: 
# 2.213 0 0 0 0 0 
# 0 1.609 0 0 0 0 
# 0 0 4.456 0 0 0 
# 0 0 0 1.747 0 0 
# 0 0 0 0 26.21 0 
# 0 0 0 0 0 31.5 
# qpos -2.897 -1.763 -2.897 -3.072 -2.897  0.925
# Minv: 
# 2.122 0 0 0 0 0 
# 0 1.591 0 0 0 0 
# 0 0 4.296 0 0 0 
# 0 0 0 1.946 0 0 
# 0 0 0 0 26.82 0 
# 0 0 0 0 0 31.5 
# ...

# So, I need to read the qpos and Minv values, and print them in the following format:
# q: [-2.897, -1.763, -2.897, -3.072, -2.897, 0.4537], Minv: [2.122, 1.591, 4.296, 1.946, 26.82, 31.5]
# For Minv values, I need to print only the diagonal values.

# I need to read all the files and print the values in the above format.

import sys

files = ['output0.txt', 'output1.txt', 'output2.txt', 'output3.txt', 'output4.txt']

for file in files:
    with open(file, 'r') as f:
        lines = f.readlines()
        i = 0
        while i < len(lines):
            if lines[i].startswith('qpos'):
                # Extract qpos values
                qpos = list(map(float, lines[i].split()[1:]))
                # Extract Minv diagonal values
                Minv = []
                i += 2  # Skip the 'Minv:' line
                for j in range(6):
                    Minv.append(float(lines[i + j].split()[j]))  # Diagonal values are at the j-th index of the j-th row
                
                # if there are some values that are below 0.5 in Minv, print them in stderr
                for j in range(6):
                    if Minv[j] > 60:
                        # print Minv and qpos in stderr
                        print(f'Minv: {Minv} \t q: {qpos}', file=sys.stderr)

                # Format the qpos and Minv with tabs for alignment
                qpos_str = '\t'.join(f'{x:.4f}' for x in qpos)
                Minv_str = '\t'.join(f'{x:.4f}' for x in Minv)
                
                print(f'Minv:\t{Minv_str} \t q:\t')
            i += 1




