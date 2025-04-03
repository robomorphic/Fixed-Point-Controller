# create a bar graph with dummy data with 2 bars, each element will have 2 data side by side for matplotlib
import matplotlib.pyplot as plt



# data
labels = ['4DoF', '5DoF', '6DoF']
data3 = [1240, 1675, 2074]
data2 = [65, 785, 1078]
data1 = [1167, 882, 992]

title_fontsize = 24

# change title font
plt.rcParams.update({'font.size': 22})

# plot
x = range(len(labels))
width = 0.15

fig, ax = plt.subplots()
ax.bar(x, data1, width, label='16bit count', color='#fc8d62')
ax.bar([i + width for i in x], data2, width, label='32bit count', color='#8da0cb')
ax.bar([i + width*3 for i in x], data3, width, label='Double count', color='#66c2a5')

ax.set_xticks([i + width/2 for i in x])
ax.set_xticklabels(labels)
ax.legend()

# set y axis label
plt.ylabel('KB', fontsize=22)
# set x axis label
plt.xlabel('Robots', fontsize=22)

fig.set_size_inches(12, 7)

# savefig
plt.savefig('bar_graph_op_comparison.png', dpi=600, bbox_inches='tight')


















# data
labels = ['4DoF', '5DoF', '6DoF']
data3 = [1240, 1675, 2074]
data2 = [65, 785, 1078]
data1 = [1167, 882, 992]

title_fontsize = 24

# change title font
plt.rcParams.update({'font.size': 22})

# plot
x = range(len(labels))
width = 0.25

fig, ax = plt.subplots()
#ax.bar(x, data1, width, label='16bit count', color='#fc8d62')
#ax.bar([i + width for i in x], data2, width, label='32bit count', color='#8da0cb')
#ax.bar([i + width*3 for i in x], data3, width, label='Double count', color='#66c2a5')

ax.bar(x, data3, width, label='double', color='#fc8d62')
ax.bar([i + width for i in x], data1, width, label='16-bit int', color='#66c2a5')
ax.bar([i + width for i in x], data2, bottom=data1, width=width, label='32-bit int', color='#8da0cb')

ax.set_xticks([i + width/2 for i in x])
ax.set_xticklabels(labels)
ax.legend()

# set y axis label
plt.ylabel('# of operations', fontsize=22)
# set x axis label
plt.xlabel('Robots', fontsize=22)

plt.title("Number of Operations Required for Forward Dynamics", fontsize=title_fontsize)

fig.set_size_inches(12, 7)

# savefig
plt.savefig('bar_graph_op_comparison2.png', dpi=600, bbox_inches='tight')

















































# plot
labels = ['Double', 'Float', 'Int32', 'Int16']
x = range(len(labels))


data21 = [515, 266, 86, 19]


fig, ax = plt.subplots()
ax.bar([i + width for i in x], data21, width, color='#8da0cb')


ax.set_xticks([i + width for i in x])
ax.set_xticklabels(labels)


# log scale
#plt.yscale('log')

# set y axis label
plt.ylabel('Microsecond', fontsize=22)
# set x axis label
plt.xlabel('Number precision', fontsize=22)

fig.set_size_inches(12, 7)

# set title
plt.title('Runtime in Raspberry Pi Pico for Inverse Dynamics', fontsize=title_fontsize)

# savefig
plt.savefig('bar_graph_pico_runtime.png', dpi=600, bbox_inches='tight')


 
























# plot
labels = ['3DoF', '4DoF']
x = range(len(labels))


data21 = [3.7, 15]
data11 = [215, 515]


fig, ax = plt.subplots()
ax.bar(x, data11, width, label='Naive approach', color='#8da0cb')
ax.bar([i + width for i in x], data21, width, label='Our approach', color='#fc8d62')

ax.set_xticks([i + width/2 for i in x])
ax.set_xticklabels(labels)
ax.legend()

# log scale
plt.yscale('log')

# set y axis label
plt.ylabel('Minutes', fontsize=22)
# set x axis label
plt.xlabel('Robots', fontsize=22)

fig.set_size_inches(12, 7)

# set title
plt.title('Daisy Runtime', fontsize=title_fontsize)

# savefig
plt.savefig('bar_graph_daisy_runtime.png', dpi=600, bbox_inches='tight')

















































import numpy as np

min_frac_bits = 8
max_frac_bits = 11
min_int_bits = 8
max_int_bits = 11


total_scores = np.array(
    [
    [0.537, 0.254, 0.246, 0.242],
    [0.470, 0.254, 0.246, 0.242],
    [0.511, 0.254, 0.246, 0.242],
    [0.488, 0.254, 0.246, 0.242]
    ]
)


fig, ax = plt.subplots()


#plt.plasma()
#plt.jet()
cax = ax.matshow(total_scores)

# Add colorbar
fig.colorbar(cax)


# We want to show all ticks...
ax.set_xticks(np.arange(len(range(min_frac_bits, max_frac_bits+1))))
ax.set_yticks(np.arange(len(range(min_int_bits, max_int_bits+1))))

# ... and label them with the respective list entries
ax.set_xticklabels(range(min_frac_bits, max_frac_bits+1), fontsize=16)
ax.set_yticklabels(range(min_int_bits, max_int_bits+1), fontsize=16)


# Loop over data dimensions and create text annotations, but write the score with 2 decimal points
for i in range(len(range(min_int_bits, max_int_bits+1))):
    for j in range(len(range(min_frac_bits, max_frac_bits+1))):
        text = ax.text(j, i, f'{total_scores[i, j]:.2f}',
                       ha="center", va="center", color="w", fontsize=14)


# name the x and y axis
ax.set_xlabel("Fractional Bits", fontsize=22)
ax.set_ylabel("Integer Bits", fontsize=22)

ax.set_title("Avg Euclidean Distance", fontsize=22)
fig.tight_layout()


plt.savefig('heatmap_nerc.png', dpi=600, bbox_inches='tight')











