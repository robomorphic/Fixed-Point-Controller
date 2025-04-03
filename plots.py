# create a bar graph with dummy data with 2 bars, each element will have 2 data side by side for matplotlib
import matplotlib.pyplot as plt

# data
labels = ['Robot A', 'Robot B', 'Robot C', 'Robot D']
data1 = [10, 20, 30, 40]
data2 = [20, 30, 40, 50]

# plot
x = range(len(labels))
width = 0.35

fig, ax = plt.subplots()
ax.bar(x, data1, width, label='Naive approach')
ax.bar([i + width for i in x], data2, width, label='Our approach')

ax.set_xticks([i + width/2 for i in x])
ax.set_xticklabels(labels)
ax.legend()

# set y axis label
plt.ylabel('Hz')
# set x axis label
plt.xlabel('Robots')

# set title
plt.title('Optimal controller performance with different robots for X CPU')

# savefig
plt.savefig('bar_graph_robot_performance.png')

# same plot, except the title
fig, ax = plt.subplots()
ax.bar(x, data1, width, label='Our approach')
ax.bar([i + width for i in x], data2, width, label='Naive approach')

ax.set_xticks([i + width/2 for i in x])
ax.set_xticklabels(labels)
ax.legend()

# set y axis label
plt.ylabel('MB')
# set x axis label
plt.xlabel('Robots')

# set title
plt.title('Controller memory usage with different robots for X CPU')

# savefig
plt.savefig('bar_graph_robot_memory.png')




 


