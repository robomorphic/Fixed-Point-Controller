import matplotlib.pyplot as plt

# Data
values = [0.537, 0.254, 0.246, 0.242, 0.240, 0.240, 0.240, 0.240, 0.240, 0.236]
x_labels = ['16', '17', '18', '19', '20', '21', '22', '23', '24', 'double']
custom_pastel_colors = ['#8a9db8'] * (len(x_labels) - 1) + ['#d46a6a']  # Blue for others, red for "double"

# Create the bar graph
plt.figure(figsize=(16, 9))  # 4K aspect ratio
plt.bar(x_labels, values, color=custom_pastel_colors, alpha=0.9)
plt.title('Controller Performance with Different Number Precisions', fontsize=28)
plt.xlabel('Number of Bits', fontsize=24)
plt.ylabel('Controller Performance', fontsize=24)
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)

# Save the figure in 4K resolution
plt.savefig('controller_performance_4k.png', dpi=600)
plt.close()
