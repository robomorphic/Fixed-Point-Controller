heatmap.py assumes that you had an experiment with uniform precision, therefore just reads the integer bits and fraction bits, and plots a heatmap over the data. You'd like to change the EXPERIMENT_DIRECTORY accordingly.

heatmap_var.py assumes that you had different precisions for gravity compensation and forward dynamics. Currently you need to enter the gravity precision by hand in the script, but that will change pretty soon if I need this script more in future.
