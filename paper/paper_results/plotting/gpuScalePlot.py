import pandas as pd
import numpy as np
import matplotlib.pyplot as mpl
import sys


mpl.rcParams.update({
    'axes.titlesize': 20,
    'axes.labelsize': 18,
    'lines.linewidth': 1.5,
    'lines.markersize': 6,
    'xtick.labelsize': 16,
    'ytick.labelsize': 16,
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Palatino', 'serif'],
    # "font.serif" : ["Computer Modern Serif"],
})

# if statement ensuring 2 command kine arguments are passed
if (len(sys.argv) != 2):
    print("usage: python3 gpuScalePlot <save>")
    exit()


# File path
file_paths = ['../data/LFDM/11dof.out',
              '../data/LFDM/18dof.out', '../data/LFDM/24dof.out']


vehicles = {"11dof": [], "18dof": [], "24dof": []}
solve_times = {"11dof": [], "18dof": [], "24dof": []}

for file_path in file_paths:
    # Extract the part before out
    file_name = file_path.split('/')[-1].split('.')[0]
    with open(file_path, 'r') as file:
        for line in file:
            if 'Number of vehicles' in line:
                num = int(line.split(': ')[1])
                vehicles[file_name].append(num)
            elif 'Solve time (ms)' in line:
                time = float(line.split(': ')[1])
                solve_times[file_name].append(time)

# Convert each of the lists in the dict to numpy array
for key in vehicles:
    vehicles[key] = np.array(vehicles[key])
    solve_times[key] = np.array(solve_times[key])
    solve_times[key] /= 10000  # To get RTF as the simulation is 10000ms


# Plot the number of vehicles vs solve time
fig, axes = mpl.subplots(nrows=1, ncols=1, figsize=(8, 8))

axes.plot(vehicles["11dof"], solve_times["11dof"], 'r', label='11DoF')
axes.plot(vehicles["18dof"], solve_times["18dof"], 'g', label='18DoF')
axes.plot(vehicles["24dof"], solve_times["24dof"], 'b', label='24DoF')

# Plot horizontal line at y axis = 1
axes.axhline(y=1, color='k', linestyle='--')


# Set labels
axes.set_xlabel("Number of vehicles")
axes.set_ylabel("Real Time Factor (RTF)")
axes.legend(fontsize='large')

# Set x axis ticks to scientific notation
axes.ticklabel_format(style='sci', axis='x', scilimits=(0, 0))

# Set x-axis ticks
axes.set_xticks(
    np.arange(0, max(vehicles['11dof'])+1, 0.5e5))
axes.set_xlim(left=0)

# Set y axis ticks to intervals of 1
axes.yaxis.set_major_locator(mpl.MultipleLocator(base=1))

# Set y axis maximum to 5
axes.set_ylim(top=5)

fig.tight_layout()


# Save the plot
if (sys.argv[1]):
    fig.savefig("./images/gpuScale.png", format='png', facecolor='w', dpi=600)

mpl.show()
