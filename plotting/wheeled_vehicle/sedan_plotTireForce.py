import matplotlib.pyplot as mpl
import numpy as np
import sys
import pandas as pd

# Add matplotlib rc parameters to beautify plots
SMALL_SIZE = 12
MEDIUM_SIZE = 14
BIGGER_SIZE = 16

mpl.rc('font', size=SMALL_SIZE)          # controls default text sizes
mpl.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
mpl.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
mpl.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
mpl.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
mpl.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
mpl.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

# Path to 18 DOF sedan files with half implicit solver
file = sys.argv[1]
# Path to the vehicle output
path_out = "../../wheeled_vehicle_models/18dof/data/output/"


data8 = pd.read_csv(
    path_out + file + "_sedan18Hi.csv", sep=' ', header='infer')

# Plot for tire foreces about vehicle x axis
fig, axes = mpl.subplots(nrows=2, ncols=2, figsize=(10, 10), sharey=True)

# Plot tire force of lf
axes[0, 0].plot(data8['time'], data8['lf_tireForce_x'], label='lf_tireForce_x')
axes[0, 0].set_xlabel("Time (s)")
axes[0, 0].set_ylabel("Tire Force (N)")
axes[0, 0].set_title("lf_tireForce_x")

# Plot tire force of rf
axes[0, 1].plot(data8['time'], data8['rf_tireForce_x'], label='rf_tireForce_x')
axes[0, 1].set_xlabel("Time (s)")
axes[0, 1].set_title("rf_tireForce_x")


# Plot tire force of lr
axes[1, 0].plot(data8['time'], data8['lr_tireForce_x'], label='lr_tireForce_x')
axes[1, 0].set_xlabel("Time (s)")
axes[1, 0].set_ylabel("Tire Force (N)")
axes[1, 0].set_title("lr_tireForce_x")


# Plot tire force of rr
axes[1, 1].plot(data8['time'], data8['rr_tireForce_x'], label='rr_tireForce_x')
axes[1, 1].set_xlabel("Time (s)")
axes[1, 1].set_title("rr_tireForce_x")

mpl.show()

# Tire force about vehicle y axis
# Make the plots share the same y axis
fig2, axes2 = mpl.subplots(nrows=2, ncols=2, figsize=(10, 10), sharey=True)


# Plot tire force of lf
axes2[0, 0].plot(data8['time'], data8['lf_tireForce_y'],
                 label='lf_tireForce_y')
axes2[0, 0].set_xlabel("Time (s)")
axes2[0, 0].set_ylabel("Tire Force (N)")
axes2[0, 0].set_title("lf_tireForce_y")

# Plot tire force of rf
axes2[0, 1].plot(data8['time'], data8['rf_tireForce_y'],
                 label='rf_tireForce_y')
axes2[0, 1].set_xlabel("Time (s)")
axes2[0, 1].set_title("rf_tireForce_y")


# Plot tire force of lr
axes2[1, 0].plot(data8['time'], data8['lr_tireForce_y'],
                 label='lr_tireForce_y')
axes2[1, 0].set_xlabel("Time (s)")
axes2[1, 0].set_ylabel("Tire Force (N)")
axes2[1, 0].set_title("lr_tireForce_y")

# Plot tire force of rr
axes2[1, 1].plot(data8['time'], data8['rr_tireForce_y'],
                 label='rr_tireForce_y')
axes2[1, 1].set_xlabel("Time (s)")
axes2[1, 1].set_title("rr_tireForce_y")


# Show this plot
mpl.show()

# Tire force about vehicle z axis
# Make the plots share the same y axis
fig3, axes3 = mpl.subplots(nrows=2, ncols=2, figsize=(10, 10), sharey=True)

# Plot tire force of lf
axes3[0, 0].plot(data8['time'], data8['lf_tireForce_z'],
                 label='lf_tireForce_z')
axes3[0, 0].set_xlabel("Time (s)")
axes3[0, 0].set_ylabel("Tire Force (N)")
axes3[0, 0].set_title("lf_tireForce_z")

# Plot tire force of rf
axes3[0, 1].plot(data8['time'], data8['rf_tireForce_z'],
                 label='rf_tireForce_z')
axes3[0, 1].set_xlabel("Time (s)")
axes3[0, 1].set_title("rf_tireForce_z")

# Plot tire force of lr
axes3[1, 0].plot(data8['time'], data8['lr_tireForce_z'],
                 label='lr_tireForce_z')
axes3[1, 0].set_xlabel("Time (s)")
axes3[1, 0].set_ylabel("Tire Force (N)")
axes3[1, 0].set_title("lr_tireForce_z")

# Plot tire force of rr
axes3[1, 1].plot(data8['time'], data8['rr_tireForce_z'],
                 label='rr_tireForce_z')
axes3[1, 1].set_xlabel("Time (s)")
axes3[1, 1].set_title("rr_tireForce_z")


# Show this plot
mpl.show()