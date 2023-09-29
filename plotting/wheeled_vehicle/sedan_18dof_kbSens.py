import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import sys


# Add matplotlib rc parameters to beautify plots
SMALL_SIZE = 12
MEDIUM_SIZE = 14
BIGGER_SIZE = 16

plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=SMALL_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title


# Path to 18 DOF sedan files with half implicit solver
file = sys.argv[1]
# Path to the vehicle output
path_out = "../../wheeled_vehicle_models/18dof/data/output/"

# trial_k = ['04', '05', '06', '07']
# trial_b = ['03', '04', '05', '06']

# trial_k = ['04']
# trial_b = ['110', '120', '130', '140', '150']

trial_k = ['100', '200', '300', '400']
# trial_b = ['03', '04', '05', '06']
trial_b = ['03', '04']


num_colors = 16
new_cycle = plt.cycler(color=plt.cm.tab20.colors[:num_colors])
# create a list of 16 colors for matplotlib to loop over
plt.rcParams['axes.prop_cycle'] = new_cycle


# Loop through all trial k and trial b and plot on single plot
fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(10, 10), sharey=True)
fig2, axes2 = plt.subplots(nrows=2, ncols=2, figsize=(10, 10), sharey=True)
fig3, axes3 = plt.subplots(nrows=2, ncols=2, figsize=(10, 10), sharey=True)
fig4, axes4 = plt.subplots(nrows=1, ncols=2, figsize=(10, 5))

# ---------------------------------------------------------------------------------
# Chrono Data
# ---------------------------------------------------------------------------------

path_chrono = "../../wheeled_vehicle_models/chrono_reference_data/v8_sedan_withTire/"
dataChrono = pd.read_csv(
    path_chrono + file + "_4wd1e5.csv", sep=',', header='infer')

# ---------------------------------------------------------------------------------
# Plot Chrono first with dashed lines
# ---------------------------------------------------------------------------------

axes[0, 0].plot(dataChrono['time'], dataChrono['lf_tireForce_x'],
                label='Chrono', color='blue', linestyle='dashed')
axes[0, 1].plot(dataChrono['time'], dataChrono['rf_tireForce_x'],
                label='Chrono', color='blue', linestyle='dashed')
axes[1, 0].plot(dataChrono['time'], dataChrono['lr_tireForce_x'],
                label='Chrono', color='blue', linestyle='dashed')
axes[1, 1].plot(dataChrono['time'], dataChrono['rr_tireForce_x'],
                label='Chrono', color='blue', linestyle='dashed')


axes2[0, 0].plot(dataChrono['time'], dataChrono['lf_tireForce_y'],
                 label='Chrono', color='blue', linestyle='dashed')
axes2[0, 1].plot(dataChrono['time'], dataChrono['rf_tireForce_y'],
                 label='Chrono', color='blue', linestyle='dashed')
axes2[1, 0].plot(dataChrono['time'], dataChrono['lr_tireForce_y'],
                 label='Chrono', color='blue', linestyle='dashed')
axes2[1, 1].plot(dataChrono['time'], dataChrono['rr_tireForce_y'],
                 label='Chrono', color='blue', linestyle='dashed')


axes3[0, 0].plot(dataChrono['time'], dataChrono['lf_tireForce_z'],
                 label='Chrono', color='blue', linestyle='dashed')
axes3[0, 1].plot(dataChrono['time'], dataChrono['rf_tireForce_z'],
                 label='Chrono', color='blue', linestyle='dashed')
axes3[1, 0].plot(dataChrono['time'], dataChrono['lr_tireForce_z'],
                 label='Chrono', color='blue', linestyle='dashed')
axes3[1, 1].plot(dataChrono['time'], dataChrono['rr_tireForce_z'],
                 label='Chrono', color='blue', linestyle='dashed')

# Roll and Roll rate
axes4[0].plot(dataChrono['time'], dataChrono['roll'],
              label='Chrono', color='blue', linestyle='dashed')
axes4[1].plot(dataChrono['time'], dataChrono['roll_rate'],
              label='Chrono', color='blue', linestyle='dashed')


# ---------------------------------------------------------------------------------
# Plot 18DOF
# ---------------------------------------------------------------------------------

for k in trial_k:
    for b in trial_b:

        data8 = pd.read_csv(
            path_out + file + "_sedan18HiPy_" + k + "_" + b + ".csv", sep=' ', header='infer')
        # Plot for tire forces
        # Plot tire force of lf
        axes[0, 0].plot(data8['time'], data8['lf_tireForce_x'],
                        label=k + "_" + b)
        axes[0, 0].set_xlabel("Time (s)")
        axes[0, 0].set_ylabel("Tire Force (N)")
        axes[0, 0].set_title("lf_tireForce_x")
        axes[0, 0].legend(loc='upper right')

        # Plot tire force of rf
        axes[0, 1].plot(data8['time'], data8['rf_tireForce_x'],
                        label='rf_tireForce_x')
        axes[0, 1].set_xlabel("Time (s)")
        axes[0, 1].set_title("rf_tireForce_x")

        # Plot tire force of lr
        axes[1, 0].plot(data8['time'], data8['lr_tireForce_x'],
                        label='lr_tireForce_x')
        axes[1, 0].set_xlabel("Time (s)")
        axes[1, 0].set_ylabel("Tire Force (N)")
        axes[1, 0].set_title("lr_tireForce_x")

        # Plot tire force of rr
        axes[1, 1].plot(data8['time'], data8['rr_tireForce_x'],
                        label='rr_tireForce_x')
        axes[1, 1].set_xlabel("Time (s)")
        axes[1, 1].set_title("rr_tireForce_x")

        # Now plot y axis forces in the other figure
        # Plot tire force of lf
        axes2[0, 0].plot(data8['time'], data8['lf_tireForce_y'],
                         label=k + "_" + b)
        axes2[0, 0].set_xlabel("Time (s)")
        axes2[0, 0].set_ylabel("Tire Force (N)")
        axes2[0, 0].set_title("lf_tireForce_y")
        axes2[0, 0].legend(loc='upper right')

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

        # Now plot z axis forces in the other other figure
        # Plot tire force of lf
        axes3[0, 0].plot(data8['time'], data8['lf_tireForce_z'],
                         label=k + "_" + b)
        axes3[0, 0].set_xlabel("Time (s)")
        axes3[0, 0].set_ylabel("Tire Force (N)")
        axes3[0, 0].set_title("lf_tireForce_z")
        axes3[0, 0].legend(loc='upper right')

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

        # plot roll and roll rate
        axes4[0].plot(data8['time'], data8['roll'],
                      label=k + "_" + b)
        axes4[0].set_xlabel("Time (s)")
        axes4[0].set_ylabel("Roll (rad)")
        axes4[0].set_title("Roll")
        axes4[0].legend(loc='upper right')

        axes4[1].plot(data8['time'], data8['roll_rate'],
                      label=k + "_" + b)
        axes4[1].set_xlabel("Time (s)")
        axes4[1].set_ylabel("Roll Rate (rad/s)")
        axes4[1].set_title("Roll Rate")

# Show thall the plots
plt.show()
