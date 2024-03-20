import pandas as pd
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
    print("usage: python3 accTestPlot <save>")
    exit()

# Test name

file = "acc"
path_lfdm = "../data/LFDM/"
# Path to chrono output
path_chrono = "../data/Chrono/"

# 8dof file name
data11 = pd.read_csv(path_lfdm + file + "_sedan11Hi.csv",
                     sep=' ', header='infer')

data18 = pd.read_csv(path_lfdm + file + "_sedan18Hi.csv",
                     sep=' ', header='infer')

data24 = pd.read_csv(path_lfdm + file + "_sedan24Hi.csv",
                     sep=' ', header='infer')
# Chrono file name
dataChrono = pd.read_csv(path_chrono + "acc_chrono.csv",
                         sep=',', header='infer')


fig, axes = mpl.subplots(nrows=2, ncols=2, figsize=(11, 11))


# Trajectory - Chrono
axes[0, 0].plot(dataChrono['x'], dataChrono['y'], 'r', label='Chrono')
# Trajectory - 8dof
axes[0, 0].plot(data24['x'], data24['y'], 'g', label='24DOF')
axes[0, 0].plot(data18['x'], data18['y'], 'y', label='18DOF')
axes[0, 0].plot(data11['x'], data11['y'], 'b', label='11DOF')

if (file.split('_')[0] == "double"):
    axes[0, 0].set_ylim(-20, dataChrono.loc[dataChrono.shape[0]-1, 'x'])
axes[0, 0].set_xlabel("X (m)")
axes[0, 0].set_ylabel("Y (m)")
axes[0, 0].set_title("Trajectory")
axes[0, 0].legend()


# Vx - Chrono
axes[1, 0].plot(dataChrono['time'], dataChrono['vx'], 'r', label='Chrono')
# Vx - 8dof
axes[1, 0].plot(dataChrono['time'], data24['vx'], 'g', label='24DoF')
axes[1, 0].plot(dataChrono['time'], data18['vx'], 'y', label='18DoF')
axes[1, 0].plot(dataChrono['time'], data11['vx'], 'b', label='11DoF')
axes[1, 0].set_xlabel("Time (s)")
axes[1, 0].set_ylabel("Vx (m/s)")
axes[1, 0].set_title("Longitudinal Velocity")


# Yaw - Chrono
axes[0, 1].plot(dataChrono['time'], dataChrono['yaw'], 'r', label='Chrono')
axes[0, 1].plot(dataChrono['time'], data24['yaw'], 'g', label='24DoF')
# Yaw - 8DoF
axes[0, 1].plot(dataChrono['time'], data18['yaw'], 'y', label='18DoF')
# Yaw - 14DoF
axes[0, 1].plot(dataChrono['time'], data11['yaw'], 'b', label='11DoF')

axes[0, 1].set_xlabel("Time (s)")
axes[0, 1].set_ylabel("Yaw (rad)")
axes[0, 1].set_title("yaw")


# Vy - Chrono
axes[1, 1].plot(dataChrono['time'], dataChrono['vy'], 'r', label='Chrono')
axes[1, 1].plot(dataChrono['time'], data24['vy'], 'g', label='24DoF')
# Vy - 8DoF
axes[1, 1].plot(dataChrono['time'], data18['vy'], 'y', label='18DoF')
axes[1, 1].plot(dataChrono['time'], data11['vy'], 'b', label='11DoF')
axes[1, 1].set_xlabel("Time (s)")
axes[1, 1].set_ylabel("Vy (m/s)")
axes[1, 1].set_title("Lateral Velocity")

fig.tight_layout()
save = int(sys.argv[1])

if (save):
    fig.savefig(
        f'./images/accuracy_acc.png', format='png', dpi=600, facecolor='w')
mpl.show()
