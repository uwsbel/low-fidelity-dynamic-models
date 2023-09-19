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
if (len(sys.argv) != 3):
    print("usage: python3 sedan_18dofvs18dofSundialsvsChrono.py <file> <save>")
    exit()

# Test name

file = sys.argv[1]
# Path to the vehicle output
path_out = "../../wheeled_vehicle_models/18dof/data/output/"
# Path to chrono output
path_chrono = "../../wheeled_vehicle_models/chrono_reference_data/v8_sedan/"

# 8dof file name
dataSun = pd.read_csv(
    path_out + file + "_sedan18Sundials.csv", sep=' ', header=None)

data8 = pd.read_csv(
    path_out + file + "_sedan18Hi.csv", sep=',', header='infer')

# Chrono file name

dataChrono = pd.read_csv(
    path_chrono + file + ".csv", sep=',', header='infer')

# print dataChrono columnes
print(path_chrono + file + ".csv")
# Plotting script


fig, axes = mpl.subplots(nrows=2, ncols=2, figsize=(10, 10))


# Trajectory - Chrono
axes[0, 0].plot(dataChrono['x'], dataChrono['y'], 'r', label='Chrono')
# Trajectory - 8dof
axes[0, 0].plot(data8['x'], data8['y'], 'y', label='18DOF')
axes[0, 0].plot(dataSun.iloc[:, 1], dataSun.iloc[:, 2], 'b', label='18DOF Sun')

if (file.split('_')[0] == "double"):
    axes[0, 0].set_ylim(-20, dataChrono.loc[dataChrono.shape[0]-1, 'x'])
axes[0, 0].set_xlabel("X (m)")
axes[0, 0].set_ylabel("Y (m)")
axes[0, 0].set_title("Trajectory")
axes[0, 0].legend()


# Vx - Chrono
axes[1, 0].plot(dataChrono['time'], dataChrono['vx'], 'r', label='Chrono')
# Vx - 8dof
axes[1, 0].plot(dataChrono['time'], data8['vx'], 'y', label='18DOF')
axes[1, 0].plot(dataChrono['time'], dataSun.iloc[:, 3], 'b', label='18DOF Sun')
axes[1, 0].set_xlabel("Time (s)")
axes[1, 0].set_ylabel("Vx (m/s)")
axes[1, 0].set_title("Longitudinal Velocity")


# Yaw - Chrono
axes[0, 1].plot(dataChrono['time'], dataChrono['yaw'], 'r', label='Chrono')
# Yaw - 8dof
axes[0, 1].plot(dataChrono['time'], data8['yaw'], 'y', label='18DOF')
# Yaw - 14dof
axes[0, 1].plot(dataChrono['time'], dataSun.iloc[:, 5], 'b', label='18DOF Sun')

axes[0, 1].set_xlabel("Time (s)")
axes[0, 1].set_ylabel("Yaw (rad)")
axes[0, 1].set_title("yaw")


# Vy - Chrono
axes[1, 1].plot(dataChrono['time'], dataChrono['vy'], 'r', label='Chrono')
# Vy - 8dof
axes[1, 1].plot(dataChrono['time'], data8['vy'], 'y', label='18DOF')
axes[1, 1].plot(dataChrono['time'], dataSun.iloc[:, 4], 'b', label='18DOF Sun')
axes[1, 1].set_xlabel("Time (s)")
axes[1, 1].set_ylabel("Vy (m/s)")
axes[1, 1].set_title("Lateral Velocity")


fig.suptitle(f"{file}")

fig.tight_layout()
save = int(sys.argv[2])

if (save):
    fig.savefig(f'./images/sedan_chvs18_{file}.eps', format='eps', dpi=3000)
mpl.show()
