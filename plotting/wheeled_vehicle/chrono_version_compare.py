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
    print("usage: python3 hmmwv_Chrono latestvsChrono.py <file> <save>")
    exit()

# Test name

file = sys.argv[1]
# Path to the vehicle output
path_chrono_latest = "../../wheeled_vehicle_models/chrono_reference_data/latest_sedan_withTire/"
# Path to chrono output
path_chrono_8 = "../../wheeled_vehicle_models/chrono_reference_data/v8_sedan_withTire/"

# 8dof file name
data8 = pd.read_csv(
    path_chrono_latest + file + "_4wd5e6.csv", sep=',', header='infer')
print(data8.shape)
dataChrono = pd.read_csv(
    path_chrono_8 + file + "_4wd5e6.csv", sep=',', header='infer')

# print dataChrono columnes

fig, axes = mpl.subplots(nrows=2, ncols=2, figsize=(10, 10))


# Trajectory - Chrono
axes[0, 0].plot(dataChrono['x'], dataChrono['y'], 'r', label='Chrono 8.0.0')
# Trajectory - 8dof
axes[0, 0].plot(data8['x'], data8['y'], 'y', label='Chrono latest')
if (file.split('_')[0] == "double"):
    axes[0, 0].set_ylim(-20, dataChrono.loc[dataChrono.shape[0]-1, 'x'])
axes[0, 0].set_xlabel("X (m)")
axes[0, 0].set_ylabel("Y (m)")
axes[0, 0].set_title("Trajectory")
axes[0, 0].legend()


# Vx - Chrono
axes[1, 0].plot(dataChrono['time'], dataChrono['vx'],
                'r', label='Chrono 8.0.0')
# Vx - 8dof
axes[1, 0].plot(dataChrono['time'], data8['vx'], 'y', label='Chrono latest')
axes[1, 0].set_xlabel("Time (s)")
axes[1, 0].set_ylabel("Vx (m/s)")
axes[1, 0].set_title("Longitudinal Velocity")


# Yaw - Chrono
axes[0, 1].plot(dataChrono['time'], dataChrono['yaw'],
                'r', label='Chrono 8.0.0')
# Yaw - 8dof
axes[0, 1].plot(dataChrono['time'], data8['yaw'], 'y', label='Chrono latest')
# Yaw - 14dof

axes[0, 1].set_xlabel("Time (s)")
axes[0, 1].set_ylabel("Yaw (rad)")
axes[0, 1].set_title("yaw")


# Vy - Chrono
axes[1, 1].plot(dataChrono['time'], dataChrono['vy'],
                'r', label='Chrono 8.0.0')
# Vy - 8dof
axes[1, 1].plot(dataChrono['time'], data8['vy'], 'y', label='Chrono latest')
axes[1, 1].set_xlabel("Time (s)")
axes[1, 1].set_ylabel("Vy (m/s)")
axes[1, 1].set_title("Lateral Velocity")


fig.suptitle(f"{file}")

fig.tight_layout()
save = int(sys.argv[2])
path_out = "../../wheeled_vehicle_models/18dof/data/output/image/"

if (save):
    fig.savefig(path_out + file + '_chronoComp.png', dpi=300)
mpl.show()
print(path_out)
