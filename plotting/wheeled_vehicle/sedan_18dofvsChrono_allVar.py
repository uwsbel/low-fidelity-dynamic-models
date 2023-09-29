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

# Test name

file = sys.argv[1]
# Path to the vehicle output
path_out = "../../wheeled_vehicle_models/18dof/data/output/"
# Path to chrono output
path_chrono = "../../wheeled_vehicle_models/chrono_reference_data/v8_sedan_withTire/"
# path_chrono = "../../wheeled_vehicle_models/chrono_reference_data/latest_sedan_withTire/"


data8 = pd.read_csv(
    path_out + file + "_sedan18Hi.csv", sep=' ', header='infer')


# Chrono file name
dataChrono = pd.read_csv(
    path_chrono + file + "_4wd5e6.csv", sep=',', header='infer')


# Plotting script
print(data8.columns)

# Drop last 4 columns using -4
data8 = data8.iloc[:, :-5]

# Plot each column against chrono data one by one
for col in data8.columns:
    fig, axes = mpl.subplots(nrows=1, ncols=1, figsize=(10, 10))
    if (col == 'time'):
        continue
    else:
        axes.plot(data8['time'], data8[col], label=col)
        axes.plot(dataChrono['time'], dataChrono[col], label=col + ' Chrono')
        axes.set_ylabel(col)
        axes.set_title(f"{file}")
        axes.legend()
        fig.show()
# Clear the axes


save = int(sys.argv[2])

if (save):
    fig.savefig(f'./images/sedan_chvs18_{file}.eps', format='eps', dpi=3000)
mpl.show()
