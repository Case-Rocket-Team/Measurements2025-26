import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys

# Usage:
# python visualize.py data.csv

filename = sys.argv[1]

# Load CSV
df = pd.read_csv(filename, on_bad_lines="warn")

# Convert to numpy for convenience
data = df.to_numpy()

# ---------------------------
# Time column
# ---------------------------

time_us = data[:, 0]
time_s = (time_us - time_us[0]) / 1_000_000.0

# ---------------------------
# Accelerometer columns
# ---------------------------

accel_scale = 8.0 / (2 ** 15)

accel_x = data[:, -3] * accel_scale
accel_y = data[:, -2] * accel_scale
accel_z = data[:, -1] * accel_scale

# ---------------------------
# Strain gauge columns
# ---------------------------

strain_scale = 5.0 / 524288.0

# Everything between time and accelerometer columns
strain_raw = data[:, 1:-3]
strain_voltage = strain_raw * strain_scale

num_gauges = strain_voltage.shape[1]

# ---------------------------
# Plot
# ---------------------------

fig, (ax_strain, ax_accel) = plt.subplots(
    2,
    1,
    figsize=(14, 8),
    sharex=True,
    gridspec_kw={"height_ratios": [3, 1]}
)

# Strain gauges
for i in range(num_gauges):
    ax_strain.plot(
        time_s,
        strain_voltage[:, i],
        label=f"gauge{i}",
        linewidth=1
    )

ax_strain.set_ylabel("Voltage (V)")
ax_strain.set_title("Strain Gauges")
ax_strain.grid(True)
ax_strain.legend()

# Accelerometer
ax_accel.plot(time_s, accel_x, label="Accel X")
ax_accel.plot(time_s, accel_y, label="Accel Y")
ax_accel.plot(time_s, accel_z, label="Accel Z")

ax_accel.set_xlabel("Time (s)")
ax_accel.set_ylabel("Acceleration (g)")
ax_accel.set_title("Accelerometer")
ax_accel.grid(True)
ax_accel.legend()

plt.tight_layout()
plt.show()
