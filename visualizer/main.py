import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

np.random.seed(42)
time = np.linspace(0, 10, 500)  # Time in seconds
inclinometer_data = 30 * np.sin(2 * np.pi * 0.1 * time)
accel_data_uncalibrated = inclinometer_data + 5 + 3 * np.random.normal(size=len(time))
accel_data_kalman = inclinometer_data + 1.5 * np.random.normal(size=len(time))

# Creating a DataFrame with giro before (uncalibrated), after (Kalman filter), and inclinometer data
data = {
    'Time (s)': time,
    'Giro Sebelum Kalman': accel_data_uncalibrated,
    'Giro Sesudah Kalman': accel_data_kalman,
    'Inclinometer': inclinometer_data
}
df = pd.DataFrame(data)

# Displaying the first 10 rows of the table
df_head = df.head(10)

# Plot comparison of giro before and after Kalman filter
plt.figure(figsize=(12, 8))
plt.plot(time, accel_data_uncalibrated, label='Giro', linestyle='--', color='red')
plt.plot(time, accel_data_kalman, label='Giro Kalman Filter', linestyle='-', color='green')
plt.plot(time, inclinometer_data, label='Inclinometer', linestyle='-', color='blue')
plt.title('Comparison Giro, Giro DKF, & Inclinometer', fontsize=14)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Angle (°)', fontsize=12)
plt.legend(fontsize=12)
plt.grid(True)
plt.show()

df_head
