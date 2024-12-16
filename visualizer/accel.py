import numpy as np
import matplotlib.pyplot as plt

# Dummy data generation for accelerometer and inclinometer
np.random.seed(42)  # Ensuring reproducibility

time = np.linspace(0, 10, 500)  # Time in seconds

# Simulated inclinometer data (ground truth) in degrees
inclinometer_data = 30 * np.sin(2 * np.pi * 0.1 * time)

# Simulated accelerometer readings (with noise and bias) in degrees
accel_data_uncalibrated = inclinometer_data + 5 + 3 * np.random.normal(size=len(time))  # Bias and noise

# Simulated accelerometer readings after Kalman Filter in degrees
accel_data_kalman = inclinometer_data + 1.5 * np.random.normal(size=len(time))  # Reduced noise

# Plotting
plt.figure(figsize=(12, 8))

# Plot uncalibrated accelerometer data
plt.plot(time, accel_data_uncalibrated, label='Accelerometer (Uncalibrated)', linestyle='--', color='red')

# Plot Kalman-filtered accelerometer data
plt.plot(time, accel_data_kalman, label='Accelerometer (Kalman Filter)', linestyle='-', color='green')

# Plot ground truth inclinometer data
plt.plot(time, inclinometer_data, label='Inclinometer (Ground Truth)', linestyle='-', color='blue')

plt.title('Comparison of Accelerometer Readings Before and After Kalman Filter', fontsize=14)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Angle (°)', fontsize=12)
plt.legend(fontsize=12)
plt.grid(True)
plt.show()