import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Redefine variables for reproducibility
np.random.seed(42)

# Define the total duration and sampling rate
duration = 10  # total time in seconds
sampling_rate = 100  # number of samples per second
time = np.linspace(0, duration, duration * sampling_rate)

# Define the frequency to achieve oscillations within 10 seconds (1 full cycle within 10 seconds)
frequency = 1 / 10  # 1 cycle in 10 seconds

# Create the inclinometer values using sine wave oscillating between 70 and 120
amplitude = 25  # Amplitude for oscillation (half the range between 70 and 120)
offset = 95  # Offset to center the oscillations around 95 degrees
inclinometer_values = amplitude * \
    np.sin(2 * np.pi * frequency * time) + offset  # Sinusoidal oscillation

# Simulate giro data (raw and Kalman-filtered)
error_percentage_before = 0.10  # 10% error
error_percentage_after = np.random.uniform(0.025, 0.035)  # 2.5% to 3.5% error

# Generate synthetic raw giro data with noise
giro_raw = inclinometer_values + inclinometer_values * \
    error_percentage_before + np.random.normal(0, 3, len(inclinometer_values))

# Apply Kalman filter effect to reduce error
giro_kalman = inclinometer_values + inclinometer_values * \
    error_percentage_after + np.random.normal(0, 1.5, len(inclinometer_values))

# Generate sine wave for fluctuation (simulating different peaks)
sine_wave = 5 * np.sin(np.linspace(0, 4 * np.pi,
                       len(inclinometer_values)))  # Varying peaks

# Add sine wave to the data to create varied peaks and valleys
giro_raw += sine_wave
giro_kalman += sine_wave

# Create DataFrame for data
data = {
    'Time (s)': time,
    # Keep original inclinometer values for clarity
    'Inclinometer (deg)': inclinometer_values,
    'Giro Raw (deg)': giro_raw,
    'Giro Kalman (deg)': giro_kalman
}
df = pd.DataFrame(data)

# Display first 10 rows of the table
print("Data Sample (70 Rows):")
print(df.head(50))

# Plot comparison of inclinometer, raw giro, and Kalman-filtered giro
plt.figure(figsize=(12, 8))
plt.plot(time, inclinometer_values, label='Inclinometer',
         linestyle='-', color='blue')
plt.plot(time, giro_raw, label='Giro Raw', linestyle='--', color='red')
plt.plot(time, giro_kalman, label='Giro DKF', linestyle='-', color='green')
plt.title('Perbandingan Inclinometer, Giro Raw, dan Giro Kalman dengan Puncak Berbeda', fontsize=14)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Angle (deg)', fontsize=12)
plt.legend(fontsize=12)
plt.grid(True)
plt.show()

# Display summary table of average error


def calculate_error(true_values, estimated_values):
    return np.mean(np.abs((estimated_values - true_values) / true_values) * 100)


error_raw = calculate_error(inclinometer_values, giro_raw)
error_kalman = calculate_error(inclinometer_values, giro_kalman)

summary_data = {
    "Kondisi": ["Giro Raw", "Giro Kalman"],
    "Rata-rata Error (%)": [error_raw, error_kalman]
}
summary_df = pd.DataFrame(summary_data)
print("\nTabel Rata-rata Error:")
print(summary_df)


# import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt

# # Redefine variables for reproducibility
# np.random.seed(42)

# # Define the total duration and sampling rate
# duration = 10  # total time in seconds
# sampling_rate = 100  # number of samples per second
# time = np.linspace(0, duration, duration * sampling_rate)

# # Define parameters for the oscillations
# frequency = 1 / 10  # Base frequency for one cycle every 10 seconds
# amplitude_variation = np.random.uniform(15, 30, size=(len(time) // 2))  # Random amplitude variations
# phase_shift = np.random.uniform(0, 2 * np.pi, size=(len(time) // 2))  # Random phase shift

# # Create the inclinometer values with varying amplitude and frequency
# inclinometer_values = np.zeros(len(time))

# # Generate sinusoidal oscillations with changing amplitudes and phases
# for i in range(len(time) // 2):
#     start_idx = i * 2
#     end_idx = (i + 1) * 2
#     # Generate oscillation with random amplitude and phase shift
#     amplitude = amplitude_variation[i]
#     phase = phase_shift[i]
#     inclinometer_values[start_idx:end_idx] = amplitude * np.sin(2 * np.pi * frequency * time[start_idx:end_idx] + phase) + 95

# # Simulate giro data (raw and Kalman-filtered)
# error_percentage_before = 0.10  # 10% error
# error_percentage_after = np.random.uniform(0.025, 0.035)  # 2.5% to 3.5% error

# # Generate synthetic raw giro data with noise
# giro_raw = inclinometer_values + inclinometer_values * error_percentage_before + np.random.normal(0, 3, len(inclinometer_values))

# # Apply Kalman filter effect to reduce error
# giro_kalman = inclinometer_values + inclinometer_values * error_percentage_after + np.random.normal(0, 1.5, len(inclinometer_values))

# # Generate sine wave for fluctuation (simulating different peaks)
# sine_wave = 5 * np.sin(np.linspace(0, 4 * np.pi, len(inclinometer_values)))  # Varying peaks

# # Add sine wave to the data to create varied peaks and valleys
# giro_raw += sine_wave
# giro_kalman += sine_wave

# # Create DataFrame for data
# data = {
#     'Time (s)': time,
#     'Inclinometer (deg)': inclinometer_values,  # Keep original inclinometer values for clarity
#     'Giro Raw (deg)': giro_raw,
#     'Giro Kalman (deg)': giro_kalman
# }
# df = pd.DataFrame(data)

# # Display first 10 rows of the table
# print("Data Sample (10 Rows):")
# print(df.head(10))

# # Plot comparison of inclinometer, raw giro, and Kalman-filtered giro
# plt.figure(figsize=(12, 8))
# plt.plot(time, inclinometer_values, label='Inclinometer', linestyle='-', color='blue')  
# plt.plot(time, giro_raw, label='Giro Raw', linestyle='--', color='red')
# plt.plot(time, giro_kalman, label='Giro DKF', linestyle='-', color='green')

# # Add annotations for every 5 Hz interval (every 0.2 seconds)
# for i in range(0, len(time), int(sampling_rate * 0.2)):  # every 0.2 second (5 Hz)
#     plt.text(time[i], inclinometer_values[i], f'{inclinometer_values[i]:.2f}', color='blue', fontsize=8, ha='center', va='bottom')

# plt.title('Perbandingan Inclinometer, Giro Raw, dan Giro Kalman dengan Puncak Berbeda', fontsize=14)
# plt.xlabel('Time (s)', fontsize=12)
# plt.ylabel('Angle (deg)', fontsize=12)
# plt.legend(fontsize=12)
# plt.grid(True)
# plt.show()

# # Display summary table of average error
# def calculate_error(true_values, estimated_values):
#     return np.mean(np.abs((estimated_values - true_values) / true_values) * 100)

# error_raw = calculate_error(inclinometer_values, giro_raw)
# error_kalman = calculate_error(inclinometer_values, giro_kalman)

# summary_data = {
#     "Kondisi": ["Giro Raw", "Giro Kalman"],
#     "Rata-rata Error (%)": [error_raw, error_kalman]
# }
# summary_df = pd.DataFrame(summary_data)
# print("\nTabel Rata-rata Error:")
# print(summary_df)

# import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt

# # Redefine variables for reproducibility
# np.random.seed(42)

# # Define the total duration and sampling rate
# duration = 10  # total time in seconds
# sampling_rate = 100  # number of samples per second
# time = np.linspace(0, duration, duration * sampling_rate)

# # Define parameters for the oscillations
# frequency = 1 / 1  # Base frequency for one cycle every 1 second (every 1 second for the peak changes)
# amplitude_variation = np.random.uniform(15, 30, size=(len(time) // 2))  # Random amplitude variations
# phase_shift = np.random.uniform(0, 2 * np.pi, size=(len(time) // 2))  # Random phase shift

# # Create the inclinometer values with varying amplitude and frequency
# inclinometer_values = np.zeros(len(time))

# # Generate sinusoidal oscillations with changing amplitudes and phases
# for i in range(len(time) // 2):
#     start_idx = i * 2
#     end_idx = (i + 1) * 2
#     # Generate oscillation with random amplitude and phase shift
#     amplitude = amplitude_variation[i]
#     phase = phase_shift[i]
#     inclinometer_values[start_idx:end_idx] = amplitude * np.sin(2 * np.pi * frequency * time[start_idx:end_idx] + phase) + 95

# # Simulate giro data (raw and Kalman-filtered)
# error_percentage_before = 0.10  # 10% error
# error_percentage_after = np.random.uniform(0.025, 0.035)  # 2.5% to 3.5% error

# # Generate synthetic raw giro data with noise
# giro_raw = inclinometer_values + inclinometer_values * error_percentage_before + np.random.normal(0, 3, len(inclinometer_values))

# # Apply Kalman filter effect to reduce error
# giro_kalman = inclinometer_values + inclinometer_values * error_percentage_after + np.random.normal(0, 1.5, len(inclinometer_values))

# # Generate sine wave for fluctuation (simulating different peaks)
# sine_wave = 5 * np.sin(np.linspace(0, 2 * np.pi, len(inclinometer_values)))  # Varying peaks with lower frequency

# # Add sine wave to the data to create varied peaks and valleys
# giro_raw += sine_wave
# giro_kalman += sine_wave

# # Create DataFrame for data
# data = {
#     'Time (s)': time,
#     'Inclinometer (deg)': inclinometer_values,  # Keep original inclinometer values for clarity
#     'Giro Raw (deg)': giro_raw,
#     'Giro Kalman (deg)': giro_kalman
# }
# df = pd.DataFrame(data)

# # Display first 10 rows of the table
# print("Data Sample (10 Rows):")
# print(df.head(10))

# # Plot comparison of inclinometer, raw giro, and Kalman-filtered giro
# plt.figure(figsize=(12, 8))
# plt.plot(time, inclinometer_values, label='Inclinometer', linestyle='-', color='blue')  
# plt.plot(time, giro_raw, label='Giro Raw', linestyle='--', color='red')
# plt.plot(time, giro_kalman, label='Giro DKF', linestyle='-', color='green')

# # Add annotations for every 5 Hz interval (every 0.2 seconds)
# for i in range(0, len(time), int(sampling_rate * 1)):  # every 1 second (1 Hz)
#     plt.text(time[i], inclinometer_values[i], f'{inclinometer_values[i]:.2f}', color='blue', fontsize=8, ha='center', va='bottom')

# plt.title('Perbandingan Inclinometer, Giro Raw, dan Giro Kalman dengan Puncak Berbeda', fontsize=14)
# plt.xlabel('Time (s)', fontsize=12)
# plt.ylabel('Angle (deg)', fontsize=12)
# plt.legend(fontsize=12)
# plt.grid(True)
# plt.show()

# # Display summary table of average error
# def calculate_error(true_values, estimated_values):
#     return np.mean(np.abs((estimated_values - true_values) / true_values) * 100)

# error_raw = calculate_error(inclinometer_values, giro_raw)
# error_kalman = calculate_error(inclinometer_values, giro_kalman)

# summary_data = {
#     "Kondisi": ["Giro Raw", "Giro Kalman"],
#     "Rata-rata Error (%)": [error_raw, error_kalman]
# }
# summary_df = pd.DataFrame(summary_data)
# print("\nTabel Rata-rata Error:")
# print(summary_df)

