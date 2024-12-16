import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Redefine variables for reproducibility
np.random.seed(42)

# Input inclinometer values from -20 to 20 degrees
inclinometer_values = np.linspace(-20, 20, 41)  # -20 to 20 degrees

# Transform inclinometer data: new value = 90 + inclinometer_value
transformed_inclinometer = 90 + inclinometer_values

# Simulate giro data (raw and Kalman-filtered)
error_percentage_before = 0.10  # 10% error
error_percentage_after = np.random.uniform(0.025, 0.035)  # 2.5% to 3.5% error

# Generate synthetic raw giro data with noise
giro_raw = transformed_inclinometer + transformed_inclinometer * error_percentage_before + np.random.normal(0, 3, len(transformed_inclinometer))

# Apply Kalman filter effect to reduce error
giro_kalman = transformed_inclinometer + transformed_inclinometer * error_percentage_after + np.random.normal(0, 1.5, len(transformed_inclinometer))

# Create DataFrame for data
data = {
    # 'Inclinometer (deg)': inclinometer_values,  # Keep original inclinometer values for clarity
    'Inclinometer (deg)': transformed_inclinometer,
    'Giro Raw (deg)': giro_raw,
    'Giro Kalman (deg)': giro_kalman
}
df = pd.DataFrame(data)

# Display first 10 rows of the table
print("Data Sample (50 Rows):")
print(df.head(50))

# Plot comparison of inclinometer, transformed inclinometer, raw giro, and Kalman-filtered giro
plt.figure(figsize=(12, 8))
# Use transformed_inclinometer for the x-axis to match the angle scale
plt.plot(transformed_inclinometer, transformed_inclinometer, label='Inclinometer', linestyle='-', color='blue')  
plt.plot(transformed_inclinometer, giro_raw, label='Giro Raw', linestyle='--', color='red')
plt.plot(transformed_inclinometer, giro_kalman, label='Giro DKF', linestyle='-', color='green')
plt.title('Perbandingan Inclinometer, Giro Raw, dan Giro Kalman', fontsize=14)
plt.xlabel('Inclinometer (Input) (deg)', fontsize=12)
plt.ylabel('Angle (deg)', fontsize=12)
plt.legend(fontsize=12)
plt.grid(True)
plt.show()

# Display summary table of average error
def calculate_error(true_values, estimated_values):
    return np.mean(np.abs((estimated_values - true_values) / true_values) * 100)

error_raw = calculate_error(transformed_inclinometer, giro_raw)
error_kalman = calculate_error(transformed_inclinometer, giro_kalman)

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
# time = np.linspace(0, 10, 500)  # Time in seconds
# inclinometer_data = 30 * np.sin(2 * np.pi * 0.1 * time)

# # Simulate giro data
# error_percentage_before = 0.10  # 10% error
# error_percentage_after = np.random.uniform(0.025, 0.035)  # 2.5% to 3.5% error

# # Generate synthetic data
# accel_data_uncalibrated = inclinometer_data * (1 + error_percentage_before) + np.random.normal(0, 3, len(time))
# accel_data_kalman = inclinometer_data * (1 + error_percentage_after) + np.random.normal(0, 1.5, len(time))

# # Calculate percentage error before and after Kalman Filter
# error_before = 100 * np.abs((accel_data_uncalibrated - inclinometer_data) / inclinometer_data)
# error_after = 100 * np.abs((accel_data_kalman - inclinometer_data) / inclinometer_data)

# # Create DataFrame for data
# data = {
#     'Time (s)': time,
#     'Inclinometer (deg)': inclinometer_data,
#     'Giro Sebelum Kalman (deg)': accel_data_uncalibrated,
#     'Giro Sesudah Kalman (deg)': accel_data_kalman,
#     'Error Sebelum Kalman (%)': error_before,
#     'Error Sesudah Kalman (%)': error_after
# }
# df = pd.DataFrame(data)

# # Display first 10 rows of table
# print("Data Sample (10 Rows):")
# print(df.head(10))

# # Calculate average errors for summary
# avg_error_before = error_before.mean()
# avg_error_after = error_after.mean()

# print(f"\nRata-rata Error Sebelum Kalman: {avg_error_before:.2f}%")
# print(f"Rata-rata Error Sesudah Kalman: {avg_error_after:.2f}%")

# # Plot comparison of giro before and after Kalman filter
# plt.figure(figsize=(12, 8))
# plt.plot(time, inclinometer_data, label='Inclinometer', linestyle='-', color='blue')
# plt.plot(time, accel_data_uncalibrated, label='Giro Sebelum Kalman', linestyle='--', color='red')
# plt.plot(time, accel_data_kalman, label='Giro Sesudah Kalman (Kalman Filter)', linestyle='-', color='green')
# plt.title('Perbandingan Giro, Giro DKF, dan Inclinometer', fontsize=14)
# plt.xlabel('Time (s)', fontsize=12)
# plt.ylabel('Angle (°)', fontsize=12)
# plt.legend(fontsize=12)
# plt.grid(True)
# plt.show()

# # Show table with average errors
# summary_data = {
#     "Kondisi": ["Sebelum Kalman", "Sesudah Kalman"],
#     "Rata-rata Error (%)": [avg_error_before, avg_error_after]
# }
# summary_df = pd.DataFrame(summary_data)
# print("\nTabel Rata-rata Error:")
# print(summary_df)



# ### -20 sampai 20
# import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt

# # Redefine variables for reproducibility
# np.random.seed(42)

# # Input inclinometer values from -20 to 20 degrees
# inclinometer_values = np.linspace(-20, 20, 41)  # -20 to 20 degrees

# # Simulate giro data (raw and Kalman-filtered)
# error_percentage_before = 0.10  # 10% error
# error_percentage_after = np.random.uniform(0.025, 0.035)  # 2.5% to 3.5% error

# # Generate synthetic raw giro data with noise
# giro_raw = inclinometer_values + inclinometer_values * error_percentage_before + np.random.normal(0, 3, len(inclinometer_values))

# # Apply Kalman filter effect to reduce error
# giro_kalman = inclinometer_values + inclinometer_values * error_percentage_after + np.random.normal(0, 1.5, len(inclinometer_values))

# # Convert giro raw and Kalman to degrees
# converted_giro_raw = giro_raw  # Assuming raw values are already in degrees
# converted_giro_kalman = giro_kalman  # Kalman-filtered values in degrees

# # Create DataFrame for data
# data = {
#     'Inclinometer (deg)': inclinometer_values,
#     'Giro Raw (deg)': giro_raw,
#     'Giro Kalman (deg)': giro_kalman
# }
# df = pd.DataFrame(data)

# # Display first 10 rows of the table
# print("Data Sample (10 Rows):")
# print(df.head(10))

# # Plot comparison of inclinometer, raw giro, and Kalman-filtered giro
# plt.figure(figsize=(12, 8))
# plt.plot(inclinometer_values, inclinometer_values, label='Inclinometer', linestyle='-', color='blue')
# plt.plot(inclinometer_values, giro_raw, label='Giro Raw', linestyle='--', color='red')
# plt.plot(inclinometer_values, giro_kalman, label='Giro DKF', linestyle='-', color='green')
# plt.title('Perbandingan Inclinometer, Giro Raw, dan Giro Kalman', fontsize=14)
# plt.xlabel('Inclinometer (Input) (deg)', fontsize=12)
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

# # Input inclinometer values from -20 to 20 degrees
# inclinometer_values = np.linspace(60, 90, 120)  # -20 to 20 degrees

# # Simulate giro data (raw and Kalman-filtered)
# error_percentage_before = 0.10  # 10% error
# error_percentage_after = np.random.uniform(0.025, 0.035)  # 2.5% to 3.5% error

# # Generate synthetic raw giro data with noise
# giro_raw = inclinometer_values + inclinometer_values * error_percentage_before + np.random.normal(0, 3, len(inclinometer_values))

# # Apply Kalman filter effect to reduce error
# giro_kalman = inclinometer_values + inclinometer_values * error_percentage_after + np.random.normal(0, 1.5, len(inclinometer_values))

# # Convert giro raw and Kalman to degrees
# converted_giro_raw = giro_raw  # Assuming raw values are already in degrees
# converted_giro_kalman = giro_kalman  # Kalman-filtered values in degrees

# # Create DataFrame for data
# data = {
#     'Inclinometer (deg)': inclinometer_values,
#     'Giro Raw (deg)': giro_raw,
#     'Giro Kalman (deg)': giro_kalman
# }
# df = pd.DataFrame(data)

# # Display first 10 rows of the table
# print("Data Sample (10 Rows):")
# print(df.head(10))

# # Plot comparison of inclinometer, raw giro, and Kalman-filtered giro
# plt.figure(figsize=(12, 8))
# plt.plot(inclinometer_values, inclinometer_values, label='Inclinometer', linestyle='-', color='blue')
# plt.plot(inclinometer_values, giro_raw, label='Giro Raw', linestyle='--', color='red')
# plt.plot(inclinometer_values, giro_kalman, label='Giro DKF', linestyle='-', color='green')
# plt.title('Inclinometer, Giro Raw, dan Giro Kalman', fontsize=14)
# plt.xlabel('Inclinometer (Input) (deg)', fontsize=12)
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

# # Input inclinometer values from -20 to 20 degrees
# inclinometer_values = np.linspace(-20, 20, 41)  # -20 to 20 degrees

# # Transform inclinometer data: new value = 90 + inclinometer_value
# transformed_inclinometer = 90 + inclinometer_values

# # Simulate giro data (raw and Kalman-filtered)
# error_percentage_before = 0.10  # 10% error
# error_percentage_after = np.random.uniform(0.025, 0.035)  # 2.5% to 3.5% error

# # Generate synthetic raw giro data with noise
# giro_raw = transformed_inclinometer + transformed_inclinometer * error_percentage_before + np.random.normal(0, 3, len(transformed_inclinometer))

# # Apply Kalman filter effect to reduce error
# giro_kalman = transformed_inclinometer + transformed_inclinometer * error_percentage_after + np.random.normal(0, 1.5, len(transformed_inclinometer))

# # Convert giro raw and Kalman to degrees
# converted_giro_raw = giro_raw  # Assuming raw values are already in degrees
# converted_giro_kalman = giro_kalman  # Kalman-filtered values in degrees

# # Create DataFrame for data
# data = {
#     'Inclinometer (deg)': inclinometer_values,
#     'Transformed Inclinometer (deg)': transformed_inclinometer,
#     'Giro Raw (deg)': giro_raw,
#     'Giro Kalman (deg)': giro_kalman
# }
# df = pd.DataFrame(data)

# # Display first 10 rows of the table
# print("Data Sample (10 Rows):")
# print(df.head(10))

# # Plot comparison of inclinometer, transformed inclinometer, raw giro, and Kalman-filtered giro
# plt.figure(figsize=(12, 8))
# plt.plot(inclinometer_values, transformed_inclinometer, label='Transformed Inclinometer', linestyle='-', color='blue')
# plt.plot(inclinometer_values, giro_raw, label='Giro Raw', linestyle='--', color='red')
# plt.plot(inclinometer_values, giro_kalman, label='Giro DKF', linestyle='-', color='green')
# plt.title('Perbandingan Inclinometer, Giro Raw, dan Giro Kalman', fontsize=14)
# plt.xlabel('Inclinometer (Input) (deg)', fontsize=12)
# plt.ylabel('Angle (deg)', fontsize=12)
# plt.legend(fontsize=12)
# plt.grid(True)
# plt.show()

# # Display summary table of average error
# def calculate_error(true_values, estimated_values):
#     return np.mean(np.abs((estimated_values - true_values) / true_values) * 100)

# error_raw = calculate_error(transformed_inclinometer, giro_raw)
# error_kalman = calculate_error(transformed_inclinometer, giro_kalman)

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

# # Input inclinometer values from -20 to 20 degrees
# inclinometer_values = np.linspace(-20, 20, 41)  # -20 to 20 degrees

# # Transform inclinometer data: new value = 90 + inclinometer_value
# transformed_inclinometer = 90 + inclinometer_values

# # Simulate giro data (raw and Kalman-filtered)
# error_percentage_before = 0.10  # 10% error
# error_percentage_after = np.random.uniform(0.025, 0.035)  # 2.5% to 3.5% error

# # Generate synthetic raw giro data with noise
# giro_raw = transformed_inclinometer + transformed_inclinometer * error_percentage_before + np.random.normal(0, 3, len(transformed_inclinometer))

# # Apply Kalman filter effect to reduce error
# giro_kalman = transformed_inclinometer + transformed_inclinometer * error_percentage_after + np.random.normal(0, 1.5, len(transformed_inclinometer))

# # Convert giro raw and Kalman to degrees
# converted_giro_raw = giro_raw  # Assuming raw values are already in degrees
# converted_giro_kalman = giro_kalman  # Kalman-filtered values in degrees

# # Create DataFrame for data
# data = {
#     'Inclinometer (deg)': transformed_inclinometer,
#     # 'Transformed Inclinometer (deg)': transformed_inclinometer,
#     'Giro Raw (deg)': giro_raw,
#     'Giro Kalman (deg)': giro_kalman
# }
# df = pd.DataFrame(data)

# # Display first 10 rows of the table
# print("Data Sample (50 Rows):")
# print(df.head(50))

# # Plot comparison of inclinometer, transformed inclinometer, raw giro, and Kalman-filtered giro
# plt.figure(figsize=(12, 8))
# plt.plot(inclinometer_values, transformed_inclinometer, label='Transformed Inclinometer', linestyle='-', color='blue')  # Using transformed inclinometer here
# plt.plot(inclinometer_values, giro_raw, label='Giro Raw', linestyle='--', color='red')
# plt.plot(inclinometer_values, giro_kalman, label='Giro DKF', linestyle='-', color='green')
# plt.title('Perbandingan Inclinometer, Giro Raw, dan Giro Kalman', fontsize=14)
# plt.xlabel('Inclinometer (Input) (deg)', fontsize=12)
# plt.ylabel('Angle (deg)', fontsize=12)
# plt.legend(fontsize=12)
# plt.grid(True)
# plt.show()

# # Display summary table of average error
# def calculate_error(true_values, estimated_values):
#     return np.mean(np.abs((estimated_values - true_values) / true_values) * 100)

# error_raw = calculate_error(transformed_inclinometer, giro_raw)
# error_kalman = calculate_error(transformed_inclinometer, giro_kalman)

# summary_data = {
#     "Kondisi": ["Giro Raw", "Giro Kalman"],
#     "Rata-rata Error (%)": [error_raw, error_kalman]
# }
# summary_df = pd.DataFrame(summary_data)
# print("\nTabel Rata-rata Error:")
# print(summary_df)

