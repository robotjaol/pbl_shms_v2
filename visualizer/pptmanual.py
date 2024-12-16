import pandas as pd
import matplotlib.pyplot as plt

# Data yang diberikan
data = {
    'Inclinometer (deg)': [70.0, 71.0, 72.0, 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0, 81.0, 82.0, 83.0, 84.0, 85.0, 86.0, 87.0, 88.0, 89.0, 90.0, 91.0, 92.0, 93.0, 94.0, 95.0, 96.0, 97.0, 98.0, 99.0, 100.0, 101.0, 102.0, 103.0, 104.0, 105.0, 106.0, 107.0, 108.0, 109.0, 110.0],
    'Giro Raw (deg)': [73.664360, 79.056707, 80.037124, 83.331546, 79.657366, 80.924491, 81.885860, 81.927751, 77.962353, 89.751109, 90.449335, 84.528372, 88.915862, 89.072779, 90.289969, 87.081138, 92.711575, 97.493161, 104.478464, 99.082699, 99.366657, 98.553693, 99.399238, 105.142319, 104.273102, 102.593321, 102.535343, 106.214734, 106.199054, 108.883416, 109.311649, 112.268047, 108.404643, 116.575977, 122.734939, 119.080919, 117.255915, 120.345283, 115.772744, 115.150117, 123.321101],
    'Giro Kalman (deg)': [71.204966, 71.020906, 72.748782, 73.402586, 76.328803, 78.029089, 79.516273, 80.554894, 81.374638, 80.960138, 81.364416, 81.066148, 86.006593, 85.119270, 85.799039, 89.212934, 87.124793, 90.753043, 90.974437, 90.001596, 92.473380, 95.075277, 95.837970, 97.916474, 97.209340, 102.789257, 97.378972, 99.190346, 100.725753, 99.717669, 104.438278, 105.258584, 104.960531, 105.159152, 104.746941, 106.834380, 110.162579, 109.757516, 110.464200, 112.885421, 114.899274]
}

# Membuat DataFrame
df = pd.DataFrame(data)

# Plot data
plt.figure(figsize=(12, 8))
plt.plot(df.index, df['Inclinometer (deg)'], label='Inclinometer', color='blue', linestyle='-', marker='o')
plt.plot(df.index, df['Giro Raw (deg)'], label='Giro Raw', color='red', linestyle='--', marker='x')
plt.plot(df.index, df['Giro Kalman (deg)'], label='Giro Kalman', color='green', linestyle='-', marker='^')

# Menambahkan label dan judul
plt.title('Perbandingan Inclinometer, Giro Raw, dan Giro Kalman', fontsize=14)
plt.xlabel('Index', fontsize=12)
plt.ylabel('Angle (deg)', fontsize=12)
plt.legend(fontsize=12)
plt.grid(True)

# Menampilkan plot
plt.show()
