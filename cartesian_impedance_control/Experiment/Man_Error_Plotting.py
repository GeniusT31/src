import pandas as pd
import matplotlib.pyplot as plt

# Replace 'your_file.csv' with the path to your CSV file
df = pd.read_csv("/home/anthonyli/franka_ros2_ws/src/cartesian_impedance_control/Experiment/singAvoidOn.csv", delimiter=',')  # or delimiter=',' if it's comma-separated

# Plot Manipulability and Error over Time
plt.figure(figsize=(10, 6))
plt.plot(df['Time'], df['Manipulability'], label='Manipulability', color='blue', linewidth=2)
plt.plot(df['Time'], df['Error'], label='Error', color='red', linewidth=2)

# Add titles and labels
plt.title('Manipulability and Error Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
