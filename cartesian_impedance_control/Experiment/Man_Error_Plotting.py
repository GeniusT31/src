import pandas as pd
import matplotlib.pyplot as plt

# Replace 'your_file.csv' with the path to your CSV file
On_df = pd.read_csv("./cartesian_impedance_control/Experiment/singAvoidOn.csv", delimiter=',')
Off_df = pd.read_csv("./cartesian_impedance_control/Experiment/singAvoidOff.csv", delimiter=',')

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6))

# Plot the first graph (Manipulability vs Time) on the first axis
ax1.plot(On_df['Time'], On_df['Manipulability'], label='On Manipulability', color='green', linewidth=2)
ax1.plot(Off_df['Time'], Off_df['Manipulability'], label='Off Manipulability', color='red', linewidth=2)
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Manipulability')
ax1.set_title('Manipulability vs Time')
ax1.grid(True)

# Plot the second graph (Error vs Time) on the second axis
ax2.plot(On_df['Time'], On_df['Error'], label='On Error', color='green', linewidth=2)
ax2.plot(Off_df['Time'], Off_df['Error'], label='Off Error', color='red', linewidth=2)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Error')
ax2.set_title('Error vs Time')
ax2.grid(True)

# Adjust layout to avoid overlap
plt.tight_layout()

# Show the plot
plt.show()