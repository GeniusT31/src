import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def lowest(array):
    lowest = 10000
    for i in array:
        if i < lowest:
            lowest = i
    
    return lowest

# Replace 'your_file.csv' with the path to your CSV file
On_df = pd.read_csv("./cartesian_impedance_control/Experiment/singAvoidOn.csv", delimiter=',')
Off_df = pd.read_csv("./cartesian_impedance_control/Experiment/singAvoidOff.csv", delimiter=',')


fig, (ax1) = plt.subplots(1, 1, figsize = (12,6))

#fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 6))

# Plot the first graph (Manipulability vs Time) on the first axis
ax1.plot(On_df['Time'], On_df['Manipulability'], label='Singularity Avoidance On', color='green', linewidth=2)
#ax1.axhline(y=On_df['Manipulability'].mean(), linestyle=':', color='green', linewidth=2)
ax1.axhline(y = lowest(On_df['Manipulability']), color = 'g', linewidth = 4)

ax1.plot(Off_df['Time'], Off_df['Manipulability'], label='Singularity Avoidance Off', color='red', linewidth=2)
#ax1.axhline(y=Off_df['Manipulability'].mean(), linestyle=':', color='red', linewidth=2)
ax1.axhline(y = lowest(np.array(Off_df['Manipulability'])), color = 'r', linewidth = 4)



ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Manipulability')
ax1.set_title('Manipulability vs Time')
ax1.legend()
ax1.grid(True)
'''
# Plot the second graph (Error vs Time) on the second axis
ax2.plot(On_df['Time'], On_df['Error'], color='green', linewidth=2)
ax2.plot(Off_df['Time'], Off_df['Error'], color='red', linewidth=2)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Error')
ax2.set_title('Error vs Time')
ax2.grid(True)

ax3.scatter(Off_df['Manipulability'], On_df['Manipulability'] - Off_df['Manipulability'], s=10)
# Fit a linear trendline (1 = degree of the polynomial)
coeffs = np.polyfit(Off_df['Manipulability'], On_df['Manipulability'] - Off_df['Manipulability'], 1)
trendline = np.poly1d(coeffs)
ax3.plot(Off_df['Manipulability'], trendline(Off_df['Manipulability']), color='red')  # Plot the trendline
q1 = Off_df['Manipulability'].quantile(0.1)
ax3.axvline(x=q1, color='red', linestyle='--', linewidth=1, label = 'Lowest 10%')
q2 = Off_df['Manipulability'].quantile(0.50)
ax3.axvline(x=q2, color='red', linewidth=1, label = 'median')

q3 = Off_df['Manipulability'].quantile(0.9)
ax3.axvline(x=q3, color='red', linestyle='--', linewidth=1, label = 'Highest 10%')
ax3.legend()
ax3.set_xlabel('Manipulability without singularity avoidance')
ax3.set_ylabel('Difference in Manipulability (Sing. Avoid on vs Sing. Avoid off)')
ax3.set_title('Difference in Manipulability vs Manipulability without Singularity Avoidance')
ax3.grid(True)
'''

# Adjust layout to avoid overlap
plt.tight_layout()


print(range(300))
print(On_df['Manipulability'].mean())
print(Off_df['Manipulability'].mean())
# Show the plot
plt.show()