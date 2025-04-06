import csv
import random

# File to save
filename = "Random_positions.csv"

# Number of rows
num_rows = 200

# Open the CSV for writing
with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)

    # Write the header
    writer.writerow(['x', 'y', 'z'])

    # Generate random values
    for i in range(num_rows):
        x = round(random.uniform(-0.85 , 0.85), 2)
        y = round(random.uniform(-0.85 , 0.85), 2)
        z = round(random.uniform(0.1 , 1.2), 2)

        writer.writerow([x, y, z])

print(f"✅ Created '{filename}' with 3 columns and {num_rows} rows.")