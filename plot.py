import csv
import matplotlib.pyplot as plt
import sys

# Path to the CSV file
csv_file = sys.argv[1]

# Lists to store the x and y coordinates
x_values = []
y_values = []

xv2 = []
yv2 = []

idx = 0

# Read data from the CSV file
with open(csv_file, "r") as file:
    reader = csv.reader(file)
    for row in reader:
        idx += 1

        x_values.append(float(row[0]))
        y_values.append(float(row[1]))

plt.plot(x_values, y_values, marker='.')
plt.plot(xv2, yv2, marker='.')
csv_file = "middle.csv"

# Lists to store the x and y coordinates
x_values = []
y_values = []

xv2 = []
yv2 = []

idx = 0

# Read data from the CSV file
with open(csv_file, "r") as file:
    reader = csv.reader(file)
    for row in reader:
        idx += 1

        x_values.append(float(row[0]))
        y_values.append(float(row[1]))

# Plot the graph
plt.plot(x_values, y_values, marker='.')
plt.plot(xv2, yv2, marker='.')

plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Graph from CSV Data')
plt.grid(True)
plt.show()

