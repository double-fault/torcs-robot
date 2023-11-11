import pandas as pd
import csv
import sys
import matplotlib.pyplot as plt

# Read the CSV file into a pandas DataFrame
data = pd.read_csv(sys.argv[1], header=None)

# Extract the x and y values from the DataFrame
x = data.iloc[:, 0]
y = data.iloc[:, 1]
colors = []
for i in range(len(x)):
    colors.append('red')

# Create a scatter plot
fig, ax = plt.subplots()
scatter = ax.scatter(x, y, c=colors)

# Add text on hover using matplotlib's annotate function
tooltip = plt.gca().annotate('', xy=(0, 0), xytext=(20, 20), textcoords='offset points', bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.5))

lines = ax.plot(x, y, '-')

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
plt.plot(x_values, y_values, marker='.', color='g')
plt.plot(xv2, yv2, marker='x')

csv_file = "edges.csv"

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

        if idx % 2 == 0:
            x_values.append(float(row[0]))
            y_values.append(float(row[1]))

# Plot the graph
plt.plot(x_values, y_values, marker='.', color='khaki')

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

        if idx % 2 == 1:
            x_values.append(float(row[0]))
            y_values.append(float(row[1]))

# Plot the graph
plt.plot(x_values, y_values, marker='.', color='khaki')

plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Graph from CSV Data')
plt.grid(True)

# Show the scatter plot
plt.show()
