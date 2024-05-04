import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Function to read triangles from text file
def read_triangles_from_file(filename):
    triangles = []
    point_set = []
    with open(filename, 'r') as file:
        for line in file:
            # Split each line by '[' and ']' to extract coordinates
            points = line.strip().split(']')
            triangle = []
            for point in points[0:3]:
                point = point.strip('[')
                coords = point.split(',')
                x = float(coords[0].strip())
                y = float(coords[1].strip())
                z = float(coords[2].strip())
                triangle.append([x,y,z])
                point_set.append([x,y,z])
            triangles.append(triangle)
    return (point_set,triangles)
# Read triangles from text file
 
points,triangles = read_triangles_from_file('MCSphere.txt')
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Extract coordinates
x = [t[0] for t in points]
y = [t[1] for t in points]
z = [t[2] for t in points]

# Plot points
ax.scatter(x, y, z, c='r', marker='o')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
