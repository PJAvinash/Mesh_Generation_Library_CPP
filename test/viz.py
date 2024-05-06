import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

# Function to read triangles from text file
def read_triangles_from_file(filename):
    triangles = []
    point_set = []
    edge_set = []
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
            edge_set.append([triangle[0],triangle[1]])
            edge_set.append([triangle[1],triangle[2]])
            edge_set.append([triangle[2],triangle[0]])
    return (point_set,triangles,edge_set)
# Read triangles from text file
 



def plot_traingle_points(point_set):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Extract coordinates
    x = [t[0] for t in point_set]
    y = [t[1] for t in point_set]
    z = [t[2] for t in point_set]

    # Plot points
    ax.scatter(x, y, z, c='r', marker='o')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

# plot_traingle_points(points)

def plot_line_segments(segments):
    """
    Plot line segments in 3D space.
    
    Parameters:
        segments (array-like): Array where each row represents a line segment,
            with each row containing the coordinates of the starting and ending
            points of the segment.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for segment in segments:
        ax.plot3D(*zip(*segment), marker='o')
  
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()


def plot_line_segments2(segments):
    """
    Plot line segments in 3D space.
    
    Parameters:
        segments (array-like): Array where each row represents a line segment,
            with each row containing the coordinates of the starting and ending
            points of the segment.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Combine coordinates of all line segments
    all_x = []
    all_y = []
    all_z = []
    for segment in segments:
        start, end = segment
        x_values = [start[0], end[0]]
        y_values = [start[1], end[1]]
        z_values = [start[2], end[2]]
        all_x.extend(x_values)
        all_y.extend(y_values)
        all_z.extend(z_values)

    # Plot all line segments in a single call
    ax.plot3D(all_x, all_y, all_z, marker='o')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()


def visualize_triangles(Traingles):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for triangle in Traingles:
        vertices = np.array(triangle)
        x = vertices[:, 0]
        y = vertices[:, 1]
        z = vertices[:, 2]
        ax.plot_trisurf(x, y, z, linewidth=0.2, antialiased=True)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.title('3D Triangles Visualization')
    plt.show()



if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python viz.py <file_name>")
    else:
        file_path = sys.argv[1]
        points,triangles,edge_set = read_triangles_from_file(file_path)
        plot_line_segments(edge_set)
