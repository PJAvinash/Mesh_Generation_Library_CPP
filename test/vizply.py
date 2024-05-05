import sys
import open3d as o3d

#pip install open3d if you dont have
def visualize_ply(file_path):
  cloud = o3d.io.read_point_cloud(file_path)
  o3d.visualization.draw_geometries([cloud])


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python visualize_ply.py <file_name>")
    else:
        file_path = sys.argv[1]
        visualize_ply(file_path)
