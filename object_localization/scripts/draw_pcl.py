import sys
import open3d as o3d
import numpy as np

# Load the point cloud
pcd = o3d.io.read_point_cloud(sys.argv[1])

print(f"Point cloud has {len(pcd.points)} points")
# Create a visualization window
vis = o3d.visualization.Visualizer()
vis.create_window()

# Add the point cloud to the visualization
vis.add_geometry(pcd)

# Access the render options and set the point size
render_option = vis.get_render_option()
render_option.point_size = 1.7  # Set the point size (lower value = smaller points)

# Run the visualization
vis.run()

# Destroy the window when done
vis.destroy_window()
