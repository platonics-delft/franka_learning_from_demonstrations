import sys
import open3d as o3d
import numpy as np

def transform_point_cloud(input_file, output_file):
    # Load the point cloud
    pcd = o3d.io.read_point_cloud(input_file)

    # Define a transformation matrix (4x4) for translation
    translation = np.array([0.1, 0.0, 0.0])  # Example: translate 1 unit along the x-axis
    transform_matrix = np.eye(4)
    transform_matrix[:3, 3] = translation
    transform_matrix = np.array([
        [ 0.83924644,  0.01006041, -0.54390867,  0.064639961],
        [-0.15102344,  0.96521988, -0.21491604,  0.075166079],
        [ 0.52191123,  0.2616952 ,  0.81146378, -0.0150303533],
        [ 0.        ,  0.        ,  0.        ,  1.        ],
    ])

    # Random transformation matrix around z axis, not around x and y
    # That means that most part of the ratotion matrix is 0 or 1
    theta = 0.3
    transform_matrix = np.array([
        [np.cos(theta), -np.sin(theta), 0, 0.02],
        [np.sin(theta), np.cos(theta), 0, 0.04],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    print(f"Transforming point cloud with matrix:\n{transform_matrix}")

    # Apply the transformation to the point cloud
    pcd.transform(transform_matrix)

    # Save the transformed point cloud
    o3d.io.write_point_cloud(output_file, pcd)
    print(f"Transformed point cloud saved to {output_file}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python transform_point_cloud.py <input_file> <output_file>")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]

    transform_point_cloud(input_file, output_file)

