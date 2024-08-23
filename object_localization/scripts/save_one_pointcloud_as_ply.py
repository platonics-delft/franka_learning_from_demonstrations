import rospy
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf2_ros
import struct

FILTER_HEIGHT = 0.075

def float32_array_to_rgb(float_array):
    # Interpret the float32 array as uint32 for bitwise operations
    int_array = float_array.view(np.uint32)
    
    # Extract the RGB components
    r = (int_array >> 16) & 0xFF
    g = (int_array >> 8) & 0xFF
    b = int_array & 0xFF
    
    # Stack them into an array of tuples (r, g, b)
    rgb_array = np.stack((r, g, b), axis=-1)
    
    return rgb_array

class PointCloudSaver:
    def __init__(self):
        rospy.init_node('pointcloud_saver', anonymous=True)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        
        # Subscriber for the input point cloud
        self.subscriber = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.callback)
        
        rospy.spin()

    def transform_point_cloud(self, pcd_o3d, source_frame, target_frame):
    
        # Get the transformation matrix
        try:
            transform = self._tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Transform not available")
            return None
        
        
        # Convert transform to a 4x4 matrix
        trans = transform.transform.translation
        rot = transform.transform.rotation
        transform_matrix = np.array([
            [1-2*(rot.y**2 + rot.z**2), 2*(rot.x*rot.y - rot.z*rot.w), 2*(rot.x*rot.z + rot.y*rot.w), trans.x],
            [2*(rot.x*rot.y + rot.z*rot.w), 1-2*(rot.x**2 + rot.z**2), 2*(rot.y*rot.z - rot.x*rot.w), trans.y],
            [2*(rot.x*rot.z - rot.y*rot.w), 2*(rot.y*rot.z + rot.x*rot.w), 1-2*(rot.x**2 + rot.y**2), trans.z],
            [0, 0, 0, 1]
        ])
        
        # Apply the transformation
        pcd_o3d.transform(transform_matrix)

    def draw_pcl(self, pcd):
        # Create a visualization window
        vis = o3d.visualization.Visualizer()
        vis.create_window()

        # Add the point cloud to the visualization
        vis.add_geometry(pcd)

        # Access the render options and set the point size
        render_option = vis.get_render_option()
        render_option.point_size = 0.7
        vis.run()

        vis.destroy_window()


    def callback(self, msg):
        # Transform Pointcloud2 to frame panda_link0
        # Convert ROS PointCloud2 message to numpy array
        pc_data = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)))
        
        if pc_data.size == 0:
            return

        # Separate points and RGB data
        points = pc_data[:, :3]  # XYZ coordinates
        colors = float32_array_to_rgb(np.array(pc_data[:, 3], dtype=np.float32))/255.0


        # Create an Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        self.transform_point_cloud(pcd, msg.header.frame_id, "panda_link0")

        voxel_size = 0.001
        down_pcd = pcd.voxel_down_sample(voxel_size)

        cl, ind = down_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=5.0)
        filtered_pcd = down_pcd.select_by_index(ind)

        # remove points with z value less than 3cm
        filtered_pcd = filtered_pcd.select_by_index(np.where(np.array(filtered_pcd.points)[:,2] > FILTER_HEIGHT)[0])

        # Save the point cloud as a .ply file
        o3d.io.write_point_cloud("pointcloud_from_ros.ply", filtered_pcd)

        rospy.loginfo("Point cloud saved as pointcloud_from_ros.ply!")
        self.draw_pcl(filtered_pcd)
        rospy.signal_shutdown("Finished processing")



if __name__ == "__main__":
    try:
        PointCloudSaver()
    except rospy.ROSInterruptException:
        pass

