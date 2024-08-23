#!/usr/bin/env python3

import rospy
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
import numpy as np
import tf2_ros
import tf
import copy
from time import perf_counter

from object_localization.global_registration import GlobalRegistration
from object_localization.srv import SavingPointcloud, SavingPointcloudResponse
from object_localization.srv import GlobalRegistrationLocalizer, GlobalRegistrationLocalizerResponse

SOURCE_FILE = "../data/0_0_0_150.ply"

best_params = {'voxel_size': 0.006967875759741542, 'normal_radius': 0.008678468798304324, 'search_radius': 0.0013600689653366928, 'distance_threshold': 0.8405376811331261, 'max_nn_normal': 96, 'max_nn_fpfh': 64}


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


class GlobalRegistrationService():
    _filtered_pcd: o3d.geometry.PointCloud
    _filter_height: float = 0.075

    def __init__(self):
        rospy.init_node("global_registration_service")
        self._rate = rospy.Rate(10)
        self._global_registration = GlobalRegistration(SOURCE_FILE, "")
        """
        self._global_registration.voxel_size = best_params['voxel_size']
        self._global_registration.normal_radius = best_params['normal_radius']
        self._global_registration.search_radius = best_params['search_radius']
        self._global_registration.distance_threshold = best_params['distance_threshold']
        self._global_registration.max_nn_normal = best_params['max_nn_normal']
        self._global_registration.max_nn_fpfh = best_params['max_nn_fpfh']
        """
        self.establish_ros_connections()

    def establish_ros_connections(self):
        self._pcl_sub = rospy.Subscriber("point_cloud", PointCloud2, self.pointcloud_callback)
        rospy.Service("global_registration_service", GlobalRegistrationLocalizer, self.handle_registration_request)
        rospy.Service("saving_pointcloud", SavingPointcloud, self.handle_saving_pcl_request)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        
        # Subscriber for the input point cloud
        self._pcl_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback)

    def pointcloud_callback(self, msg):
        self._global_registration.set_target(msg)

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

    def pointcloud_callback(self, msg: PointCloud2):
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
        self._filtered_pcd = filtered_pcd.select_by_index(np.where(np.array(filtered_pcd.points)[:,2] > self._filter_height)[0])
        points = np.asarray(self._filtered_pcd.points)
        colors = np.asarray(self._filtered_pcd.colors)

        threshold = 0.4  # Adjust this value to control what "close to black" means

        non_black_mask = np.any(colors > threshold, axis=1)

        filtered_points = points[non_black_mask]
        filtered_colors = colors[non_black_mask]

        self._filtered_pcd = o3d.geometry.PointCloud()
        self._filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
        self._filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)




    def handle_saving_pcl_request(self, request: SavingPointcloud):
        rospy.loginfo("Saving point cloud")
        o3d.io.write_point_cloud(request.file_name.data, self._filtered_pcd)
        response = SavingPointcloudResponse()
        response.success.data = True
        return response


    def handle_registration_request(self, request: GlobalRegistrationLocalizer):
        self._global_registration.source_file = request.template_file_name.data
        self._global_registration.target_pcl = copy.deepcopy(self._filtered_pcd)
        if request.save_pcl.data:
            o3d.io.write_point_cloud(request.file_name.data, self._filtered_pcd)
        t0 = perf_counter()
        self._global_registration.colored_registration()
        t1 = perf_counter()
        tf_matrix = self._global_registration.transformation
        response = GlobalRegistrationLocalizerResponse()
        response.compute_time.data = t1 - t0

        response.pose = Pose()

        position = tf_matrix[0:3, 3]
        quaternion = tf.transformations.quaternion_from_matrix(tf_matrix[0:4, 0:4])
        quaternion = quaternion/np.linalg.norm(quaternion)
        # Publish pose
        response.pose.position.x = position[0]
        response.pose.position.y = position[1]
        response.pose.position.z = position[2]
        response.pose.orientation.w = quaternion[3]
        response.pose.orientation.x = quaternion[0]
        response.pose.orientation.y = quaternion[1]
        response.pose.orientation.z = quaternion[2]
        return response

    def run(self):
        while not rospy.is_shutdown():
            self._rate.sleep()



if __name__ == "__main__":
    global_registration_service = GlobalRegistrationService()
    try:
        global_registration_service.run()
    except rospy.ROSInterruptException:
        pass


