from manipulation_helpers.pose_transform_functions import orientation_2_quaternion, pose_st_2_transformation, position_2_array, array_quat_2_pose, transformation_2_pose, transform_pose, list_2_quaternion, transform_pos_ori
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
import tf
import copy
from tf.transformations import euler_from_quaternion

class Transform():
    def __init__(self):
        super(Transform, self).__init__()

        self.equilibrium_pose_sub = rospy.Subscriber('/equilibrium_pose', PoseStamped, self.eq_pose_callback)

        # Home pose of panda_EE frame
        # trans_home_pose = np.array([0.30681743793194804, -0.0005964840257302781, 0.4838100689233732])
        # quat_home_pose = np.quaternion(0.0001316886225132278, 0.9999887362754397, -0.004173250786325315, -0.0005296947552688665)
        # self.home_pose = array_quat_2_pose(trans_home_pose, quat_home_pose)

        # Home pose of panda_hand frame
        trans_home_pose = np.array([0.307, 0.000, 0.586])
        quat_home_pose = np.quaternion(-0.001, 1.000, -0.005, -0.002)
        self.home_pose = array_quat_2_pose(trans_home_pose, quat_home_pose)

        self.home_EE_height = 0.4841658147707774

        self._tf_listener = tf.TransformListener()

    def eq_pose_callback(self, eq_pose):
        self.eq_pose = eq_pose

    def get_transform(self, source_frame, target_frame):
        while True:
            try:
                now = rospy.Time.now()
                self._tf_listener.waitForTransform(source_frame, target_frame, now, rospy.Duration(4.0))
                rp_tr, rp_rt = self._tf_listener.lookupTransform(source_frame, target_frame, now)
                break
            except Exception as e:
                rospy.logwarn(e)
        transform = np.dot(tf.transformations.translation_matrix(rp_tr), tf.transformations.quaternion_matrix(rp_rt))
        return transform
    
    def transform_traj_ori(self, traj, ori, transform):
        transformed_traj = np.empty_like(traj)
        transformed_ori = np.empty_like(ori)
        for i in range(traj.shape[1]):
            transformed_traj[:,i], transformed_ori[:,i] = transform_pos_ori(traj[:, i], ori[:, i], transform)
        return transformed_traj, transformed_ori

    def compute_final_transform(self):
        transform_new = self.get_transform('panda_link0', 'panda_hand')
        home_pose_matrix = pose_st_2_transformation(self.home_pose)
        self.final_transform =  transform_new @ np.linalg.inv(home_pose_matrix)
        print("final transform", self.final_transform)
        return self.final_transform
    
    def compute_final_transform_marker(self):
        self.aruco_old_transform = np.load(self._package_path + '/trajectories/aruco_old.npy')
        self.aruco_new_transform = self.get_transform('panda_link0', 'marker')
        self.final_transform = self.aruco_new_transform @ np.linalg.inv(self.aruco_old_transform)
    
    def transform(self, transformation_pose, pose):
        transform_base_2_cam = self.get_transform('panda_link0', 'camera_color_optical_frame')
        
        # if transform box is not in camera frame, remove the base_2_cam transforms
        transform_box = pose_st_2_transformation(transformation_pose)
        transform = transform_base_2_cam @ transform_box @ np.linalg.inv(transform_base_2_cam)

        print("transforming", transform)
        pose = transform_pose(pose, transform)
        pose_quat = orientation_2_quaternion(pose.pose.orientation)

        # Maintain orientation and only apply 'yaw' (rotation around EE z-axis)
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 0
        new_magnitude = np.sqrt(pose_quat.x * pose_quat.x + pose_quat.y * pose_quat.y)
        pose_quat.x = pose_quat.x / new_magnitude
        pose_quat.y = pose_quat.y / new_magnitude
        pose.pose.orientation.x = pose_quat.x
        pose.pose.orientation.y = pose_quat.y

        pose.pose.position.z=self.home_EE_height  # Maintain same height
        return pose
