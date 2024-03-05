import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
import tf
from panda_ros.pose_transform_functions import orientation_2_quaternion, pose_st_2_transformation, position_2_array, array_quat_2_pose, transformation_2_pose, transform_pose, list_2_quaternion, transform_pos_ori

class Transform():
    def __init__(self):
        super(Transform, self).__init__()
        self._tf_listener = tf.TransformListener()
    def transform_traj_ori(self, traj, ori, transform):
        transformed_traj = np.empty_like(traj)
        transformed_ori = np.empty_like(ori)
        for i in range(traj.shape[1]):
            transformed_traj[:,i], transformed_ori[:,i] = transform_pos_ori(traj[:, i], ori[:, i], transform)
        return transformed_traj, transformed_ori

    def compute_final_transform(self):
        # Home pose of panda_EE frame
        position = rospy.get_param("position")
        orientation = rospy.get_param("orientation")
        trans_home_pose = np.array([position['x'], position['y'], position['z']])
        quat_home_pose = np.quaternion(orientation['w'], orientation['x'], orientation['y'], orientation['z'])
        self.home_pose = array_quat_2_pose(trans_home_pose, quat_home_pose)
        transform_new = pose_st_2_transformation(self.curr_pose)
        home_pose_matrix = pose_st_2_transformation(self.home_pose)
        print('transform_new', transform_new)
        print('home pose', home_pose_matrix)
        self.final_transform =  transform_new @ np.linalg.inv(home_pose_matrix)
        self.final_transform[2,0]=0
        self.final_transform[0,2]=0
        self.final_transform[2,1]=0
        self.final_transform[1,2]=0
        self.final_transform[2,2]=1
        self.final_transform[2,3]=0
        print("final transform", self.final_transform)
        return self.final_transform
    
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
