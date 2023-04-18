from manipulation_helpers.pose_transform_functions import orientation_2_quaternion, pose_st_2_transformation, position_2_array, array_quat_2_pose, transformation_2_pose, transform_pose, list_2_quaternion
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
import tf
import copy
class Transform():
    def __init__(self):
        super(Transform, self).__init__()
        trans_base_2_hand = np.array([0.308, -0.000, 0.588])

        rot_base_2_hand = list_2_quaternion([-0.002, 1.000, 0.006, -0.002]) 
        self.pose_base_2_hand = array_quat_2_pose(trans_base_2_hand, rot_base_2_hand)
        self.transform_base_2_hand = pose_st_2_transformation(self.pose_base_2_hand)

        trans_hand_2_cam = np.array([0.05073875796492183, -0.03418064441841842, 0.033397])
        rot_hand_2_cam = list_2_quaternion([0.7140855447, 0.005087158, -0.00459640, 0.70002409]) 
        self.pose_hand_2_cam = array_quat_2_pose(trans_hand_2_cam, rot_hand_2_cam)
        self.transform_hand_2_cam = pose_st_2_transformation(self.pose_hand_2_cam)

        self.transform_box_sub = rospy.Subscriber("/box_transform", PoseStamped, self.transform_icp_callback)
        self._tf_listener = tf.TransformListener()

   
    def transform_icp_callback(self, pose_icp):
        self.pose_icp = pose_icp
        # self.pose_icp.pose.position.z = 0
     
    def get_transform(self, source_frame, target_frame):
        while True:
            try:
                rp_tr, rp_rt = self._tf_listener.lookupTransform(source_frame, target_frame, rospy.Time.now() - rospy.Duration(1))
                break
            except Exception as e:
                rospy.logwarn(e)
        transform = np.dot(tf.transformations.translation_matrix(rp_tr), tf.transformations.quaternion_matrix(rp_rt))
        return transform
    
    def transform(self, transformation_pose, pose):
        # transform_base_2_cam = self.transform_base_2_hand @ self.transform_hand_2_cam
        # while True:
        #     try:
        #         rp_tr, rp_rt = self._tf_listener.lookupTransform('panda_link0', 'camera_color_optical_frame', rospy.Time.now() - rospy.Duration(1))
        #         break
        #     except Exception as e:
        #         rospy.logwarn(e)
        transform_base_2_cam = self.get_transform('panda_link0', 'camera_color_optical_frame')
        
        # if transform box is not in camera frame, remove the base_2_cam transforms
        transform_box = pose_st_2_transformation(transformation_pose)
        transform = transform_base_2_cam @ transform_box @ np.linalg.inv(transform_base_2_cam)
        # transform[0,2]=0
        # transform[1,2]=0
        # transform[2,0]=0
        # transform[2,1]=0
        # transform[2,2]=1
        # transform[2,3]=pose.pose.position.z
        pose_old=copy.deepcopy(pose)
        print("transforming", transform)
        pose = transform_pose(pose, transform)
        pose.pose.position.z=pose_old.pose.position.z
        return pose
