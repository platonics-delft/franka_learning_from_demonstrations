#%%
#!/usr/bin/env python
import cv2
import rospy
import math
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
import tf
from camera_feedback import CameraFeedback, image_process
from std_msgs.msg import Float32
import rospkg
from geometry_msgs.msg import PoseStamped, Pose
from panda_ros import Panda
from feedback import Feedback
from insertion import Insertion
from transfom import Transform 
from panda_ros.pose_transform_functions import position_2_array, pos_quat_2_pose_st, list_2_quaternion
class LfD(Panda, Feedback, Insertion, Transform, CameraFeedback):
    def __init__(self):
        rospy.init_node("learning_node")
        super(LfD, self).__init__()
        self.r=rospy.Rate(20)
        
        self._tf_broadcaster = tf.TransformBroadcaster()

        self.curr_image = None
        self.recorded_traj = None
        self.recorded_ori = None

        # self.height = 720
        # self.width = 1280
        # self.width_ds = int(1280/4)
        # self.row_bias_pct = (self.row_crop_pct_top + self.row_crop_pct_bot)/2 - 0.5
        # self.col_bias_pct = (self.col_crop_pct_left + self.col_crop_pct_right)/2 - 0.5

        self.end = False
        self.grip_open_width = 0.02


        self.insertion_force_threshold = 6
        self.retry_counter = 0

        rospy.sleep(1)

    def traj_rec(self, trigger=0.005, rec_position=True, rec_orientation=True):
        # trigger for starting the recording
        if rec_position: 
            self.set_K.update_configuration({"translational_stiffness_X": 0})
            self.set_K.update_configuration({"translational_stiffness_Y": 0})
            self.set_K.update_configuration({"translational_stiffness_Z": 0})  
        if rec_orientation: 
            self.set_K.update_configuration({"rotational_stiffness_X": 0})
            self.set_K.update_configuration({"rotational_stiffness_Y": 0})
            self.set_K.update_configuration({"rotational_stiffness_Z": 0})

        init_pos = self.curr_pos
        vel = 0 
        print("Move robot to start recording.")
        while vel < trigger:
            vel = math.sqrt((self.curr_pos[0]-init_pos[0])**2 + (self.curr_pos[1]-init_pos[1])**2 + (self.curr_pos[2]-init_pos[2])**2)
        self.recorded_traj = self.curr_pos
        self.recorded_ori = self.curr_ori
        if self.gripper_width < self.grip_open_width * 0.9:
            self.grip_value = 0
        else:
            self.grip_value = self.grip_open_width
        self.recorded_gripper= self.grip_value
        self.recorded_img_feedback_flag = np.array([0])
        self.recorded_spiral_flag = np.array([0])
     
        resized_img_gray=image_process(self.curr_image, self.ds_factor,  self.row_crop_pct_top , self.row_crop_pct_bot,
                                        self.col_crop_pct_left, self.col_crop_pct_right)
        resized_img_msg = self.bridge.cv2_to_imgmsg(resized_img_gray)
        self.cropped_img_pub.publish(resized_img_msg)
        self.recorded_img = resized_img_gray.reshape((1, resized_img_gray.shape[0], resized_img_gray.shape[1]))

        print("Recording started. Press e to stop.")
        while not self.end:
            while(self.pause):
                self.r.sleep()               
            self.recorded_traj = np.c_[self.recorded_traj, self.curr_pos]
            self.recorded_ori  = np.c_[self.recorded_ori, self.curr_ori]
            self.recorded_gripper = np.c_[self.recorded_gripper, self.grip_value]

            resized_img_gray=image_process(self.curr_image, self.ds_factor, self.row_crop_pct_top , self.row_crop_pct_bot,self.col_crop_pct_left, self.col_crop_pct_right)
            resized_img_msg = self.bridge.cv2_to_imgmsg(resized_img_gray)
            self.cropped_img_pub.publish(resized_img_msg)
            self.recorded_img = np.r_[self.recorded_img, resized_img_gray.reshape((1, resized_img_gray.shape[0], resized_img_gray.shape[1]))]
            self.recorded_img_feedback_flag = np.c_[self.recorded_img_feedback_flag, self.img_feedback_flag]
            self.recorded_spiral_flag = np.c_[self.recorded_spiral_flag, self.spiral_flag]

            self.r.sleep()

        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = self.curr_pos[0]
        goal.pose.position.y = self.curr_pos[1]
        goal.pose.position.z = self.curr_pos[2]
        
        goal.pose.orientation.w = self.curr_ori[0]
        goal.pose.orientation.x = self.curr_ori[1]
        goal.pose.orientation.y = self.curr_ori[2]
        goal.pose.orientation.z = self.curr_ori[3]
        self.goal_pub.publish(goal)
        self.set_stiffness(3000, 3000, 3000, 40, 40, 40, 0)
        rospy.loginfo("Ending trajectory recording")

    def execute(self, retry_insertion_flag=0):
        self.spiralling_occured = False
        # print('entered execute')
        start = PoseStamped()

        quat_start = list_2_quaternion(self.recorded_ori[:, 0])
        start = pos_quat_2_pose_st(self.recorded_traj[:, 0], quat_start)
         
        self.go_to_pose(start)
        self.set_stiffness(3000, 3000, 3000, 40, 40, 40, 0)

        self.time_index=0

        if self.recorded_gripper[0][0] < self.grip_open_width/2 and self.gripper_width > 0.9 * self.grip_open_width:
            print("closing gripper")
            self.grasp_gripper(self.recorded_gripper[0][self.time_index])
            time.sleep(0.1)
        if self.recorded_gripper[0][0] > self.grip_open_width/2:
            print("opening gripper")
            self.move_gripper(self.recorded_gripper[0][self.time_index])
            time.sleep(0.1)


        while self.time_index <( self.recorded_traj.shape[1]):
            quat_goal = list_2_quaternion(self.recorded_ori[:, self.time_index])
            goal = pos_quat_2_pose_st(self.recorded_traj[:, self.time_index] + self.camera_correction, quat_goal)
            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = 'panda_link0'
            ori_threshold = 0.3
            pos_threshold = 0.1
            
            self.correct()

            if (self.recorded_gripper[0][self.time_index]-self.recorded_gripper[0][max(0,self.time_index-1)]) < -self.grip_open_width/2:
                # print("closing gripper")
                self.grasp_gripper(self.recorded_gripper[0][self.time_index])
                time.sleep(0.1)

            if (self.recorded_gripper[0][self.time_index]-self.recorded_gripper[0][max(0,self.time_index-1)]) > self.grip_open_width/2:
                # print("open gripper")
                self.move_gripper(self.recorded_gripper[0][self.time_index])
                time.sleep(0.1)
            self.goal_pub.publish(goal)

            if self.recorded_img_feedback_flag[0, self.time_index]:
                self.sift_matching()
            
            if self.recorded_spiral_flag[0, self.time_index]:
                if self.force.z > 5:
                    spiral_success, offset_correction = self.spiral_search(goal)
                    self.spiralling_occured = True
                    if spiral_success:
                        self.recorded_traj[0, self.time_index:] += offset_correction[0]
                        self.recorded_traj[1, self.time_index:] += offset_correction[1]

            goal_pos_array = position_2_array(goal.pose.position)
            pos_2_goal_diff = np.linalg.norm(self.curr_pos-goal_pos_array)
  
            if pos_2_goal_diff <= self.attractor_distance_threshold:
                self.time_index=self.time_index + 1

            force_xy_plane = np.sqrt(self.force.x ** 2 + self.force.y ** 2)
            if retry_insertion_flag and force_xy_plane > self.insertion_force_threshold:
                # print("Camera correction", self.camera_correction)
                if self.retry_counter >= 3:
                    self.move_gripper(self.grip_open_width)
                    break
                self.go_to_pose(start)
                self.time_index = 0
                self.retry_counter = self.retry_counter + 1
            self.r.sleep()
            # Stop playback if at end of trajectory (some indices might be deleted by feedback)
            if self.time_index == self.recorded_traj.shape[1]-1:
                break

    def save(self, file='last'):
        ros_pack = rospkg.RosPack()
        self._package_path = ros_pack.get_path('trajectory_data')
        np.savez(self._package_path + '/trajectories/' + str(file) + '.npz',
                 traj=self.recorded_traj,
                 ori=self.recorded_ori,
                 grip=self.recorded_gripper,
                 img=self.recorded_img, 
                 img_feedback_flag=self.recorded_img_feedback_flag,
                 spiral_flag=self.recorded_spiral_flag)
    
    def load(self, file='last'):
        ros_pack = rospkg.RosPack()
        self._package_path = ros_pack.get_path('trajectory_data')
        data = np.load(self._package_path + '/trajectories/' + str(file) + '.npz')
        self.recorded_traj = data['traj']
        self.recorded_ori = data['ori']
        self.recorded_gripper = data['grip']
        self.recorded_img = data['img']
        self.recorded_img_feedback_flag = data['img_feedback_flag']
        self.recorded_spiral_flag = data['spiral_flag']
        if self.final_transform is not None:
            self.recorded_traj, self.recorded_ori = self.transform_traj_ori(self.recorded_traj, self.recorded_ori, self.final_transform)
        
        self.filename=str(file)
