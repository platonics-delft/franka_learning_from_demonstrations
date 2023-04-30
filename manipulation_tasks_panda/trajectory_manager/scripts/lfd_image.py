#%%
#!/usr/bin/env python
import os
import sys
import cv2
import rospy
import math
import numpy as np
import time
import rospkg
from geometry_msgs.msg import PoseStamped, Pose, WrenchStamped
from sensor_msgs.msg import Image
from pynput.keyboard import Listener, KeyCode
from cv_bridge import CvBridge, CvBridgeError
from manipulation_helpers.pose_transform_functions import orientation_2_quaternion, pose_st_2_transformation, position_2_array, array_quat_2_pose, transformation_2_pose, transform_pose, list_2_quaternion
from cv_bridge import CvBridgeError, CvBridge
from lfd import LfD
import tf
import quaternion
import pdb
from camera_feedback import CameraFeedback, image_process
from std_msgs.msg import Float32MultiArray,Float32

class LfD_image(LfD, CameraFeedback):
    def __init__(self):
        rospy.init_node("learning_node")
        super().__init__()
        self.r=rospy.Rate(20)

        # self._tf_listener = tf.TransformListener()
        self.spiraling = False
        
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        self.cropped_img_pub = rospy.Publisher('/modified_img', Image, queue_size=0)
        self.annotated_img_pub = rospy.Publisher('/annotated_img', Image, queue_size=0)
        self.current_template_pub = rospy.Publisher('/current_template', Image, queue_size=0)
        self.corrected_goal_pub = rospy.Publisher('/corrected_goal', PoseStamped, queue_size=0)
        self.pos_2_goal_diff = rospy.Publisher('/pos_2_goal', Float32, queue_size=0)

        self.bridge = CvBridge()
        self._tf_broadcaster = tf.TransformBroadcaster()

        self.curr_image = None
        self.recorded_traj = None
        self.recorded_ori = None
        self.height = 720
        self.width = 1280
        self.width_ds = int(1280/4)
        self.end = False
        self.grip_open_width = 0.02

        # self.row_crop_pct_top = 0.3
        # self.row_crop_pct_bot = 0.9
        # self.col_crop_pct_left = 0.6
        # self.col_crop_pct_right = 0.8
        self.row_bias_pct = (self.row_crop_pct_top + self.row_crop_pct_bot)/2 - 0.5
        self.col_bias_pct = (self.col_crop_pct_left + self.col_crop_pct_right)/2 - 0.5
        # self.ds_factor = 4

        # self.x_dist_threshold = 5
        # self.y_dist_threshold = 5

        self.insertion_force_threshold = 6
        self.retry_counter = 0

        self.slider_sub=rospy.Subscriber("/slider_node/result", Float32MultiArray, self.slider_callback)
        self.screen_yellow_distance=None
        self.screen_green_distance=None
        self.screen_orientation=None

        ros_pack = rospkg.RosPack()
        self._package_path = ros_pack.get_path('trajectory_manager')

        rospy.sleep(1)

    def image_callback(self, msg):
            # Convert the ROS message to a OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            self.curr_image = cv_image

        except CvBridgeError as e:
            print(e)

    def slider_callback(self, msg):
        self.screen_yellow_distance = msg.data[0]
        self.screen_green_distance = msg.data[1]
        self.screen_orientation = msg.data[2]

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
        vel = 0 ##### Change to a more meaningful name like distance? Trigger could be distance_interval or something.
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
            self.recorded_traj = np.c_[self.recorded_traj, self.curr_pos]
            self.recorded_ori  = np.c_[self.recorded_ori, self.curr_ori]
            self.recorded_gripper = np.c_[self.recorded_gripper, self.grip_value]

            resized_img_gray=image_process(self.curr_image, self.ds_factor, self.row_crop_pct_top , self.row_crop_pct_bot,
                                        self.col_crop_pct_left, self.col_crop_pct_right)
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
        self.set_stiffness(100, 100, 100, 5, 5, 5, 0)
        rospy.loginfo("Ending trajectory recording")

    def execute(self, retry_insertion_flag=0):
        self.spiralling_occured = False
        # print("spiral flag", bool(int(spiral_flag)))
        print('entered execute')
        start = PoseStamped()

        quat_start = list_2_quaternion(self.recorded_ori[:, 0])
        start = array_quat_2_pose(self.recorded_traj[:, 0], quat_start)
        
        ns_msg = [-0.0, -0.78775220299602, -0, -2.363247138397349, -0.0, 1.5758730952857454, 0.7762000998565743]
        # self.set_configuration(ns_msg)
        
        # self.set_stiffness(0, 0, 0, 0, 0, 0, 10)
        # rospy.sleep(2)
        # self.set_stiffness(1000, 1000, 1000, 20, 20, 20, 20)    
        self.go_to_pose(start)
        self.set_stiffness(2000, 2000, 2000, 30, 30, 30, 0)

        self.time_index=0
        z_sum = 0

        if self.recorded_gripper[0][0] < self.grip_open_width/2 and self.gripper_width > 0.9 * self.grip_open_width:
            print("closing gripper")
            self.grasp_gripper(self.recorded_gripper[0][self.time_index])
            # self.move_gripper(self.recorded_gripper[0][self.time_index])
            time.sleep(0.1)
        if self.recorded_gripper[0][0] > self.grip_open_width/2:
            print("opening gripper")
            self.move_gripper(self.recorded_gripper[0][self.time_index])
            time.sleep(0.1)

        while self.time_index <( self.recorded_traj.shape[1]):
            quat_goal = list_2_quaternion(self.recorded_ori[:, self.time_index])
            goal = array_quat_2_pose(self.recorded_traj[:, self.time_index], quat_goal)
            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = 'panda_link0'
            ori_threshold = 0.3
            pos_threshold = 0.1
            
            self.correct()

            if (self.recorded_gripper[0][self.time_index]-self.recorded_gripper[0][max(0,self.time_index-1)]) < -self.grip_open_width/2:
                print("closing gripper")
                self.grasp_gripper(self.recorded_gripper[0][self.time_index])
                # self.move_gripper(self.recorded_gripper[0][self.time_index])
                time.sleep(0.1)

            if (self.recorded_gripper[0][self.time_index]-self.recorded_gripper[0][max(0,self.time_index-1)]) > self.grip_open_width/2:
                print("open gripper")
                self.move_gripper(self.recorded_gripper[0][self.time_index])
                time.sleep(0.1)
            self.goal_pub.publish(goal)

            if self.recorded_img_feedback_flag[0, self.time_index] and not self.time_index % 2:
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
            self.pos_2_goal_diff.publish(pos_2_goal_diff)
            if pos_2_goal_diff <= self.attractor_distance_threshold:
                self.time_index=self.time_index + 1

            force_xy_plane = np.sqrt(self.force.x ** 2 + self.force.y ** 2)
            if retry_insertion_flag and force_xy_plane > self.insertion_force_threshold:
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
        if self.spiralling_occured:
            print(f"recording {self.filename}, spiralling occured")
            
    def image_process(self, ds_factor, row_crop_top, row_crop_bottom, col_crop_left, col_crop_right):
        h, w = self.curr_image.shape[:2]

        # Define the new dimensions
        width= int(w/ ds_factor)
        height = int(width * (h / w))

        # Resize the image
        resized_img = cv2.resize(self.curr_image, (width, height), interpolation=cv2.INTER_AREA)
        row_idx_start = int(height * row_crop_top)
        row_idx_end = int(height * row_crop_bottom)
        col_idx_start= int(width * col_crop_left)
        col_idx_end = int(width * col_crop_right)

        resized_img = resized_img[row_idx_start:row_idx_end, col_idx_start:col_idx_end, :]
        self.resized_img_gray = cv2.cvtColor(resized_img, cv2.COLOR_BGR2GRAY)

    def save(self, file='last'):
        np.savez(self._package_path + '/trajectories/' + str(file) + '.npz',
                 traj=self.recorded_traj,
                 ori=self.recorded_ori,
                 grip=self.recorded_gripper,
                 img=self.recorded_img, 
                 img_feedback_flag=self.recorded_img_feedback_flag,
                 spiral_flag=self.recorded_spiral_flag)
    
    def load(self, file='last'):
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
