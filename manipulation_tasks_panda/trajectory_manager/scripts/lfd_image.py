#%%
#!/usr/bin/env python
import os
import sys
import torch
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
from visual_servoing import max_correlation, template_matching, template_matching_SSD
import tf
import quaternion
import pdb

width = int(1280/4)
def image_process(image, ds_factor, row_crop_top, row_crop_bottom, col_crop_left, col_crop_right):
    h, w = image.shape[:2]

    # Define the new dimensions
    width= int(w/ ds_factor)
    height = int(width * (h / w))

    # Resize the image
    resized_img = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    row_idx_start = int(height * row_crop_top)
    row_idx_end = int(height * row_crop_bottom)
    col_idx_start= int(width * col_crop_left)
    col_idx_end = int(width * col_crop_right)

    resized_img = resized_img[row_idx_start:row_idx_end, col_idx_start:col_idx_end, :]
    resized_img_gray = cv2.cvtColor(resized_img, cv2.COLOR_BGR2GRAY)
    return resized_img_gray

class LfD_image(LfD):
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
        self.bridge = CvBridge()
        self._tf_broadcaster = tf.TransformBroadcaster()

        self.curr_image = None
        self.recorded_traj = None
        self.recorded_ori = None
        self.height = 720
        self.width = 1280
        self.width_ds = int(1280/4)
        self.end = False

        self.row_crop_pct_top = 0.3
        self.row_crop_pct_bot = 0.9
        self.col_crop_pct_left = 0.6
        self.col_crop_pct_right = 0.8
        self.row_bias_pct = (self.row_crop_pct_top + self.row_crop_pct_bot)/2 - 0.5
        self.col_bias_pct = (self.col_crop_pct_left + self.col_crop_pct_right)/2 - 0.5
        self.ds_factor = 4

        self.x_dist_threshold = 2
        self.y_dist_threshold = 2

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
        if self.gripper_width < 0.03:
            self.grip_value = 0
        else:
            self.grip_value = 0.04
        self.recorded_gripper= self.grip_value
        self.recorded_gripper= self.grip_value
     
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
        # self.set_stiffness(100, 100, 100, 5, 5, 5, 0)
        rospy.loginfo("Ending trajectory recording")
        # self.recorded_traj = savgol_filter(self.recorded_traj, 51, 3)
        # self.recorded_ori = savgol_filter(self.recorded_ori, 51, 3)

    def execute(self, spiral_flag):
        self.spiralling_occured = False
        # self.pose_icp = None 
        print("spiral flag", bool(int(spiral_flag)))
        print('entered execute')
        start = PoseStamped()

        quat_start = list_2_quaternion(self.recorded_ori[:, 0])
        start = array_quat_2_pose(self.recorded_traj[:, 0], quat_start)
        
        if self.pose_icp:
            start = self.transform(start)
            
        self.go_to_pose(start)
        self.set_stiffness(2000, 2000, 2000, 30, 30, 30, 0)

        i=0
        z_sum = 0

        while i <( self.recorded_traj.shape[1]):
            quat_goal = list_2_quaternion(self.recorded_ori[:, i])
            goal = array_quat_2_pose(self.recorded_traj[:, i], quat_goal)
            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = 'panda_link0'
            ori_threshold = 0.3
            pos_threshold = 0.1
            
            if self.pose_icp:
                goal = self.transform(goal)
            
            self.correct()

            if (self.recorded_gripper[0][i]-self.recorded_gripper[0][max(0,i-1)]) < -0.02:
                print("closing gripper")
                self.grasp_gripper(self.recorded_gripper[0][i])
                time.sleep(0.1)

            if (self.recorded_gripper[0][i]-self.recorded_gripper[0][max(0,i-1)]) >0.02:
                print("open gripper")
                self.move_gripper(self.recorded_gripper[0][i])
                time.sleep(0.1)

            resized_img_gray = image_process(self.curr_image, self.ds_factor,  0, 1, 0, 1)

            idx=np.argmin(np.linalg.norm(self.recorded_traj-(self.curr_pos).reshape(3,1),axis=0))
            recorded_image_msg = self.bridge.cv2_to_imgmsg(self.recorded_img[idx])

            self.current_template_pub.publish(recorded_image_msg)  
            res = cv2.matchTemplate(resized_img_gray, self.recorded_img[idx], cv2.TM_SQDIFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            top_left_xy = min_loc
            x_rect = int(top_left_xy[0])
            y_rect = int(top_left_xy[1])

            w_rect = int(self.recorded_img_tensor[idx].shape[1])
            h_rect = int(self.recorded_img_tensor[idx].shape[0])
            cv2.rectangle(resized_img_gray, (x_rect, y_rect), (x_rect + w_rect, y_rect + h_rect), (0, 255, 0), 2)
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.8
            font_color = (0, 0, 255)
            thickness = 2            
            cv2.putText(resized_img_gray, str(min_val), (x_rect, int(y_rect+h_rect/2)), font, font_scale, font_color, thickness)
            cv2.putText(resized_img_gray, str(max_val), (x_rect, int(y_rect+h_rect/2+20)), font, font_scale, font_color, thickness)             
            
            original_center_x = (top_left_xy[0] + self.recorded_img[idx].shape[1] / 2) - self.col_bias_pct * resized_img_gray.shape[1]
            original_center_y = (top_left_xy[1] + self.recorded_img[idx].shape[0] / 2) - self.row_bias_pct * resized_img_gray.shape[0]

            x_distance = -resized_img_gray.shape[1] / 2 + original_center_x
            y_distance = -resized_img_gray.shape[0] / 2 + original_center_y

            resized_img_gray_msg = self.bridge.cv2_to_imgmsg(resized_img_gray)

            self.annotated_img_pub.publish(resized_img_gray_msg)

            transform_base_2_cam = self.get_transform('panda_link0', 'camera_color_optical_frame')
            transform_correction = np.identity(4)
            correction_increment = 0.0004
            if abs(x_distance) > self.x_dist_threshold:
                transform_correction[0, 3] = np.sign(x_distance) * correction_increment
            if abs(y_distance) > self.y_dist_threshold:
                transform_correction[1, 3] = np.sign(y_distance) * correction_increment
            transform = transform_base_2_cam @ transform_correction @ np.linalg.inv(transform_base_2_cam)
            transform[2,3] = 0   # ignore z translation (in final transform/pose in base frame)

            corrected_pos = position_2_array(goal.pose.position) + transform[:3,3]
            corrected_quat = orientation_2_quaternion(goal.pose.orientation)
            corrected_goal = array_quat_2_pose(corrected_pos, corrected_quat)
            transform_rot = tuple([corrected_quat.x, corrected_quat.y, corrected_quat.z, corrected_quat.w])

            self._tf_broadcaster.sendTransform(tuple(corrected_pos), transform_rot, rospy.Time.now(), 'new_goal', 'panda_link0')

            corrected_goal.header.frame_id = 'panda_link0'
            self.corrected_goal_pub.publish(corrected_goal)
            self.goal_pub.publish(goal)
            if min_val < 0.3:
                if abs(x_distance) > self.x_dist_threshold or abs(y_distance) > self.y_dist_threshold:
                    print("applying correction ", x_distance, " ", y_distance)
                    z_sum += transform[2,3]
                self.recorded_traj = self.recorded_traj + transform[:3, 3].reshape((3,1))

            if self.force.z > 9 and bool(int(spiral_flag)):
                spiral_success, offset_correction = self.spiral_search(goal)
                self.spiralling_occured = True
                if spiral_success:
                    self.recorded_traj[0, i:] += offset_correction[0]
                    self.recorded_traj[1, i:] += offset_correction[1]

            goal_pos_array = position_2_array(goal.pose.position)
            if np.linalg.norm(self.curr_pos-goal_pos_array) <= self.attractor_distance_threshold:
                i=i+1
            self.r.sleep()

            # Stop playback if at end of trajectory (some indices might be deleted by feedback)
            if i == self.recorded_traj.shape[1]-1:
                break
        if self.spiralling_occured:
            print(f"recording {self.filename}, spiralling occured")
            
    def save(self, file='last'):
        np.savez(self._package_path + '/trajectories/' + str(file) + '.npz',
                 traj=self.recorded_traj,
                 ori=self.recorded_ori,
                 grip=self.recorded_gripper,
                 img=self.recorded_img)
    
    def load(self, file='last'):
        data = np.load(self._package_path + '/trajectories/' + str(file) + '.npz')
        self.recorded_traj = data['traj']
        self.recorded_ori = data['ori']
        self.recorded_gripper = data['grip']
        self.recorded_img = data['img']
        self.recorded_img_tensor = torch.from_numpy(self.recorded_img).float()
        if torch.cuda.is_available():
            self.recorded_img_tensor  = self.recorded_img_tensor.cuda()
        self.filename=str(file)
