import rospy
import math
import numpy as np
import time
import rospkg
from geometry_msgs.msg import PoseStamped, Pose, WrenchStamped
from pynput.keyboard import Listener, KeyCode
from panda import Panda
from manipulation_helpers.pose_transform_functions import orientation_2_quaternion, pose_st_2_transformation, position_2_array, array_quat_2_pose, transformation_2_pose, transform_pose, list_2_quaternion
class Feedback():
    def __init__(self):
        super(Feedback, self).__init__()
        self.feedback=np.zeros(4)
        self.feedback_gain=0.002
        self.faster_counter=0
        self.length_scale = 0.005
        self.correction_window = 300
        self.img_feedback_flag = 0
        self.spiral_flag = 0
        self.img_feedback_correction = 0
        self.gripper_feedback_correction = 0

    def _on_press(self, key):
        rospy.loginfo(f"Event happened, user pressed {key}")
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('e'):
            self.end = True
        # Feedback for translate forward/backward
        if key == KeyCode.from_char('w'):
            self.feedback[0] = self.feedback_gain
        if key == KeyCode.from_char('s'):
            self.feedback[0] = -self.feedback_gain
        # Feedback for translate left/right
        if key == KeyCode.from_char('a'):
            self.feedback[1] = self.feedback_gain
        if key == KeyCode.from_char('d'):
            self.feedback[1] = -self.feedback_gain
        # Feedback for translate up/down
        if key == KeyCode.from_char('u'):
            self.feedback[2] = self.feedback_gain
        if key == KeyCode.from_char('j'):
            self.feedback[2] = -self.feedback_gain
        # Close/open gripper
        if key == KeyCode.from_char('c'):
            self.grip_value = 0
            self.grasp_command.goal.epsilon.inner = 0.1
            self.grasp_command.goal.epsilon.outer = 0.1
            self.grasp_command.goal.force = 50
            self.grasp_gripper(self.grip_value)
            self.gripper_feedback_correction = 1

        if key == KeyCode.from_char('o'):
            self.grip_value = self.grip_open_width
            self.move_gripper(self.grip_value)
            self.gripper_feedback_correction = 1
        if key == KeyCode.from_char('f'):
            self.feedback[3] = 1
        if key == KeyCode.from_char('k'):
            print("camera feedback enabled")
            self.img_feedback_flag = 1
            self.img_feedback_correction = 1
        if key == KeyCode.from_char('l'):
            print("camera feedback disabled")
            self.img_feedback_flag = 0
            self.img_feedback_correction = 1
        if key == KeyCode.from_char('z'):
            print("spiral enabled")
            self.spiral_flag = 1
        if key == KeyCode.from_char('x'):
            print("spiral disabled")

        if key == KeyCode.from_char('m'):    
            self.spiral_flag = 0
            quat_goal = list_2_quaternion(self.curr_ori)
            goal = array_quat_2_pose(self.curr_pos, quat_goal)
            self.goal_pub.publish(goal)
            self.set_stiffness(0, 0, 0, 50, 50, 50, 0)
            print("higher rotatioal stiffness")

        if key == KeyCode.from_char('n'):    
            self.set_stiffness(0, 0, 0, 0, 0, 0, 0)
            print("zero rotatioal stiffness")
        key=0

    def square_exp(self, ind_curr, ind_j):
        dist = np.sqrt((self.recorded_traj[0][ind_curr]-self.recorded_traj[0][ind_j])**2+(self.recorded_traj[1][ind_curr]-self.recorded_traj[1][ind_j])**2+(self.recorded_traj[2][ind_curr]-self.recorded_traj[2][ind_j])**2)
        sq_exp = np.exp(-dist**2/self.length_scale**2)
        return sq_exp    

    def correct(self):
        if np.sum(self.feedback[:3])!=0:
            for j in range(self.recorded_traj.shape[1]):
                x = self.feedback[0]*self.square_exp(self.time_index, j)
                y = self.feedback[1]*self.square_exp(self.time_index, j)
                z = self.feedback[2]*self.square_exp(self.time_index, j)

                self.recorded_traj[0][j] += x
                self.recorded_traj[1][j] += y
                self.recorded_traj[2][j] += z
        
        if self.img_feedback_correction:
            self.recorded_img_feedback_flag[0, self.time_index:] = self.img_feedback_flag

        if self.gripper_feedback_correction:
            self.recorded_gripper[0, self.time_index:] = self.grip_value
            # print(self.recorded_gripper)

        if self.feedback[3] != 0:
            self.faster_counter = 10
            
        if self.faster_counter > 0 and self.time_index != self.recorded_traj.shape[1]-1:
            self.faster_counter -= 1
            self.recorded_traj = np.delete(self.recorded_traj, self.time_index+1, 1)
            self.recorded_ori = np.delete(self.recorded_ori, self.time_index+1, 1)
            self.recorded_gripper = np.delete(self.recorded_gripper, self.time_index+1, 1)
            self.recorded_img = np.delete(self.recorded_img, self.time_index+1, 0)
                       
        self.feedback = np.zeros(4)
        self.img_feedback_correction = 0
        self.gripper_feedback_correction = 0

    