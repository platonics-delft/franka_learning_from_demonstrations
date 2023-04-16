import rospy
import math
import numpy as np
import time
import rospkg
from geometry_msgs.msg import PoseStamped, Pose, WrenchStamped
from pynput.keyboard import Listener, KeyCode
from panda import Panda
class Feedback(self):
    def __init__(self):
        self.feedback=np.zeros(4)
        self.feedback_gain=0.002
        self.faster_counter=0
        self.length_scale = 0.005
        self.correction_window = 300
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
        if key == KeyCode.from_char('o'):
            self.grip_value = 0.04
            self.move_gripper(self.grip_value)
        if key == KeyCode.from_char('f'):
            self.feedback[3] = 1
        key=0

    def square_exp(self, ind_curr, ind_j):
        dist = np.sqrt((self.recorded_traj[0][ind_curr]-self.recorded_traj[0][ind_j])**2+(self.recorded_traj[1][ind_curr]-self.recorded_traj[1][ind_j])**2+(self.recorded_traj[2][ind_curr]-self.recorded_traj[2][ind_j])**2)
        sq_exp = np.exp(-dist**2/self.length_scale**2)
        return sq_exp    

    def correct(self):
        if np.sum(self.feedback[:3])!=0:
            for j in range(self.recorded_traj.shape[1]):
                x = self.feedback[0]*self.square_exp(i, j)
                y = self.feedback[1]*self.square_exp(i, j)
                z = self.feedback[2]*self.square_exp(i, j)

                self.recorded_traj[0][j] += x
                self.recorded_traj[1][j] += y
                self.recorded_traj[2][j] += z
            
        if self.feedback[3] != 0:
            self.faster_counter = 10
            
        if self.faster_counter > 0 and i!=self.recorded_traj.shape[1]-1:
            self.faster_counter -= 1
            self.recorded_traj = np.delete(self.recorded_traj, i+1, 1)
            self.recorded_ori = np.delete(self.recorded_ori, i+1, 1)
            self.recorded_gripper = np.delete(self.recorded_gripper, i+1, 1)
                       
        self.feedback = np.zeros(4)    