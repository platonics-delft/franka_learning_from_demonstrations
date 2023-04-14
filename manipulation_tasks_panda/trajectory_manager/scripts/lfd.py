#%%
#!/usr/bin/env python
import rospy
import math
import numpy as np
import time
import rospkg
from geometry_msgs.msg import PoseStamped, Pose, WrenchStamped
from pynput.keyboard import Listener, KeyCode
from manipulation_helpers.pose_transform_functions import orientation_2_quaternion, pose_st_2_transformation, position_2_array, array_quat_2_pose, transformation_2_pose, transform_pose, list_2_quaternion
from panda import Panda
class LfD(Panda):
    def __init__(self):
        super().__init__()
        rospy.init_node("learning_node")
        self.r=rospy.Rate(100)
        self.pose = Pose()
        self.feedback=np.zeros(4)
        self.feedback_gain=0.002
        self.faster_counter=0
        self.length_scale = 0.005
        self.correction_window = 300
        self.recorded_traj = None 
        self.recorded_ori=None
        self.recorded_gripper= None
        self.end=False
        self.grip_value=0.04
        self.attractor_distance_threshold=0.05
        
        self.transform_box_sub = rospy.Subscriber("/box_transform", PoseStamped, self.transform_icp_callback)

        self.listener = Listener(on_press=self._on_press)
        self.listener.start()
        self.spiraling = False
        
        self.pose_icp = None

        ros_pack = rospkg.RosPack()
        self._package_path = ros_pack.get_path('trajectory_manager')
        
        trans_base_2_hand = np.array([0.308, -0.000, 0.588])
        rot_base_2_hand = list_2_quaternion([-0.002, 1.000, 0.006, -0.002]) 
        self.pose_base_2_hand = array_quat_2_pose(trans_base_2_hand, rot_base_2_hand)
        self.transform_base_2_hand = pose_st_2_transformation(self.pose_base_2_hand)

        trans_hand_2_cam = np.array([0.05073875796492183, -0.03418064441841842, 0.033397])
        rot_hand_2_cam = list_2_quaternion([0.7140855447, 0.005087158, -0.00459640, 0.70002409]) 
        self.pose_hand_2_cam = array_quat_2_pose(trans_hand_2_cam, rot_hand_2_cam)
        self.transform_hand_2_cam = pose_st_2_transformation(self.pose_hand_2_cam)

        rospy.sleep(1)

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

    def transform_icp_callback(self, pose_icp):
        self.pose_icp = pose_icp
        # self.pose_icp.pose.position.z = 0

    def square_exp(self, ind_curr, ind_j):
        dist = np.sqrt((self.recorded_traj[0][ind_curr]-self.recorded_traj[0][ind_j])**2+(self.recorded_traj[1][ind_curr]-self.recorded_traj[1][ind_j])**2+(self.recorded_traj[2][ind_curr]-self.recorded_traj[2][ind_j])**2)
        sq_exp = np.exp(-dist**2/self.length_scale**2)
        return sq_exp

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
        self.set_K.update_configuration({"nullspace_stiffness": 0})  

        init_pos = self.curr_pos
        vel = 0 ##### Change to a more meaningful name like distance? Trigger could be distance_interval or something.
        print("Move robot to start recording.")
        while vel < trigger:
            vel = math.sqrt((self.curr_pos[0]-init_pos[0])**2 + (self.curr_pos[1]-init_pos[1])**2 + (self.curr_pos[2]-init_pos[2])**2)
        
        self.recorded_traj = self.curr_pos
        self.recorded_ori = self.curr_ori
        self.recorded_gripper= self.grip_value
        
        print("Recording started. Press e to stop.")
        while not self.end:
            self.recorded_traj = np.c_[self.recorded_traj, self.curr_pos]
            self.recorded_ori  = np.c_[self.recorded_ori, self.curr_ori]
            self.recorded_gripper = np.c_[self.recorded_gripper, self.grip_value]

            self.r.sleep()

        quat_goal = list_2_quaternion(self.curr_ori)
        goal = array_quat_2_pose(self.curr_pos, quat_goal)
        self.goal_pub.publish(goal)
        self.set_stiffness(100, 100, 100, 5, 5, 5, 0)
        rospy.loginfo("Ending trajectory recording")

    def execute(self, spiral_flag):
        self.spiralling_occured = False
        # self.pose_icp = None 
        print("spiral flag", bool(int(spiral_flag)))
        print('entered execute')
        start = PoseStamped()

        quat_start = list_2_quaternion(self.recorded_ori[:, 0])
        start = array_quat_2_pose(self.recorded_traj[:, 0], quat_start)
        
        if self.pose_icp:
            transform_base_2_cam = self.transform_base_2_hand @ self.transform_hand_2_cam
            # if transform box is not in camera frame, remove the base_2_cam transforms
            transform_box = pose_st_2_transformation(self.pose_icp)
            transform = transform_base_2_cam @ transform_box @ np.linalg.inv(transform_base_2_cam)
            print("transforming", transform)
            start = transform_pose(start, transform)
        self.go_to_pose(start)

        i=0
        while i <( self.recorded_traj.shape[1]):

            quat_goal = list_2_quaternion(self.recorded_ori[:, i])
            goal = array_quat_2_pose(self.recorded_traj[:, i], quat_goal)
            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            ori_threshold = 0.3
            pos_threshold = 0.1
            
            if self.pose_icp:
            	goal = transform_pose(goal, transform)

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
                
            self.goal_pub.publish(goal)
            self.feedback = np.zeros(4)

            if self.force.z > 9 and bool(int(spiral_flag)):
                spiral_success, offset_correction = self.spiral_search(goal)
                self.spiralling_occured = True
                if spiral_success:
                    self.recorded_traj[0, i:] += offset_correction[0]
                    self.recorded_traj[1, i:] += offset_correction[1]

            if (self.recorded_gripper[0][i]-self.recorded_gripper[0][max(0,i-1)]) < -0.02:
                print("closing gripper")
                self.grasp_gripper(self.recorded_gripper[0][i])
                time.sleep(0.1)

            if (self.recorded_gripper[0][i]-self.recorded_gripper[0][max(0,i-1)]) >0.02:
                print("open gripper")
                self.move_gripper(self.recorded_gripper[0][i])
                time.sleep(0.1)

            goal_pos_array = position_2_array(goal.pose.position)
            if np.linalg.norm(self.curr_pos-goal_pos_array) <= self.attractor_distance_threshold:
                i=i+1
            self.r.sleep()

            # Stop playback if at end of trajectory (some indices might be deleted by feedback)
            if i == self.recorded_traj.shape[1]-1:
                break
        if self.spiralling_occured:
            print(f"recording {self.filename}, spiralling occured")

    def spiral_search(self, goal):
        goal_init = position_2_array(goal.pose.position)
        pos_init = self.curr_pos
        ori_quat = orientation_2_quaternion(goal.pose.orientation)
        goal_pose = array_quat_2_pose(goal_init, ori_quat)
        time_spiral = 0
        spiral_success = False
        spiral_width = 2 * np.pi
        self.set_stiffness(4000, 4000, 1000, 30, 30, 30, 0)
        for i in range(3000):
            spiral_width = 2 * np.pi   ######### Should we make this a class variable?
            goal_pose.pose.position.x = pos_init[0] + np.cos(
                spiral_width * time_spiral) * 0.0005 * time_spiral  # What is the 0.02?
            goal_pose.pose.position.y = pos_init[1] + np.sin(
                spiral_width * time_spiral) * 0.0005 * time_spiral  # What is the 0.02?
            self.goal_pub.publish(goal_pose)
            if self.force.z <= 1: #np.abs(goal_init[2] - self.curr_pos[2]) < 0.001:
                spiral_success = True
                break
            time_spiral += 1. / 100.
            self.r.sleep()
        self.set_stiffness(4000, 4000, 4000, 30, 30, 30, 0)    
        offset_correction = self.curr_pos - goal_init

        return spiral_success, offset_correction

    def save(self, file='last'):
        np.savez(self._package_path + '/trajectories/' + str(file) + '.npz',
                 traj=self.recorded_traj,
                 ori=self.recorded_ori,
                 grip=self.recorded_gripper)

    def load(self, file='last'):
        data = np.load(self._package_path + '/trajectories/' + str(file) + '.npz')
        self.recorded_traj = data['traj']
        self.recorded_ori = data['ori']
        self.recorded_gripper = data['grip']
        self.filename=str(file)
