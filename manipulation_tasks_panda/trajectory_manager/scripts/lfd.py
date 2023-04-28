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
from feedback import Feedback
from insertion import Insertion
from transform import Transform 

class LfD(Panda, Feedback, Insertion, Transform):
    def __init__(self):
        rospy.init_node("learning_node")
        super(LfD, self).__init__()
        self.r=rospy.Rate(20)
        self.pose = Pose()
        self.recorded_traj = None 
        self.recorded_ori=None
        self.recorded_gripper= None
        self.end=False
        self.attractor_distance_threshold=0.05
        self.final_transform = np.eye(4)

        self.listener = Listener(on_press=self._on_press)
        self.listener.start()
        self.spiraling = False
        
        self.pose_icp = None

        ros_pack = rospkg.RosPack()
        self._package_path = ros_pack.get_path('trajectory_manager')
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
        self.set_K.update_configuration({"nullspace_stiffness": 0})  

        init_pos = self.curr_pos
        vel = 0 ##### Change to a more meaningful name like distance? Trigger could be distance_interval or something.
        print("Move robot to start recording.")
        while vel < trigger:
            vel = math.sqrt((self.curr_pos[0]-init_pos[0])**2 + (self.curr_pos[1]-init_pos[1])**2 + (self.curr_pos[2]-init_pos[2])**2)
        
        self.recorded_traj = self.curr_pos
        self.recorded_ori = self.curr_ori
        if self.gripper_width < 0.02:
            self.grip_value = 0
        else:
            self.grip_value = 0.025
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
        
        # start = transform_pose(start, self.final_transform) 
        self.go_to_pose(start)

        self.time_index=0
        while self.time_index <( self.recorded_traj.shape[1]):

            quat_goal = list_2_quaternion(self.recorded_ori[:, self.time_index])
            goal = array_quat_2_pose(self.recorded_traj[:, self.time_index], quat_goal)
            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            ori_threshold = 0.3
            pos_threshold = 0.1
            
            # goal = transform_pose(goal, self.final_transform)
            
            self.correct()

            if (self.recorded_gripper[0][self.time_index]-self.recorded_gripper[0][max(0,self.time_index-1)]) < -0.02:
                print("closing gripper")
                self.grasp_gripper(self.recorded_gripper[0][self.time_index])
                time.sleep(0.1)

            if (self.recorded_gripper[0][self.time_index]-self.recorded_gripper[0][max(0,self.time_index-1)]) >0.02:
                print("open gripper")
                self.move_gripper(self.recorded_gripper[0][self.time_index])
                time.sleep(0.1)

            self.goal_pub.publish(goal)

            if self.force.z > 9 and bool(int(spiral_flag)):
                spiral_success, offset_correction = self.spiral_search(goal)
                self.spiralling_occured = True
                if spiral_success:
                    self.recorded_traj[0, self.time_index:] += offset_correction[0]
                    self.recorded_traj[1, self.time_index:] += offset_correction[1]

            goal_pos_array = position_2_array(goal.pose.position)
            if np.linalg.norm(self.curr_pos-goal_pos_array) <= self.attractor_distance_threshold:
                self.time_index=self.time_index+1
            self.r.sleep()

            # Stop playback if at end of trajectory (some indices might be deleted by feedback)
            if self.time_index == self.recorded_traj.shape[1]-1:
                break
        if self.spiralling_occured:
            print(f"recording {self.filename}, spiralling occured")

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
        self.recorded_traj, self.recorded_ori = self.transform_traj_ori(self.recorded_traj, self.recorded_ori, self.final_transform)
        self.filename=str(file)
