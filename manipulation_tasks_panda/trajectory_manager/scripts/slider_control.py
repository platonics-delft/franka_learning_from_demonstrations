#%%
#!/usr/bin/env python
import rospy
import math
import numpy as np
import quaternion
import time
import rospkg
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float32MultiArray
from pynput.keyboard import Listener, KeyCode
from manipulation_helpers.pose_transform_functions import orientation_2_quaternion, pose_st_2_transformation, position_2_array, array_quat_2_pose, transformation_2_pose, transform_pose, list_2_quaternion
from panda import Panda
import copy
import pdb
class LfD(Panda):
    def __init__(self):
        super().__init__()
        rospy.init_node("slider_control_node")
        self.r=rospy.Rate(5)
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
        self.screen_threshold = 10

        self.slider_sub=rospy.Subscriber("/slider_node/result", Float32MultiArray, self.slider_callback)
        self.screen_yellow_distance=None
        self.screen_green_distance=None
        self.screen_orientation=0.01 
        rospy.sleep(1)

    def slider_callback(self, msg):
        if self.screen_yellow_distance:
            if abs(msg.data[0]-self.screen_yellow_distance)<50: #pixel space
                self.screen_yellow_distance = msg.data[0]
            else:
                print("Outlier detected!")
        else:    
            self.screen_yellow_distance = msg.data[0]

        if self.screen_green_distance:
            if abs(msg.data[1]-self.screen_green_distance)<50: #pixel space
                self.screen_green_distance = msg.data[1]
            else:
                print("Outlier detected!")
        elif msg.data[1] < 1000:     
            self.screen_green_distance = msg.data[1]    

        if  np.abs(msg.data[2]) < 30: #safety in degree space
            self.screen_orientation = msg.data[2]
        
    def slerp_sat(self, q1, q2, theta_max_perc): 
        '''
        This function goes to q2 from q1 but with set maximum theta
        '''
        theta_max=theta_max_perc*np.pi/2
        inner=np.inner(q1,q2)

        theta= np.arccos(np.abs(inner)) 
        q_slerp=copy.deepcopy(q2)
        # print("Theta",theta)
        if theta>theta_max:
            #print('out_of_bounds')
            #if 
            q_slerp.w=(np.sin(theta-theta_max)*q1.w+np.sin(theta_max)*q2.w)/np.sin(theta)
            q_slerp.x=(np.sin(theta-theta_max)*q1.x+np.sin(theta_max)*q2.x)/np.sin(theta)
            q_slerp.y=(np.sin(theta-theta_max)*q1.y+np.sin(theta_max)*q2.y)/np.sin(theta)
            q_slerp.z=(np.sin(theta-theta_max)*q1.z+np.sin(theta_max)*q2.z)/np.sin(theta)
        return q_slerp 

    def execute(self):
        self.set_stiffness(4000, 4000, 4000, 30, 30, 30, 0)
        curr_quat = list_2_quaternion(self.curr_ori)
        # curr_euler = quaternion.as_euler_angles(curr_quat)
        # straight_down_quat = quaternion.from_euler_angles(0, 0, curr_euler[2])
        # goal_pose = array_quat_2_pose(self.curr_pos, straight_down_quat * curr_quat)
        goal_quat= copy.deepcopy(curr_quat)
        goal_quat.z=0
        goal_quat.w=0
        goal_quat.x=np.sqrt(1-goal_quat.y**2)
        self.goal_pub.publish( array_quat_2_pose(self.curr_pos, goal_quat))
        rospy.sleep(0.5)
        curr_pos=copy.deepcopy(self.curr_pos)
        self.y_increment=0.001
        # for i in range(30):
        #     angle_rad = - 0.2* np.radians(self.screen_orientation) #Mariano likes a minus here
        #     diff_quat = quaternion.from_euler_angles(0, 0, angle_rad)
        #     curr_quat = list_2_quaternion(self.curr_ori)
        #     diff_quat_base_frame = diff_quat * goal_quat
        #     # pdb.set_trace()
        #     goal_quat = self.slerp_sat(curr_quat, diff_quat_base_frame, 0.02)
        #     goal_pose = array_quat_2_pose(curr_pos, goal_quat)
        #     goal_pose.header.frame_id = "panda_link0"
        #     self.goal_pub.publish(goal_pose)
        #     self.r.sleep()
               #     angle_rad = - 0.2* np.radians(self.screen_orientation) #Mariano likes a minus here
        angle_rad = -np.radians(self.screen_orientation) #Mariano likes a minus here       
        diff_quat = quaternion.from_euler_angles(0, 0, angle_rad)
        curr_quat = list_2_quaternion(self.curr_ori)
        diff_quat_base_frame = diff_quat * goal_quat
        # pdb.set_trace()
        goal_quat = self.slerp_sat(curr_quat, diff_quat_base_frame, 0.02)
        goal_pose = array_quat_2_pose(curr_pos, goal_quat)
        goal_pose.header.frame_id = "panda_link0"
        # self.goal_pub.publish(goal_pose)
        self.go_to_pose(goal_pose)
        self.r.sleep()  
        goal_pos_prev = self.curr_pos
        while self.screen_yellow_distance > self.screen_threshold and not(self.screen_green_distance): #np.abs(self.screen_distance) > self.screen_threshold:
            goal_pos_ee = np.array([0, -np.sign(self.screen_yellow_distance)*self.y_increment, 0])
            ee_rot_mat= quaternion.as_rotation_matrix(goal_quat)
            goal_pos= ee_rot_mat @ goal_pos_ee
            goal_pose = array_quat_2_pose(goal_pos_prev+goal_pos, goal_quat)
            goal_pose.header.frame_id = "panda_link0"
            self.goal_pub.publish(goal_pose)
            goal_pos_prev = goal_pos_prev + goal_pos
            self.r.sleep()
        if  self.screen_green_distance < 0: #switch side
                goal_pos = np.array([0, 0, 0.05])
                goal_pos_prev=goal_pos_prev+goal_pos
                goal_pose = array_quat_2_pose(goal_pos_prev+goal_pos, goal_quat)
                self.go_to_pose(goal_pose) 
                goal_pos_ee = np.array([0, -0.03, 0])
                ee_rot_mat= quaternion.as_rotation_matrix(goal_quat)
                goal_pos= ee_rot_mat @ goal_pos_ee
                goal_pos_prev=goal_pos_prev+goal_pos
                goal_pos = np.array([0, 0, -0.05])
                goal_pos_prev=goal_pos_prev+goal_pos
                goal_pose = array_quat_2_pose(goal_pos_prev, goal_quat)
                goal_pose.header.frame_id = "panda_link0"
                self.go_to_pose(goal_pose)    
        while self.screen_green_distance > self.screen_threshold: #np.abs(self.screen_distance) > self.screen_threshold:
            goal_pos_ee = np.array([0, -np.sign(self.screen_green_distance)*self.y_increment, 0])
            ee_rot_mat= quaternion.as_rotation_matrix(goal_quat)
            goal_pos= ee_rot_mat @ goal_pos_ee
            goal_pose = array_quat_2_pose(goal_pos_prev+goal_pos, goal_quat)
            goal_pose.header.frame_id = "panda_link0"
            self.goal_pub.publish(goal_pose)
            goal_pos_prev = goal_pos_prev + goal_pos
            self.r.sleep()       
    
if __name__ == "__main__":
    slider_control = LfD()
    rospy.sleep(0.1)
    slider_control.execute()


