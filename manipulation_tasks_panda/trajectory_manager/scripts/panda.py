#%%
#!/usr/bin/env python
import rospy
import sys
import math
import numpy as np
import quaternion # pip install numpy-quaternion
import time
import rospkg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, WrenchStamped
from std_msgs.msg import Float32MultiArray, Float32
import dynamic_reconfigure.client
from pynput.keyboard import Listener, KeyCode
from manipulation_helpers.pose_transform_functions import  array_quat_2_pose
from franka_gripper.msg import GraspActionGoal, HomingActionGoal, StopActionGoal, MoveActionGoal
class Panda():
    def __init__(self):
        super(Panda, self).__init__()
        self.K_pos=1000
        self.K_ori=30
        self.K_ns=10 ##### not being used
        self.curr_pos=None
        self.curr_ori=None
        self.attractor_distance_threshold=0.05
        
        self.pos_sub=rospy.Subscriber("/cartesian_pose", PoseStamped, self.ee_pos_callback)

        self.force_feedback_sub = rospy.Subscriber('/force_torque_ext', WrenchStamped, self.force_feedback_callback)
        self.goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=0)
        self.configuration_pub = rospy.Publisher('/equilibrium_confguration', Float32MultiArray, queue_size=0)
        self.grasp_pub = rospy.Publisher("/franka_gripper/grasp/goal", GraspActionGoal,
                                           queue_size=0)
        self.move_pub = rospy.Publisher("/franka_gripper/move/goal", MoveActionGoal,
                                           queue_size=0)
        self.homing_pub = rospy.Publisher("/franka_gripper/homing/goal", HomingActionGoal,
                                          queue_size=0)
        self.stop_pub = rospy.Publisher("/franka_gripper/stop/goal", StopActionGoal,
                                          queue_size=0)
        
        self.force_feedback = 0.
        self.set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
        self.joint_states_sub = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
 
        self.move_command=MoveActionGoal()
        self.grasp_command = GraspActionGoal()
        self.home_command = HomingActionGoal()
        self.stop_command = StopActionGoal()
        self.gripper_width = 0
        self.move_command.goal.speed=1
        self.grasp_command.goal.epsilon.inner = 0.3
        self.grasp_command.goal.epsilon.outer = 0.3
        self.grasp_command.goal.speed = 0.1
        self.grasp_command.goal.force = 50
        self.grasp_command.goal.width = 1
        rospy.sleep(1)

    def ee_pos_callback(self, curr_conf):
        self.curr_pos = np.array([curr_conf.pose.position.x, curr_conf.pose.position.y, curr_conf.pose.position.z])
        self.curr_ori = np.array([curr_conf.pose.orientation.w, curr_conf.pose.orientation.x, curr_conf.pose.orientation.y, curr_conf.pose.orientation.z])

    def move_gripper(self,width):
        self.move_command.goal.width=width
        self.move_pub.publish(self.move_command)

    def grasp_gripper(self, width):
        self.grasp_command.goal.width = width
        self.grasp_pub.publish(self.grasp_command)

    def home(self):
        pos_array = np.array([0.6, 0, 0.4])
        quat = np.quaternion(0, 1, 0, 0)
        goal = array_quat_2_pose(pos_array, quat)
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()

        ns_msg = [0, 0, 0, -2.4, 0, 2.4, 0]

        self.go_to_pose(goal)
        self.set_configuration(ns_msg)
        self.set_K.update_configuration({f'{str(self.name)}_nullspace_stiffness':10})

        rospy.sleep(rospy.Duration(secs=5))

        self.set_K.update_configuration({f'{str(self.name)}_nullspace_stiffness':0})

    def home_gripper(self):
        self.homing_pub.publish(self.home_command)

    def stop_gripper(self):
        self.stop_pub.publish(self.stop_command)  

    def force_feedback_callback(self, feedback):
        self.force = feedback.wrench.force
        self.force_feedback = np.linalg.norm(np.array([self.force.x, self.force.y, self.force.z]))

    def joint_states_callback(self, data):
        self.curr_joint = data.position[:7]

    def set_stiffness(self, k_t1, k_t2, k_t3,k_r1,k_r2,k_r3, k_ns):
        
        self.set_K.update_configuration({"translational_stiffness_X": k_t1})
        self.set_K.update_configuration({"translational_stiffness_Y": k_t2})
        self.set_K.update_configuration({"translational_stiffness_Z": k_t3})
        self.set_K.update_configuration({"rotational_stiffness_X": k_r1}) 
        self.set_K.update_configuration({"rotational_stiffness_Y": k_r2}) 
        self.set_K.update_configuration({"rotational_stiffness_Z": k_r3})
        self.set_K.update_configuration({"nullspace_stiffness": k_ns}) 

    def set_stiffness_key(self):
        self.set_stiffness(4000, 4000, 4000, 30, 30, 30, 0) ####### How to change this?

    def set_configuration(self, joint):
        joint_des = Float32MultiArray()
        joint_des.data = np.array(joint).astype(np.float32)
        self.configuration_pub.publish(joint_des)   

    # control robot to desired goal position
    def go_to_pose(self, goal_pose, interp_dist=0.05, interp_dist_polar=0.05): ##### Are both interpolation distances needed?
        # the goal pose should be of type PoseStamped. E.g. goal_pose=PoseStampled()
        start = self.curr_pos
        start_ori = self.curr_ori
        goal_array = np.array([goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z])

        # interpolate from start to goal with attractor distance of approx 1 cm
        dist = np.sqrt(np.sum(np.subtract(start, goal_array)**2, axis=0))
        
        step_num_lin = math.floor(dist / interp_dist)

        q_start=np.quaternion(start_ori[0], start_ori[1], start_ori[2], start_ori[3])
        
        q_goal=np.quaternion(goal_pose.pose.orientation.w, goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z)
        
        ##### Why is this step needed?
        inner_prod=q_start.x*q_goal.x+q_start.y*q_goal.y+q_start.z*q_goal.z+q_start.w*q_goal.w
        if inner_prod < 0:
            q_start.x=-q_start.x
            q_start.y=-q_start.y
            q_start.z=-q_start.z
            q_start.w=-q_start.w
        inner_prod=q_start.x*q_goal.x+q_start.y*q_goal.y+q_start.z*q_goal.z+q_start.w*q_goal.w
        theta= np.arccos(np.abs(inner_prod))
        print(theta)
        
        step_num_polar = math.floor(theta / interp_dist_polar)
        
        step_num=np.max([step_num_polar,step_num_lin])
        
        x = np.linspace(start[0], goal_pose.pose.position.x, step_num)
        y = np.linspace(start[1], goal_pose.pose.position.y, step_num)
        z = np.linspace(start[2], goal_pose.pose.position.z, step_num)

        # change this accordingly for now
        self.set_stiffness_key()

        goal = PoseStamped()
        self.set_stiffness(4000, 4000, 4000, 30, 30, 30, 40)
        for i in range(step_num):
            quat=np.slerp_vectorized(q_start, q_goal, i/step_num)
            pos_array = np.array([x[i], y[i], z[i]])
            goal = array_quat_2_pose(pos_array, quat)
            self.goal_pub.publish(goal)
            self.r.sleep()
        rospy.sleep(2.0)
        self.set_stiffness(4000, 4000, 4000, 30, 30, 30, 0)
