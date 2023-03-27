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
from scipy.signal import savgol_filter
from manipulation_helpers.pose_transform_functions import orientation_2_quaternion, pose_st_2_transformation, position_2_array, array_quat_2_pose, transformation_2_pose, transform_pose, list_2_quaternion

class Learning_from_Demonstration():
    def __init__(self):
        rospy.init_node("learning_node")
        self.r=rospy.Rate(100)
        self.pose = Pose()
        self.K_pos=1000
        self.K_ori=30
        self.K_ns=10 ##### not being used
        self.feedback=np.zeros(4)
        self.feedback_gain=0.002
        self.faster_counter=0
        self.length_scale = 0.005
        self.correction_window = 300
        self.curr_pos=None
        self.curr_ori=None
        self.grip_width=None
        self.pick = 0 ##### not being used
        self.place = 0 ##### not being used
        self.recorded_traj = None 
        self.recorded_ori=None
        self.recorded_gripper= None
        self.end=False
        self.grip_value=1
        
        pose_ref_2_new_topic = rospy.get_param('pose_ref_2_new_topic', '/pose_ref_2_new')
        
        self.pos_sub=rospy.Subscriber("/cartesian_pose", PoseStamped, self.ee_pos_callback) ##### are the subs being used?
        self.gripper_sub=rospy.Subscriber("/joint_states", JointState, self.gripper_callback)
        self.pose_ref_2_new_sub = rospy.Subscriber(pose_ref_2_new_topic, PoseStamped, self.pose_ref_2_new_callback)
        self.force_feedback_sub = rospy.Subscriber('/force_torque_ext', WrenchStamped, self.force_feedback_callback)
        self.goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=0)
        self.grip_pub = rospy.Publisher('/gripper_online', Float32, queue_size=0)
        self.configuration_pub = rospy.Publisher('/equilibrium_confguration', Float32MultiArray, queue_size=0)
        self.force_feedback = 0.
        self.set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
        self.joint_states_sub = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()
        self.spiral = False
        self.spiraling = False
        self.pose_ref_2_new = None
        ros_pack = rospkg.RosPack()
        self._package_path = ros_pack.get_path('trajectory_manager')
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
            grip_command = Float32()
            grip_command.data = self.grip_value
            self.grip_pub.publish(grip_command)
            print('pressed c grip_value is ', grip_command.data)
        if key == KeyCode.from_char('o'):
            self.grip_value = 1
            grip_command = Float32()
            grip_command.data = self.grip_value
            self.grip_pub.publish(grip_command)
            print('pressed o grip_value is ', grip_command.data)
        if key == KeyCode.from_char('f'):
            self.feedback[3] = 1
        if key == KeyCode.from_char('x'):
            self.spiral = True


    def ee_pos_callback(self, curr_conf):
        self.curr_pos = np.array([curr_conf.pose.position.x, curr_conf.pose.position.y, curr_conf.pose.position.z])
        self.curr_ori = np.array([curr_conf.pose.orientation.w, curr_conf.pose.orientation.x, curr_conf.pose.orientation.y, curr_conf.pose.orientation.z])

    def pose_ref_2_new_callback(self, pose_ref_2_new):
        self.pose_ref_2_new = pose_ref_2_new

    ######## Not currently being used, can be used to specify certain width which the gripper needs to maintain
    def gripper_callback(self, curr_width):
        self.grip_width =curr_width.position[7]+curr_width.position[8]

    def force_feedback_callback(self, feedback):
        self.force = feedback.wrench.force
        self.force_feedback = np.linalg.norm(np.array([self.force.x, self.force.y, self.force.z]))

    def joint_states_callback(self, data):
        self.curr_joint = data.position[:7]

    def perf_spiral(self, goal):
        threshold = 4. ######### Should we make this a class variable?
        spiral_width = 0.3 ######### Should we make this a class variable?
        if np.abs(self.force_feedback) < threshold:
            self.spiraling = False
            return goal
        if not self.spiraling:
            self.spiraling = True
            self.time_spiral = 0

        goal.pose.position.x = goal.pose.position.x + np.cos(spiral_width * self.time_spiral) * 0.02 * self.time_spiral # What is the 0.02?
        goal.pose.position.y = goal.pose.position.y + np.sin(spiral_width * self.time_spiral) * 0.02 * self.time_spiral # What is the 0.02?
        self.time_spiral += 1./100.

        return goal

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
        self.recorded_traj = savgol_filter(self.recorded_traj, 51, 3)
        self.recorded_ori = savgol_filter(self.recorded_ori, 51, 3)

    # control robot to desired goal position
    def go_to_pose(self, goal_pose, interp_dist=0.001/1.5, interp_dist_polar=0.001/1.5): ##### Are both interpolation distances needed?
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
                   
            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = x[i]
            goal.pose.position.y = y[i]
            goal.pose.position.z = z[i]
            quat=np.slerp_vectorized(q_start, q_goal, i/step_num)
            
            goal.pose.orientation.x = quat.x
            goal.pose.orientation.y = quat.y
            goal.pose.orientation.z = quat.z
            goal.pose.orientation.w = quat.w
            self.goal_pub.publish(goal)
            self.r.sleep()
        rospy.sleep(2.0)
        self.set_stiffness(4000, 4000, 4000, 30, 30, 30, 0)


    def execute(self, spiral_flag):
        self.spiralling_occured = False

        print("spiral flag", bool(int(spiral_flag)))
        ##### Do we want to keep the sleep for closing the gripper?
        grip_command_old = self.recorded_gripper[0][0]
        print('entered execute')
        start = PoseStamped()

        start.pose.position.x = self.recorded_traj[0][0]
        start.pose.position.y = self.recorded_traj[1][0]
        start.pose.position.z = self.recorded_traj[2][0]

        start.pose.orientation.w = self.recorded_ori[0][0]
        start.pose.orientation.x = self.recorded_ori[1][0]
        start.pose.orientation.y = self.recorded_ori[2][0]
        start.pose.orientation.z = self.recorded_ori[3][0]
        
        if self.pose_ref_2_new:
        	transform = pose_st_2_transformation(self.pose_ref_2_new)
        	start = transform_pose(start, transform)
        self.go_to_pose(start)

        # desired_joints = np.array(self.curr_joint)
        # desired_joints[0] = 0.008477713281629244        
        # self.set_stiffness(self.K_pos, self.K_pos, self.K_pos, self.K_ori, self.K_ori, 0, 5)
        # self.set_configuration(desired_joints)
        # rospy.sleep(5.0)
        # self.set_stiffness_key()
        # self.set_stiffness(100, 100, 100, 5, 5, 5, 100)
        # null_space_reset = False

        for i in range (self.recorded_traj.shape[1]):
            
            # if i > 100 and not null_space_reset:
            #     self.set_stiffness(4000, 4000, 4000, 30, 30, 30, 0)
            #     null_space_reset = True


            goal = PoseStamped()

            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"		
            goal.pose.position.x = self.recorded_traj[0][i]
            goal.pose.position.y = self.recorded_traj[1][i]
            goal.pose.position.z = self.recorded_traj[2][i]

            goal.pose.orientation.w = self.recorded_ori[0][i]
            goal.pose.orientation.x = self.recorded_ori[1][i]
            goal.pose.orientation.y = self.recorded_ori[2][i]
            goal.pose.orientation.z = self.recorded_ori[3][i]
            
            ori_threshold = 0.3
            pos_threshold = 0.1
            
            
            
            if self.pose_ref_2_new:
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

            grip_command = Float32()

            grip_command.data = self.recorded_gripper[0][i]
            

            self.grip_pub.publish(grip_command)
            if (np.abs(grip_command_old-grip_command.data))>0.5:
                time.sleep(0.1)

            grip_command_old = grip_command.data
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


    def transform_trajectory(self, transform):
        # rospy.sleep(1)
        transf_pose = self.pose_ref_2_new

        trans = np.array([transf_pose.pose.position.x, transf_pose.pose.position.y, transf_pose.pose.position.z])
        quat_ref_to_new = np.quaternion(transf_pose.pose.orientation.w, transf_pose.pose.orientation.x,
                            transf_pose.pose.orientation.y, transf_pose.pose.orientation.z)

        rot_matrix = quaternion.as_rotation_matrix(quat_ref_to_new)
        transform_ref_to_new = np.append(rot_matrix, [[0, 0, 0]], axis=0)
        transform_ref_to_new = np.append(transform_ref_to_new, [[trans[0]], [trans[1]], [trans[2]], [1]], axis=1)

        for i in range(self.recorded_traj.shape[1]):
            quat_ori = np.quaternion(self.recorded_ori[0][i], self.recorded_ori[1][i], self.recorded_ori[2][i],
                                     self.recorded_ori[3][i])
            # Converting the quaternion to rotation matrix, to make a homogenous transformation and transform the points
            # with one matrix operation 'transform @ point'
            point = np.array([self.recorded_traj[0][i], self.recorded_traj[1][i], self.recorded_traj[2][i], 1])

            new_point = transform_ref_to_new @ point

            self.recorded_traj[0][i] = new_point[0]
            self.recorded_traj[1][i] = new_point[1]
            self.recorded_traj[2][i] = new_point[2]
            # Note 'extra' final rotation by q(0, 1, 0, 0) (180 deg about x axis) since we want gripper facing down
            new_ori = quat_ref_to_new * quat_ori

            self.recorded_ori[0][i] = new_ori.w
            self.recorded_ori[1][i] = new_ori.x
            self.recorded_ori[2][i] = new_ori.y
            self.recorded_ori[3][i] = new_ori.z

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
