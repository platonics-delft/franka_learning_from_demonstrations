import rospy
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridgeError, CvBridge
import math
from scipy.signal import savgol_filter
import sys
import os
import dynamic_reconfigure.client
from pynput.keyboard import Listener, KeyCode
import rospkg

# from trajectory_manager.Learning_from_Demonstration import Learning_from_Demonstration
class Learning_from_Demonstration():
    def __init__(self):
        rospy.init_node("learning_node")
        self.r=rospy.Rate(10)
        self.pose = Pose()
        self.pos_sub=rospy.Subscriber("/cartesian_pose", PoseStamped, self.ee_pos_callback) ##### are the subs being used?
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        self.goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=0)
        self.cropped_img_pub = rospy.Publisher('/modified_img', Image, queue_size=0)

        self.listener = Listener(on_press=self._on_press)
        self.listener.start()

        self.bridge = CvBridge()

        self.curr_image = None
        self.recorded_traj = None
        self.recorded_ori = None
        self.height = 720
        self.width = 1280
        self.set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
        self.end = False

        ros_pack = rospkg.RosPack()
        self._package_path = ros_pack.get_path('learning_vision')


    def _on_press(self, key):
        rospy.loginfo(f"Event happened, user pressed {key}")
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('e'):
            self.end = True
        key=0

    def set_stiffness(self, k_t1, k_t2, k_t3,k_r1,k_r2,k_r3, k_ns):
        
        self.set_K.update_configuration({"translational_stiffness_X": k_t1})
        self.set_K.update_configuration({"translational_stiffness_Y": k_t2})
        self.set_K.update_configuration({"translational_stiffness_Z": k_t3})
        self.set_K.update_configuration({"rotational_stiffness_X": k_r1}) 
        self.set_K.update_configuration({"rotational_stiffness_Y": k_r2}) 
        self.set_K.update_configuration({"rotational_stiffness_Z": k_r3})
        self.set_K.update_configuration({"nullspace_stiffness": k_ns}) 

    def ee_pos_callback(self, curr_pose):
            self.curr_pos = np.array([curr_pose.pose.position.x, curr_pose.pose.position.y, curr_pose.pose.position.z])
            self.curr_ori = np.array([curr_pose.pose.orientation.w, curr_pose.pose.orientation.x, curr_pose.pose.orientation.y, curr_pose.pose.orientation.z])
    # Create a subscriber to receive image data from the camera
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

        h, w = self.curr_image.shape[:2]

                # Define the new dimensions
        self.width = int(1280/4)
        self.height = int(self.width * (h / w))

        # Resize the image
        resized_img = cv2.resize(self.curr_image, (self.width, self.height), interpolation=cv2.INTER_AREA)
        self.width=int(self.width/2)
        resized_img = resized_img[:, self.width:, :]
        resized_img_msg = self.bridge.cv2_to_imgmsg(resized_img)
        self.cropped_img_pub.publish(resized_img_msg)

        self.recorded_img = resized_img.reshape((1, self.height, self.width, 3)) 
        
        print("Recording started. Press e to stop.")
        while not self.end:
            self.recorded_traj = np.c_[self.recorded_traj, self.curr_pos]
            self.recorded_ori  = np.c_[self.recorded_ori, self.curr_ori]

            h, w = self.curr_image.shape[:2]

                # Define the new dimensions
            self.width = int(1280/4)
            self.height = int(self.width * (h / w))

            # Resize the image
            resized_img = cv2.resize(self.curr_image, (self.width, self.height), interpolation=cv2.INTER_AREA)
            self.width=int(self.width/2)
            resized_img = resized_img[:, self.width:, :]
            resized_img_msg = self.bridge.cv2_to_imgmsg(resized_img)
            self.cropped_img_pub.publish(resized_img_msg)
            self.recorded_img = np.r_[self.recorded_img, resized_img.reshape((1, self.height, self.width, 3))]

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

    def save(self, file='last'):
        np.savez(self._package_path + '/data/' + str(file) + '.npz',
                 traj=self.recorded_traj,
                 ori=self.recorded_ori,
                 img=self.recorded_img)

if __name__ == '__main__':
    try:
        arg1 = sys.argv[1]
        if len(sys.argv) == 3:
            save_prompt = int(sys.argv[2])
        else:
            save_prompt = 1
    except IndexError:
        print("Usage: " + os.path.basename(__file__) + " <trajectory_file_name>")
        sys.exit(1)

    lfd = Learning_from_Demonstration()
  
    lfd.traj_rec()

    save = None
    if not save_prompt:
        save = 0
    while not (save in [0,1]):
        print("SAVE CURRENT RECORDING? 0 = NO, 1 = YES")
        try:
            save = int(input('\n'))
        except:
            print("INVALID INPUT")
    if save:
        lfd.save(arg1)       
        