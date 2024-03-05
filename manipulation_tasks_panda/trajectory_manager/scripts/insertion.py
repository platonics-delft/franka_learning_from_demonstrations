from panda_ros.pose_transform_functions import orientation_2_quaternion, position_2_array, array_quat_2_pose
import numpy as np
import rospy 
class Insertion():
    def __init__(self):
        super(Insertion, self).__init__()
    def spiral_search(self, goal, control_rate=30, force_min_exit=1): 
        # force_min_exit in Newton. If the verical force is lower than this value, the spiral search is considered successful
        r = rospy.Rate(control_rate)
        max_spiral_time = 30 # seconds
        increase_radius_per_second = 0.0005 # meters, distance from center of the spiral after 1 second
        rounds_per_second = 1 # how many rounds does the spiral every second
        dt = 1. / control_rate
        goal_init = position_2_array(goal.pose.position)
        pos_init = self.curr_pos
        ori_quat = orientation_2_quaternion(goal.pose.orientation)
        goal_pose = array_quat_2_pose(goal_init, ori_quat)
        time= 0
        spiral_success = False
        self.set_stiffness(4000, 4000, 1000, 30, 30, 30, 0) # get more compliant in z direction
        for _ in range(max_spiral_time * control_rate):   
            goal_pose.pose.position.x = pos_init[0] + np.cos(
                2 * np.pi *rounds_per_second*time) * increase_radius_per_second * time
            goal_pose.pose.position.y = pos_init[1] + np.sin(
                2 * np.pi *rounds_per_second* time) * increase_radius_per_second * time
            self.goal_pub.publish(goal_pose)
            if self.force.z <= force_min_exit: 
                spiral_success = True
                break
            time += dt
            r.sleep()
        self.set_stiffness(4000, 4000, 4000, 30, 30, 30, 0)    
        offset_correction = self.curr_pos - goal_init

        return spiral_success, offset_correction
