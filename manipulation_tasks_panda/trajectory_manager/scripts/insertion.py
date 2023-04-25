from manipulation_helpers.pose_transform_functions import orientation_2_quaternion, pose_st_2_transformation, position_2_array, array_quat_2_pose, transformation_2_pose, transform_pose, list_2_quaternion
import numpy as np

class Insertion():
    def __init__(self):
        super(Insertion, self).__init__()
    def spiral_search(self, goal):
        goal_init = position_2_array(goal.pose.position)
        pos_init = self.curr_pos
        ori_quat = orientation_2_quaternion(goal.pose.orientation)
        goal_pose = array_quat_2_pose(goal_init, ori_quat)
        time_spiral = 0
        spiral_success = False
        spiral_width = 2 * np.pi
        spiral_speed = 1. / 20.
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
            time_spiral += spiral_speed
            self.r.sleep()
        self.set_stiffness(4000, 4000, 4000, 30, 30, 30, 0)    
        offset_correction = self.curr_pos - goal_init

        return spiral_success, offset_correction
