#!/usr/bin/env python3
"""
Playback of trajectories and storing them into a databaseself.
"""
from lfd import LfD, RALfD
from std_srvs.srv import Trigger
import sys
import os
import rospy
import numpy as np 
from panda_ros.pose_transform_functions import array_quat_2_pose
if __name__ == '__main__':
    name_skills = rospy.get_param('/execute_node/name_skill')
    localize_box = rospy.get_param('/execute_node/localize_box')
    print("Executing skill: ", name_skills)
    print("Localize box: ", localize_box)
    lfd = RALfD()

    position = rospy.get_param("position")
    orientation = rospy.get_param("orientation") 

    pos_array = np.array([position['x'], position['y'], position['z']])
    quat = np.quaternion(orientation['w'], orientation['x'], orientation['y'], orientation['z'])
    goal = array_quat_2_pose(pos_array, quat)
    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    lfd.go_to_pose(goal)

    if localize_box:
        rospy.wait_for_service('active_localizer')
        active_localizer = rospy.ServiceProxy('active_localizer', Trigger)
        resp = active_localizer()
        lfd.compute_final_transform() 
    
    try:
        lfd.load(name_skills)
        lfd.execute()
    except rospy.ROSInterruptException:
        pass

    lfd.save(name_skills, risk_exec_trial=True)

    lfd.update_risk_model(name_skills)

