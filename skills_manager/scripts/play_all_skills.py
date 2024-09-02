#!/usr/bin/env python3
"""
Playback of trajectories and storing them into a databaseself.
"""
from lfd import LfD
from std_srvs.srv import Trigger
import sys
import os
import rospy
import numpy as np
from panda_ros.pose_transform_functions import pos_quat_2_pose_st

if __name__ == '__main__':
    localize_box = rospy.get_param('/execute_node/localize_box')
    print("Localize box: ", localize_box)
    lfd = LfD()
    
    position = rospy.get_param("position")
    orientation = rospy.get_param("orientation") 

    pos_array = np.array([position['x'], position['y'], position['z']])
    quat = np.quaternion(orientation['w'], orientation['x'], orientation['y'], orientation['z'])
    goal = pos_quat_2_pose_st(pos_array, quat)
    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    lfd.go_to_pose(goal)

    if localize_box:
        rospy.wait_for_service('active_localizer')
        active_localizer = rospy.ServiceProxy('active_localizer', Trigger)
        resp = active_localizer()
        lfd.compute_final_transform() 
    try:
        
        lfd.load("peg_pick")
        lfd.execute()

        lfd.load("peg_door")
        lfd.execute()

        lfd.load("peg_place")
        lfd.execute()

        lfd.camera_correction = np.array([0,0,0]) # reset camera corrections
        lfd.load("probe_pick")
        lfd.execute()
        lfd.camera_correction = np.array([0,0,0]) # reset camera corrections
        lfd.load("probe_probe")
        lfd.execute()
        
        lfd.camera_correction = np.array([0,0,0]) # reset camera corrections
        lfd.load("probe_place")
        lfd.execute(retry_insertion_flag=1)

        lfd.camera_correction = np.array([0,0,0]) # reset camera corrections
        lfd.load("wrap")
        lfd.execute()

    except rospy.ROSInterruptException:
        pass



