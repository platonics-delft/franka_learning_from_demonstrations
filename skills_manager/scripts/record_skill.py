#!/usr/bin/env python3
"""
Recording trajectories and storing them into a databaseself.
"""
from lfd import LfD, RALfD
import rospy

if __name__ == '__main__':
    try:
        name_skill = rospy.get_param("/recording_node/name_skill")
        risk_aware = rospy.get_param("/recording_node/risk_aware")
        print("Recording skill: ", name_skill)
        if risk_aware:
            print("Risk aware learning from demonstration")
            lfd = RALfD()
        else:
            lfd = LfD()
        lfd.traj_rec()
        lfd.save(name_skill)
    except rospy.ROSInterruptException:
        pass

