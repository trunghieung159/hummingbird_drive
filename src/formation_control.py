#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import * 
from geometry_msgs.msg import *
from hummingbird.formation_hummingbird import HummingbirdFormation
if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv = sys.argv)
        if len(argv) < 3:
            print('not enough parameters')
            pass
        print("hi")
        formation = HummingbirdFormation(number_of_drones=3)
        goal_position = Point(float(argv[1]), float(argv[2]), formation.leader.height)
        print("target:\n ", goal_position)
        formation.formation_control(goal_position)
        print(" Done")
    except rospy.ROSInterruptException:
        pass