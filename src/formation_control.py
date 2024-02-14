#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import * 
from geometry_msgs.msg import *
from hummingbird.formation_hummingbird import HummingbirdFormation
if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv = sys.argv)
        if len(argv) < 4:
            print('not enough parameters')
            pass
        formation = HummingbirdFormation(number_of_drones=int(argv[1]))
        goal_position = Point(float(argv[2]), float(argv[3]), formation.leader.height)
        print(int(argv[3]), "hummingbirds," ,"target:\n ", goal_position)
        formation.formation_control(goal_position)
        print(" Done")
    except rospy.ROSInterruptException:
        pass