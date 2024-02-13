#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import * 
from geometry_msgs.msg import *
from hummingbird.hummingbird import Hummingbird
if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv = sys.argv)
        if len(argv) < 3:
            print('not enough parameters')
            pass
        hummingbird = Hummingbird()
        goal_position = Point(float(argv[1]), float(argv[2]), hummingbird.pose.position.z)
        print("target:\n ", goal_position)
        hummingbird.single_move_to_goal(goal_position)
        print(" Done")
    except rospy.ROSInterruptException:
        pass