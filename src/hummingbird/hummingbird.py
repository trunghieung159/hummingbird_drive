import numpy
import rospy
import math
import time
import copy
import rosnode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import *
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion as efq
from tf.transformations import quaternion_from_euler as qfe
from helper.helper_functions import *

r_a = 0.3
r_s = 2.0
v_max = 2.0
d_default = 0.8
a_default = math.pi / 4

k_o = 1.0
k_c = 0.12
k_r = 0.12 
k_f = 0.8
k_g = 0.6

b_c = 2.5
b_r = 2.5

v_mtg_max = 1
height_threshold = 0.05

linear_error = 0.1
angular_error = 0.3
class Hummingbird:
    def __init__(self, control_freq = 50, name = 'hummingbird', node_name = 'hummingbird'):
        self.control_freq = control_freq
        self.name = name

        self.pose = Pose()
        self.roll, self.pitch, self.yaw = 0, 0, 0
        self.obstacle_position = None
        self.velocity_vector = Vector3()

        if not node_name in rosnode.get_node_names():
            rospy.init_node(node_name, anonymous=False)
        self.rate = rospy.Rate(control_freq)
        prefix = '/' + name
        rospy.Subscriber(prefix + '/ground_truth/odometry', Odometry, self.__update_pose_callback)
        rospy.Subscriber(prefix + '/lidar/scan', PointCloud2, self.__update_obstacle_position_callback)
        self.velo_pub = rospy.Publisher(prefix + '/command/motor_speed', Actuators, queue_size=10000)
        self.goal_pub = rospy.Publisher(prefix + '/command/pose', PoseStamped, queue_size=10000)
        self.goal_demo_pub = rospy.Publisher(prefix + '/move_base_simple/goal', PoseStamped, queue_size=10000)
        self.__fully_initialized = False
        while not self.__fully_initialized:
            pass
        self.height = self.pose.position.z

    def move_by_velo_vector(self, velo_vector):
        '''Move hummingbird by velocity vector'''
        virtual_goal_pose = self.get_virtual_goal_pose(velo_vector)
        self.__send_goal(virtual_goal_pose)
        self.virtual_goal_pose = virtual_goal_pose
        self.velocity_vector = velo_vector
    
    def __update_pose_callback(self, msg):
        '''Update hummingbird's pose'''
        self.pose = msg.pose.pose 
        euleurs = efq([self.pose.orientation.x, 
                       self.pose.orientation.y, 
                       self.pose.orientation.z, 
                       self.pose.orientation.w], 
                       'sxyz')
        self.roll, self.pitch, self.yaw = euleurs[0], euleurs[1], euleurs[2]
        self.__fully_initialized = True 

    def __update_obstacle_position_callback(self, data:PointCloud2):
        '''Read sensor and update position of closest obstacle in worldframe coordinate'''
        drone_position = self.pose.position 
        min_distance = float('inf')
        obstacle_position = None
        for point in point_cloud2.read_points(data,
                                              field_names=("x", "y", "z"),
                                              skip_nans = True):
            distance = get_norm_2D_vector(Vector3(point[0], point[1], point[2]))
            if abs(point[2]) < height_threshold and distance < min(min_distance, r_s):
                min_distance = distance 
                obstacle_position = Point(drone_position.x + point[0],
                                          drone_position.y + point[1],
                                          drone_position.z + point[2])
        # print(obstacle_position)
        self.obstacle_position = obstacle_position
                    
    def __send_goal(self, goal_pose):
        command = PoseStamped()
        command.header.frame_id = "world"
        command.header.stamp = rospy.Time.now() 
        command.pose = goal_pose
        self.goal_pub.publish(command)
        self.goal_demo_pub.publish(command)

    def get_mtg_vector(self, goal_position):
        '''Return move to goal vector 
        goal_pose is Point(x y z) '''

        drone_to_goal_vector = mul_vector(get_2D_vector(self.pose.position, goal_position), k_g)
        # return drone_to_goal_vector
        return get_constrained_vector(drone_to_goal_vector, v_mtg_max)

    def get_oa_vector(self):
        obstacle_position = self.obstacle_position
        if obstacle_position == None:
            oa_vector = Vector3(0, 0, 0)
        else:
            obstacle_distance = get_distance_2D(self.pose.position, obstacle_position)
            magnitude = k_o * ((1 / obstacle_distance ** 2) - (1 / r_s ** 2))
            drone_to_obstalce_vector = get_2D_vector(self.pose.position, obstacle_position)
            oa_vector = mul_vector(get_unit_vector(drone_to_obstalce_vector), -magnitude)
        return oa_vector

    def single_move_to_goal(self, goal_position):
        '''Move single drone to goal
        Velocity is sum of mtg and oa vector'''
        while not self.is_at_goal(goal_position):
            mtg_vector = self.get_mtg_vector(goal_position)
            oa_vector = self.get_oa_vector()
            velocity_vector = get_constrained_vector(mtg_vector, v_max)
            velocity_vector = get_constrained_vector(add_vector(mtg_vector, oa_vector), v_max)
            virtual_goal_pose = self.get_virtual_goal_pose(velocity_vector)
            # print(virtual_goal_pose.position.x, virtual_goal_pose.position.y, virtual_goal_pose.position.z, "\n")
            self.__send_goal(virtual_goal_pose)
            self.rate.sleep()


    def get_virtual_goal_pose(self, velo_vector):
        '''Measure virtual_goal_pose from velocity vector 
        '''
        velo_vector = get_constrained_vector(velo_vector, v_max)
        control_period = 1 / self.control_freq
        drone_position = self.pose.position
        virtual_position = Point(drone_position.x + control_period * velo_vector.x,
                                 drone_position.y + control_period * velo_vector.y,
                                 self.height)
        
        yaw = get_direction_2D_vector(velo_vector)
        quaternions = qfe(0, 0, yaw)
        virtual_orientation = Quaternion(quaternions[0], quaternions[1], quaternions[2], quaternions[3])

        virtual_goal_pose = Pose()
        virtual_goal_pose.position = virtual_position
        virtual_goal_pose.orientation = virtual_orientation
        return virtual_goal_pose

    def is_at_goal(self, goal_position):
        '''Return true if drone is within linear_error from goal'''
        goal_distance = get_distance_2D(self.pose.position, goal_position)
        return goal_distance < linear_error
    
    def is_at_goal_pose(self, goal_pose):
        '''Return true if drone is in goal_pose'''
        goal_distance = get_distance_3D(self.pose.position, goal_pose.position)
        euleurs = efq([goal_pose.orientation.x, 
                       goal_pose.orientation.y, 
                       goal_pose.orientation.z, 
                       goal_pose.orientation.w], 
                       'sxyz')
        goal_yaw = euleurs[2]
        return goal_distance < linear_error ##and abs(goal_yaw - self.yaw) < angular_error