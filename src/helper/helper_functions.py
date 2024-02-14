import rospy
import math
from geometry_msgs.msg import *

def standardlize_angle(angle):
    '''Standarlize an angle in range [-pi pi)
    return standardlized_angle'''
    standardlized_angle =  math.remainder(angle, math.tau)
    if standardlized_angle == math.pi:
        standardlized_angle = -standardlized_angle
    return standardlized_angle

def get_direction_2D_vector(vector):
    '''Return angle of a vector
    Angle is standardlized in range [-pi, pi)'''
    return math.atan2(vector.y, vector.x)

def get_norm_2D_vector(vector):
    '''Return norm of 2D vector
    ''' 
    return math.sqrt(vector.x ** 2 + vector.y ** 2)

def get_2D_vector(point_1, point_2):
    '''Return 2D vector (x, y, 0) points from point_1 to point_2
    point_1, point_2 are in (x, y) form'''

    return Vector3(point_2.x - point_1.x, 
                   point_2.y - point_1.y, 
                   0)
                 
def get_norm_3D_vector(vector):
    '''Return norm of 3D vector
    ''' 
    return math.sqrt(vector.x ** 2 + vector.y ** 2 + vector.z ** 2)

def get_3D_vector(point_1, point_2):
    '''Return 3D vector points from point_1 to point_2
    point_1, point_2 are in (x, y, z) form'''

    return Vector3(point_2.x - point_1.x, 
                   point_2.y - point_1.y,
                   point_2.z - point_1.z)

def get_unit_vector(vector):
    '''Return unit vector of a vector
    vector is in Vector(x, y, z) form'''

    norm = get_norm_2D_vector(vector)
    if norm == 0:
        unit_vector = vector 
    else:
        unit_vector = mul_vector(vector, 1 / norm)
    return unit_vector

def get_distance_2D(point_1, point_2):
    ''' Return distance of two 2D points
    point_1, point_2 are in (x, y) form'''

    return math.sqrt((point_1.x - point_2.x) ** 2 + 
                     (point_1.y - point_2.y) ** 2)

def get_distance_3D(point_1, point_2):
    ''' Return distance of two 3D points
    point_1, point_2 are in (x, y, z) form'''

    return math.sqrt((point_1.x - point_2.x) ** 2 + 
                     (point_1.y - point_2.y) ** 2 +
                     (point_1.z - point_2.z) ** 2)

def mul_vector(vector, k):
    '''Return k * vector'''
    return Vector3(k * vector.x, k * vector.y, k * vector.z)

def add_vector(*vectors):
    '''Return vector sum'''
    sum = Vector3(0, 0, 0)
    for vector in vectors:
        sum.x += vector.x
        sum.y += vector.y 
        sum.z += vector.z
    return sum

def get_constrained_vector(vector, max_norm):
    '''Return vector with the same direction with original vector 
    but is constrained by maximum norm'''
    norm = get_norm_3D_vector(vector)
    if norm > max_norm:
        k = norm / max_norm
        constrained_vector = Vector3(vector.x / k, vector.y / k, vector.z / k)
        return constrained_vector
    else:
        return vector
    