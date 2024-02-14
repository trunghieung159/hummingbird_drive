import numpy
from hummingbird.hummingbird import *
class HummingbirdFormation:
    def __init__(self, distance = d_default, alpha = a_default,  number_of_drones = 5, control_freq = 50):
        self.leader_id = math.ceil(float(number_of_drones) / 2)
        self.drones = numpy.empty(number_of_drones, dtype=Hummingbird)
        self.number_of_drones = number_of_drones
        self.control_freq = control_freq
        self.a = alpha 
        self.d = distance

        rospy.init_node('formation_hummingbirds', anonymous=False)
        self.rate = rospy.Rate(control_freq)
        for i in range(number_of_drones):
            self.drones[i] = Hummingbird(name=("hummingbird" + str(i + 1)),
                                         node_name='formation_hummingbirds')
        self.leader = self.drones[self.leader_id - 1]

    def formation_control(self, goal_position):
        '''Move drones in formation
        '''
        follower_ids = list(range(1, self.number_of_drones + 1))
        follower_ids.remove(self.leader_id)
        ## before leader reached goal 
        while not self.leader.is_at_goal(goal_position):
            velo_vector = self.get_composition_vector(self.leader_id, goal_position)
            self.leader.move_by_velo_vector(velo_vector)
            for id in follower_ids:
                velo_vector = self.get_composition_vector(id, goal_position)
                self.drones[id - 1].move_by_velo_vector(velo_vector)
            self.rate.sleep()
        ## after leader reached goal 
        reached_goals = 1
        goal_positions = numpy.empty(self.number_of_drones, dtype=Point)
        for id in follower_ids:
            goal_positions[id - 1] = self.get_formation_position(id) 
        while reached_goals < self.number_of_drones:
            for id in follower_ids:
                if not self.drones[id - 1].is_at_goal(goal_positions[id - 1]): 
                    velo_vector = self.get_composition_vector(id, goal_positions[id - 1])
                    self.drones[id - 1].move_by_velo_vector(velo_vector)
                else:
                    reached_goals += 1
            self.rate.sleep()

    def get_composition_vector(self, i, goal_position):
        oa_vector = self.drones[i-1].get_oa_vector()
        r_vector = self.get_sum_r_vector(i)
        ca_vector = self.get_sum_ca_vectors(i)
        if i == self.leader_id:
            mtg_vector = self.leader.get_mtg_vector(goal_position)
            composition_vector = add_vector(oa_vector, r_vector, ca_vector, mtg_vector)
        else:
            f_vector = self.get_f_vector(i)
            composition_vector = add_vector(oa_vector, r_vector, ca_vector, f_vector)
        return composition_vector      
    
    def get_ca_vector(self, i, j):
        '''Get jth drone's collision avoidance vector on ith drone
        i and j is not in the same wing'''
        ith_position = self.drones[i-1].pose.position
        jth_position = self.drones[j-1].pose.position
        ith_to_jth_vector = get_2D_vector(ith_position, jth_position)
        drones_distance = get_norm_2D_vector(ith_to_jth_vector)
        if drones_distance < r_s:
            magnitude = k_c * (math.exp(-b_c * (drones_distance - r_a)) /
                               (r_a - drones_distance))
            ca_vector = mul_vector(get_unit_vector(ith_to_jth_vector), -magnitude)
        else:
            ca_vector = Vector3(0, 0, 0)
        return ca_vector 
    
    def get_sum_ca_vectors(self, i):
        '''Get sum of all collision avoidance vectors on ith drone
        '''
        if i < self.leader_id: 
            opposite_wing_drone_ids = list(range(self.leader_id + 1, 
                                                 self.number_of_drones + 1)) 
        elif i == self.leader_id:
            opposite_wing_drone_ids = list(range(1, 
                                                 self.number_of_drones + 1)).remove(i)
        else:
            opposite_wing_drone_ids = list(range(1, 
                                                 self.leader_id))
        sum_ca_vector = Vector3(0, 0, 0)
        if opposite_wing_drone_ids:
            for id in opposite_wing_drone_ids:
                sum_ca_vector = add_vector(sum_ca_vector, self.get_ca_vector(i, id))
        return sum_ca_vector

    def get_r_vector(self, i, j):
        '''Get self-reconfiguration formation maintainance of jth drone on ith drone. 
        ith and jth drone must be in the same wing'''

        ith_position = self.drones[i-1].pose.position
        jth_position = self.drones[j-1].pose.position
        ith_to_jth_vector = get_2D_vector(ith_position, jth_position)
        drones_distance = get_norm_2D_vector(ith_to_jth_vector)
        desired_distance = abs(i - j) * self.d
        if drones_distance < r_s:
            magnitude = k_r * (abs(drones_distance - desired_distance) ** b_r) \
                          / ((drones_distance - r_a)** 2)
            r_vector = mul_vector(get_unit_vector(ith_to_jth_vector), -magnitude)
        else:
            r_vector = Vector3(0, 0, 0)
        return r_vector

    def get_sum_r_vector(self, i):
        '''Get sum of all self-reconfiguration formation maintainance vectors 
        on ith drone
        '''
        if i < self.leader_id:
            same_wing_drones_ids = list(range(1, self.leader_id + 1)).remove(i)
        elif i == self.leader_id:
            same_wing_drones_ids = list(range(1, 
                                              self.number_of_drones + 1)).remove(i)
        else:
            same_wing_drones_ids = list(range((self.leader_id), 
                                        self.number_of_drones + 1)).remove(i)
        sum_r_vector = Vector3(0, 0, 0)
        if same_wing_drones_ids:
            for id in same_wing_drones_ids:
                sum_r_vector = add_vector(sum_r_vector, self.get_r_vector(i, id))
        return sum_r_vector

    def get_f_vector(self, i):
        '''Get formation maintance vector of ith drone (follower drone)'''
        leader_velo_vector = self.leader.velocity_vector
        desired_position = self.get_formation_position(i)
        drone_to_desired_position_vector = get_2D_vector(self.drones[i-1].pose.position, 
                                                         desired_position)
        f_vector = add_vector(mul_vector(drone_to_desired_position_vector, k_f), leader_velo_vector)
        return f_vector

    def get_formation_position(self, drone_id):
        '''Get formation position of drone (x, y, z)'''
        leader_position = self.leader.pose.position
        leader_yaw = self.leader.yaw

        diff = drone_id - self.leader_id
        if diff < 0:
            follower_to_leader_direction = standardlize_angle(leader_yaw - self.a / 2)
        else:
            follower_to_leader_direction = standardlize_angle(leader_yaw + self.a / 2)
        
        distance_to_leader = abs(diff) * self.d
        x = leader_position.x \
            - distance_to_leader * math.cos(follower_to_leader_direction)
        y = leader_position.y \
            - distance_to_leader * math.sin(follower_to_leader_direction) 
        return Point(x, y, self.leader.height)
    
    def wait_step_finished(self):
        '''Wait until step is finished'''
        drones_finished_step = 0
        while True:
            for i in range(self.number_of_drones):
                if self.drones[i].is_at_goal_pose(self.drones[i].virtual_goal_pose):
                    drones_finished_step += 1
            if drones_finished_step < self.number_of_drones:
                drones_finished_step = 0
            else:
                break

    
            
