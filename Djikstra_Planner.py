'''
import math
import numpy as np
from collections import deque
from dataclasses import dataclass

@dataclass
class Vertex:
    """Represents a vertex in the state lattice graph."""
    l: float  # Longitudinal offset
    r: float  # Lateral offset

    def __hash__(self):
        return hash((self.l, self.r))

    def __eq__(self, other):
        return (self.l, self.r) == (other.l, other.r)

@dataclass
class Edge:
    """Represents an edge (transition) in the state lattice graph."""
    from_vertex: Vertex
    to_vertex: Vertex
    cost: float  # Cost of the transition

class DirectedGraph:
    """Represents the state lattice graph."""
    def __init__(self):
        self.vertices = {}
        self.edges = []

    def add_vertex(self, vertex):
        if vertex not in self.vertices:
            self.vertices[vertex] = len(self.vertices)

    def add_edge(self, edge):
        self.edges.append(edge)

    def get_vertex_index(self, vertex):
        return self.vertices[vertex]

    def get_vertices(self):
        return list(self.vertices.keys())

    def get_edges(self):
        return self.edges

class StateLatticePlanner:
    def __init__(self, vehicle_constraints, lattice_params, global_path):
        self.min_turn_radius = vehicle_constraints['min_turn_radius']
        self.max_lateral_offset = lattice_params['max_lateral_offset']
        self.longitudinal_step = lattice_params['longitudinal_step']
        self.lateral_step = lattice_params['lateral_step']
        self.connected_layers = lattice_params['connected_layers']
        self.global_path = global_path
        self.obstacle_map = None  # Assuming you have a way to get the obstacle map

    def generate_motion_primitives(self):
        """
        Generate a set of kinematically feasible motion primitives in the Frenet coordinate system and
        duplicate them along the global path.
        Returns a directed graph representing the motion primitives.
        """
        graph = DirectedGraph()

        # Generate vertices for the first layer
        first_layer_vertices = self.generate_vertices(0, self.max_lateral_offset, self.lateral_step)
        for vertex in first_layer_vertices:
            graph.add_vertex(vertex)

        # Duplicate the state lattice along the global path
        for s in self.global_path:
            new_layer_vertices = self.generate_vertices(s, self.max_lateral_offset, self.lateral_step)
            for vertex in new_layer_vertices:
                graph.add_vertex(vertex)

            # Connect vertices from the previous layer to the current layer
            for vertex1 in first_layer_vertices:
                for vertex2 in new_layer_vertices:
                    edge = self.generate_edge(vertex1, vertex2)
                    if edge is not None:
                        graph.add_edge(edge)

            first_layer_vertices = new_layer_vertices

        return graph

    def generate_vertices(self, longitudinal_offset, max_lateral_offset, lateral_step):
        """
        Generate vertices for a single layer in the Frenet coordinate system.
        """
        vertices = []
        for lateral_offset in np.arange(-max_lateral_offset, max_lateral_offset + lateral_step, lateral_step):
            vertex = Vertex(l=longitudinal_offset, r=lateral_offset)
            vertices.append(vertex)
        return vertices

    def generate_edge(self, vertex1, vertex2):
        """
        Generate an edge between two vertices if it satisfies the minimum turning radius constraint.
        Returns the edge if feasible, otherwise None.
        """
        # Calculate the curvature between vertex1 and vertex2
        curvature = self.calculate_curvature(vertex1, vertex2)

        if abs(curvature) >= 1.0 / self.min_turn_radius:
            return None

        # Generate the edge (e.g., a polynomial curve) connecting vertex1 and vertex2
        edge = Edge(from_vertex=vertex1, to_vertex=vertex2, cost=self.compute_edge_cost(vertex1, vertex2))
        return edge

    def calculate_curvature(self, vertex1, vertex2):
        """
        Calculate the curvature between two vertices.
        """
        dl = vertex2.l - vertex1.l
        dr = vertex2.r - vertex1.r
        curvature = dr / dl
        return curvature

    def compute_edge_cost(self, vertex1, vertex2):
        """
        Calculate the cost of the edge based on the criteria described in the paper.
        """
        w_safe = self.compute_safety_cost(vertex1, vertex2)
        w_dist = self.compute_distance_cost(vertex1, vertex2)
        w_man = self.compute_maneuvering_cost(vertex1, vertex2)

        k_safe = 7
        k_dist = 0.14
        k_man = 2

        return k_safe * w_safe + k_dist * w_dist + k_man * w_man

    def compute_safety_cost(self, vertex1, vertex2):
        """
        Compute the safety cost based on the distance to the nearest obstacle.
        """
        distance_to_obstacle = self.get_distance_to_nearest_obstacle(vertex1, vertex2)
        if distance_to_obstacle < 2.0:  # Adjust the threshold as needed
            return 1.0 / (distance_to_obstacle ** 2)
        else:
            return 0.0

    def compute_distance_cost(self, vertex1, vertex2):
        """
        Compute the distance cost based on the deviation from the global path.
        """
        global_path_deviation = self.calculate_global_path_deviation(vertex1, vertex2)
        return global_path_deviation

    def compute_maneuvering_cost(self, vertex1, vertex2):
        """
        Compute the maneuvering cost based on the change in lateral position.
        """
        lateral_position_change = abs(vertex2.r - vertex1.r)
        return lateral_position_change

    def get_distance_to_nearest_obstacle(self, vertex1, vertex2):
        """
        Compute the distance to the nearest obstacle between vertex1 and vertex2.
        This method should use the local occupancy grid map to determine the distance.
        """
        print(vertex1, vertex2)
        # Implement the logic to get the distance to the nearest obstacle
        # using the local occupancy grid map
        # You can use a raytracing algorithm or other methods to calculate the distance
        raise NotImplementedError

    def calculate_global_path_deviation(self, vertex1, vertex2):
        """
        Calculate the deviation of the edge between vertex1 and vertex2 from the global path.
        """
        # Implement the logic to calculate the deviation from the global path
        # You can use the Frenet coordinate system or other methods
        raise NotImplementedError

def dijkstra(graph, start_vertex, end_vertex, k_man, k_dist, k_safe):
    """
    Implement Dijkstra's algorithm to find the minimum-cost path in the state lattice graph.
    """
    queue = deque([(0, start_vertex)])
    visited = set()
    came_from = {}
    cost_so_far = {}

    came_from[start_vertex] = None
    cost_so_far[start_vertex] = 0

    while queue:
        _, current_vertex = queue.popleft()

        if current_vertex == end_vertex:
            break

        if current_vertex in visited:
            continue

        visited.add(current_vertex)

        for edge in [edge for edge in graph.get_edges() if edge.from_vertex == current_vertex]:
            new_cost = cost_so_far[current_vertex] + edge.cost
            if edge.to_vertex not in cost_so_far or new_cost < cost_so_far[edge.to_vertex]:
                cost_so_far[edge.to_vertex] = new_cost
                priority = new_cost
                queue.append((priority, edge.to_vertex))
                came_from[edge.to_vertex] = current_vertex

    # Reconstruct the minimum-cost path
    path = []
    current_vertex = end_vertex
    while current_vertex != start_vertex:
        path.append(current_vertex)
        current_vertex = came_from[current_vertex]
    path.append(start_vertex)
    path.reverse()

    return path

def main():
    # Example usage
    vehicle_constraints = {
        'min_turn_radius': 1.0,
    }

    lattice_params = {
        'max_lateral_offset': 2.0,
        'longitudinal_step': 2.0,
        'lateral_step': 0.5,
        'connected_layers': [2, 3, 5, 7],
    }

    # Example global path
    global_path = [0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0]

    planner = StateLatticePlanner(vehicle_constraints, lattice_params, global_path)
    state_lattice = planner.generate_motion_primitives()

    start_vertex = Vertex(l=0.0, r=0.0)
    end_vertex = Vertex(l=global_path[-1], r=0.0)

    min_cost_path = dijkstra(state_lattice, start_vertex, end_vertex, k_man=2, k_dist=0.14, k_safe=7)

    print("Minimum-cost path:")
    for vertex in min_cost_path:
        print(f"({vertex.l:.2f}, {vertex.r:.2f})")

if __name__ == "__main__":
    main()


'''



"""
Frenet optimal trajectory generator

author: Atsushi Sakai (@Atsushi_twi)

Ref:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame]
(https://www.youtube.com/watch?v=Cj6tAQe7UCY)

"""


import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import sys
import os
import pathlib
import csv
import json
import utm
import geopy
import time
import itertools
from geopy.distance import geodesic
import networkx as nx
import frenet_optimal_trajectory as fot

sys.path.append(str(pathlib.Path(__file__).parent.parent))

from quintic_polynomials_planner import \
    QuinticPolynomial
import cubic_spline_planner
from transformations import euler_from_quaternion, quaternion_from_euler

SIM_LOOP = 500

# # Parameter Default from Atsushi's Original Code
# MAX_SPEED = 100.0 / 3.6  # maximum speed [m/s]
# MAX_ACCEL = 2.0  # maximum acceleration [m/ss]
# MAX_CURVATURE = 1.0  # maximum curvature [1/m]
# MAX_ROAD_WIDTH = 20.0  # maximum road width [m]
# D_ROAD_W = 1.0  # road width sampling length [m]
# DT = 0.2  # time tick [s]
# MAX_T = 5.0  # max prediction time [m]
# MIN_T = 4.0  # min prediction time [m]
# TARGET_SPEED = 30.0 / 3.6  # target speed [m/s]
# D_T_S = 5.0 / 3.6  # target speed sampling length [m/s]
# N_S_SAMPLE = 1  # sampling number of target speed
# ROBOT_RADIUS = 1.0  # robot radius [m]



# # Parameters tuned for the JEEP without Obstacle Avoidance
# MAX_SPEED = 100.0 / 3.6  # maximum speed [m/s]
# MAX_ACCEL = 2.0  # maximum acceleration [m/ss]  ------------ Change as desired
# MAX_CURVATURE = 1.0  # maximum curvature [1/m]  
# MAX_ROAD_WIDTH = 6.0  # maximum road width [m]  -------------- Can be decreased to reduce search space of calc_frenet_paths() and thus reduce computation_time
# D_ROAD_W = 1.0  # road width sampling length [m] --------------- Can be increased to reduce search space of calc_frenet_paths() and thus reduce computation_time
# DT = 0.12  # time tick [s]
# MAX_T = 5.0  # max prediction time [m]
# MIN_T = 4.0  # min prediction time [m]
# TARGET_SPEED = 5 # 20 / 3.6  # target speed [m/s]    ------------ Change as desired
# D_T_S = 5.0 / 3.6  # target speed sampling length [m/s]
# N_S_SAMPLE = 1  # sampling number of target speed
# ROBOT_RADIUS = 1.0  # robot radius [m]


# Parameters tuned for the JEEP for 
# OBSTACLE AVOIDANCE (We need to plan farther ahead i.e. increase MAX_T as much as posible, 10 seems to be good enough)
MAX_SPEED = 50 #100.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 100.0  # maximum acceleration [m/ss]  ------------ Change as desired
MAX_CURVATURE = 10  # maximum curvature [1/m]  
MAX_ROAD_WIDTH = 6  # maximum road width [m]  -------------- Can be decreased to reduce search space of calc_frenet_paths() and thus reduce computation_time
D_ROAD_W = 1  # road width sampling length [m] --------------- Can be increased to reduce search space of calc_frenet_paths() and thus reduce computation_time
DT = 0.3  # time tick [s]
TARGET_SPEED = 10 # 75 / 3.6  # target speed [m/s]    ------------ Change as desired
Planning_time = 6.0  # max prediction time [m]
# MIN_T = 5.0  # min prediction time [m]  
D_T_S = 1 #TARGET_SPEED/4 # 5.0 / 3.6  # target speed sampling length [m/s]      
N_S_SAMPLE = 0.1  # sampling number of target speed   --------------Decrease to reduce computation time
ROBOT_RADIUS = 5.0  # robot radius [m]
G = nx.DiGraph()


# cost weights from Atsushi's Original Code
K_J = 0.1
K_T = 0.1
K_D = 1.0
K_LAT = 1.0
K_LON = 0

show_animation = True


class QuarticPolynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt





















'''
################ Original code (without using cython) Starts ###################

# MAX_T = 20.0  # max prediction time [m]
# MIN_T = 19.9  # min prediction time [m]
# N_S_SAMPLE = 1  # sampling number of target speed

class FrenetPath:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


def calc_frenet_paths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []

    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

        # Lateral motion planning
        for Ti in np.arange(MIN_T, MAX_T, DT):
            fp = FrenetPath()

            # lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Velocity keeping)
            for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE,
                                TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                tfp = copy.deepcopy(fp)
                lon_qp = QuarticPolynomial(s0, c_speed, c_accel, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1]) ** 2

                tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
                tfp.cv = K_J * Js + K_T * Ti + K_D * ds
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                frenet_paths.append(tfp)

    return frenet_paths


###################### Dr. Talebpour's Speed up version (03/25)


def calc_veh_points(_yaw, _x, _y):
    num_points = len(_x)
    x = np.array(_x)
    y = np.array(_y)
    yaw = np.array(_yaw)

    x = x[:, np.newaxis]
    y = y[:, np.newaxis]
    yaw = yaw[:, np.newaxis]

    key_points = np.array([[-1.0, -1.0], [-1.0, 1.0], [3.0, -1.0], [3.0, 1.0]])
    key_points = key_points[np.newaxis, :, :]
    
    x_rotate = (key_points[:, :, 0] ) * np.cos(yaw) - (key_points[:, :, 1] ) * np.sin(yaw) + x
    y_rotate = (key_points[:, :, 0] ) * np.sin(yaw) + (key_points[:, :, 1] ) * np.cos(yaw) + y

    
    return x_rotate, y_rotate


def check_collision(fp, ob):

    if len(ob) != 0:

        vehicle_points_x, vehicle_points_y = calc_veh_points(fp.yaw, fp.x, fp.y)
        ob = np.array(ob)

        vehicle_points_x = vehicle_points_x[:, :, np.newaxis]
        vehicle_points_y = vehicle_points_y[:, :, np.newaxis]
        ob = ob[np.newaxis, :, :]

        # print(vehicle_points_x)

        diff_x = vehicle_points_x - ob[:, :, 0]
        diff_y = vehicle_points_y - ob[:, :, 1]

        squared_distances = diff_x**2 + diff_y**2
        # print(np.max(squared_distances))
        collision = np.any(squared_distances <= ROBOT_RADIUS ** 2)

        if collision:
            return False
    else:
        return True

    return True


def check_paths(fplist, ob):
    ok_ind = []
    for i, _ in enumerate(fplist):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            continue
        elif any([abs(a) > MAX_ACCEL for a in
                  fplist[i].s_dd]):  # Max accel check
            continue
        elif any([abs(c) > MAX_CURVATURE for c in
                  fplist[i].c]):  # Max curvature check
            continue
        elif not check_collision(fplist[i], ob):
            continue

        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]

################ Original code (without using cython) Ends ###################
'''







#################### Cython Implementation Starts ###########################
import cython

# MAX_T = 20.0  # max prediction time [m]
# MIN_T = 19.9  # min prediction time [m]
# N_S_SAMPLE = 1  # sampling number of target speed

class FrenetPath:
    def __init__(self, t, d, d_d, d_dd, d_ddd, s, s_d, s_dd, s_ddd, cd, cv, cf):
        self.t = t
        self.d = d
        self.d_d = d_d
        self.d_dd = d_dd
        self.d_ddd = d_ddd
        self.s = s
        self.s_d = s_d
        self.s_dd = s_dd
        self.s_ddd = s_ddd
        self.cd = cd
        self.cv = cv
        self.cf = cf
        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []

@cython.inline
def calc_frenet_paths(Ti, c_speed, c_accel, c_d, c_d_d, c_d_dd, s0, ob, min_distance_to_obstacles, csp):
    frenet_paths = []
    c_speed_next, c_accel_next, c_d_next, c_d_d_next, c_d_dd_next, s0_next = [],[],[],[],[],[]
    d_values = np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W)
    #t_values = np.arange(MIN_T, MAX_T, DT)
    tv = TARGET_SPEED# np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S)

    for di in d_values:
        #for Ti in t_values:
        lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
        t_values_inner = np.arange(0.0, Ti, DT)
        d_values_inner = lat_qp.calc_point(t_values_inner)
        d_d_values_inner = lat_qp.calc_first_derivative(t_values_inner)
        d_dd_values_inner = lat_qp.calc_second_derivative(t_values_inner)
        d_ddd_values_inner = lat_qp.calc_third_derivative(t_values_inner)

        #for tv in tv_values:
        lon_qp = QuarticPolynomial(s0, c_speed, c_accel, tv, 0.0, Ti)
        s_values_inner = lon_qp.calc_point(t_values_inner)
        s_d_values_inner = lon_qp.calc_first_derivative(t_values_inner)
        s_dd_values_inner = lon_qp.calc_second_derivative(t_values_inner)
        s_ddd_values_inner = lon_qp.calc_third_derivative(t_values_inner)

        Jp = np.sum(np.power(d_ddd_values_inner, 2))  # square of jerk
        Js = np.sum(np.power(s_ddd_values_inner, 2))  # square of jerk
        ds = (TARGET_SPEED - s_d_values_inner[-1]) ** 2

        cd = K_J * Jp + K_T * Ti + K_D * d_values_inner[-1] ** 2
        cv = K_J * Js + K_T * Ti + K_D * ds
        cf = K_LAT * cd + K_LON * cv

        fp = FrenetPath(t=t_values_inner, d=d_values_inner, d_d=d_d_values_inner,
                                        d_dd=d_dd_values_inner, d_ddd=d_ddd_values_inner,
                                        s=s_values_inner, s_d=s_d_values_inner,
                                        s_dd=s_dd_values_inner, s_ddd=s_ddd_values_inner,
                                        cd=cd, cv=cv, cf=cf)
        
        # fp = calc_global_paths(fp,csp)
        
        #if check_paths(fp, ob, min_distance_to_obstacles):
        frenet_paths.append(fp)
        
        c_speed_next.append(fp.s_d[-1])
        c_accel_next.append(fp.s_dd[-1])
        c_d_next.append(fp.d[-1])
        c_d_d_next.append(fp.d_d[-1])
        c_d_dd_next.append(fp.d_dd[-1])
        s0_next.append(fp.s[-1])

    
    return frenet_paths, (c_speed_next, c_accel_next, c_d_next, c_d_d_next, c_d_dd_next, s0_next)

@cython.inline
def calc_veh_points(_yaw, _x, _y):
    x = np.array(_x)[:, np.newaxis]
    y = np.array(_y)[:, np.newaxis]
    yaw = np.array(_yaw)[:, np.newaxis]

    key_points = np.array([[-1.0, -1.0], [-1.0, 1.0], [3.0, -1.0], [3.0, 1.0]])[np.newaxis, :, :]
   
    x_rotate = (key_points[:, :, 0] * np.cos(yaw) - key_points[:, :, 1] * np.sin(yaw) + x)
    y_rotate = (key_points[:, :, 0] * np.sin(yaw) + key_points[:, :, 1] * np.cos(yaw) + y)

    return x_rotate, y_rotate

@cython.inline
def check_collision(fp, ob, min_distance_to_obstacles):
    if len(ob) == 0:
        return True

    vehicle_points_x, vehicle_points_y = calc_veh_points(fp.yaw, fp.x, fp.y)
    ob = np.array(ob)[np.newaxis, :, :]

    diff_x = vehicle_points_x[:, :, np.newaxis] - ob[:, :, 0]
    diff_y = vehicle_points_y[:, :, np.newaxis] - ob[:, :, 1]

    distances = np.sqrt(diff_x ** 2 + diff_y ** 2)
    # print(np.min(distances))

    if min_distance_to_obstacles < 1.5*(ROBOT_RADIUS + math.sqrt(5)):
        # print("Dividing")
        Collision_Allowance_Parameter = ROBOT_RADIUS/2
    else:
        # print('not dividing')
        Collision_Allowance_Parameter = ROBOT_RADIUS

    '''
    ########################### Writing to CSV File #####################################
    # Convert the attributes of fp to JSON strings
    fp_data = {}
    #for attr in ['t', 'd', 'd_d', 'd_dd', 'd_ddd', 's', 's_d', 's_dd', 's_ddd', 'cd', 'cv', 'cf']:
    for attr in ['x', 'y']:
        value = getattr(fp, attr)
        if isinstance(value, np.ndarray):
            if value.size == 1:
                fp_data[attr] = value.item()
            else:
                fp_data[attr] = value.tolist()
        else:
            fp_data[attr] = value

    fp_str = json.dumps(fp_data)

    # Convert diff_x, diff_y, and distances to lists
    diff_x_list = diff_x.flatten().tolist()
    diff_y_list = diff_y.flatten().tolist()
    distances_list = distances.flatten().tolist()

    # Write data to CSV file
    with open('test.csv', 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([fp_str, diff_x_list, diff_y_list, distances_list])
    ####################################################################################
    '''

    return not np.any(distances < Collision_Allowance_Parameter)

# @cython.inline
# def check_paths(fplist, ob, min_distance_to_obstacles):
#     ok_ind = []
#     # start_time = time.time()

#     for i, _ in enumerate(fplist):
#         if any(abs(v) > limit for v, limit in zip(fplist[i].s_d, [MAX_SPEED] * len(fplist[i].s_d))) or \
#             any(abs(a) > limit for a, limit in zip(fplist[i].s_dd, [MAX_ACCEL] * len(fplist[i].s_dd))) or \
#             any(abs(c) > limit for c, limit in zip(fplist[i].c, [MAX_CURVATURE] * len(fplist[i].c))) or \
#             not check_collision(fplist[i], ob, min_distance_to_obstacles):
#             return False
    
#     # end_time = time.time()
#     # elapsed_time = end_time - start_time
#     # print(elapsed_time)

#     return True


@cython.inline
def check_paths(fplist, ob, min_distance_to_obstacles):
    ok_ind = []
    # start_time = time.time()

    for i, _ in enumerate(fplist):
        if any(abs(v) > limit for v, limit in zip(fplist[i].s_d, [MAX_SPEED] * len(fplist[i].s_d))) or \
           any(abs(a) > limit for a, limit in zip(fplist[i].s_dd, [MAX_ACCEL] * len(fplist[i].s_dd))) or \
           any(abs(c) > limit for c, limit in zip(fplist[i].c, [MAX_CURVATURE] * len(fplist[i].c))) or \
           not check_collision(fplist[i], ob, min_distance_to_obstacles):
            continue
        ok_ind.append(i)

    # end_time = time.time()
    # elapsed_time = end_time - start_time
    # print(elapsed_time)

    return [fplist[i] for i in ok_ind]


######################### Cython Implementation Ends ###############################



































'''
################### Dr. Talebpour's original function is slower but we're trying to speed it up

def check_collision(fp, ob):
    if len(ob) != 0:
        vehicle_points_x, vehicle_points_y = calc_veh_points(fp.yaw, fp.x, fp.y)
        for i in range(len(ob[:, 0])):
            
            for point in range(len(vehicle_points_x)):
                d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
                    for (ix, iy) in zip(vehicle_points_x[point], vehicle_points_y[point])]
                collision = any([di <= ROBOT_RADIUS ** 2 for di in d])

                if collision:
                    return False
    else:
        return True

    return True
###########################
'''






'''
#################### Atsushi's function works fast and does the job for now.

def check_collision(fp, ob):
    for i in range(len(ob[:, 0])):
        d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
             for (ix, iy) in zip(fp.x, fp.y)]

        collision = any([di <= ROBOT_RADIUS ** 2 for di in d])

        if collision:
            return False

    return True 
#########################
'''
















def calc_global_paths(fplist, csp):
    # print(fp)

    if isinstance(fplist, list):
        for fp in fplist:

            # calc global positions
            for i in range(len(fp.s)):
                ix, iy = csp.calc_position(fp.s[i])
                if ix is None:
                    break
                i_yaw = csp.calc_yaw(fp.s[i])
                di = fp.d[i]
                fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
                fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
                fp.x.append(fx)
                fp.y.append(fy)

            # calc yaw and ds
            for i in range(len(fp.x) - 1):
                dx = fp.x[i + 1] - fp.x[i]
                dy = fp.y[i + 1] - fp.y[i]
                fp.yaw.append(math.atan2(dy, dx))
                fp.ds.append(math.hypot(dx, dy))
            
            # print(fp.yaw)
            fp.yaw.append(fp.yaw[-1])
            fp.ds.append(fp.ds[-1])

            # calc curvature
            for i in range(len(fp.yaw) - 1):
                fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])
    else:
        fp = fplist
        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            i_yaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
            fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.hypot(dx, dy))
        
        # print(fp.yaw)
        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])


    return fplist







class DirectedGraph:
    """Represents the state lattice graph."""
    def __init__(self):
        self.vertices = {}
        self.edges = []

    def add_vertex(self, vertex):
        if vertex not in self.vertices:
            self.vertices[vertex] = len(self.vertices)

    def add_edge(self, edge):
        self.edges.append(edge)

    def get_vertex_index(self, vertex):
        return self.vertices[vertex]

    def get_vertices(self):
        return list(self.vertices.keys())

    def get_edges(self):
        return self.edges




class StateLatticePlanner:
    def __init__(self, vehicle_constraints, lattice_params):
        self.min_turn_radius = vehicle_constraints['min_turn_radius']
        self.max_lateral_offset = lattice_params['max_lateral_offset']
        self.longitudinal_step = lattice_params['longitudinal_step']
        self.lateral_step = lattice_params['lateral_step']
        self.connected_layers = lattice_params['connected_layers']
        
    def generate_motion_primitives(self):
        """
        Generate a set of kinematically feasible motion primitives in the Frenet coordinate system.
        Returns a directed graph representing the motion primitives.
        """
        motion_primitives = DirectedGraph()
        
        # Generate vertices for the first layer
        first_layer_vertices = generate_vertices(0, self.max_lateral_offset, self.lateral_step)
        for vertex in first_layer_vertices:
            motion_primitives.add_vertex(vertex)
        
        # Generate subsequent layers and connect vertices
        for layer_idx in self.connected_layers:
            new_layer_vertices = generate_vertices(layer_idx * self.longitudinal_step, self.max_lateral_offset, self.lateral_step)
            for vertex in new_layer_vertices:
                motion_primitives.add_vertex(vertex)
            
            # Connect vertices from the previous layer to the current layer
            for vertex1 in motion_primitives.get_vertices(layer_idx - 1):
                for vertex2 in new_layer_vertices:
                    edge = generate_edge(vertex1, vertex2, self.min_turn_radius)
                    if edge is not None:
                        motion_primitives.add_edge(vertex1, vertex2, edge)
                        
        return motion_primitives
    
def generate_vertices(longitudinal_offset, max_lateral_offset, lateral_step):
    """
    Generate vertices for a single layer in the Frenet coordinate system.
    """
    vertices = []
    for lateral_offset in np.arange(-max_lateral_offset, max_lateral_offset + lateral_step, lateral_step):
        vertex = (longitudinal_offset, lateral_offset)
        vertices.append(vertex)
    return vertices

def generate_edge(vertex1, vertex2, min_turn_radius):
    """
    Generate an edge between two vertices if it satisfies the minimum turning radius constraint.
    Returns the edge (e.g., a polynomial curve) if feasible, otherwise None.
    """
    # Calculate the curvature between vertex1 and vertex2
    curvature = calculate_curvature(vertex1, vertex2)
    
    if abs(curvature) >= 1.0 / min_turn_radius:
        return None
    
    # Generate the edge (e.g., a polynomial curve) connecting vertex1 and vertex2
    edge = compute_polynomial_curve(vertex1, vertex2)
    return edge





def frenet_optimal_planning(csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob, min_distance_to_obstacles):
    
    # global TARGET_SPEED 
    # TARGET_SPEED = target_speed
    
    
    # start_time = time.time()
    fplist = calc_frenet_paths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0)
    # end_time = time.time()
    # elapsed_time = end_time - start_time
    # print("Time for calc_frenet_paths(): " + str(elapsed_time))
    

    '''
    vehicle_constraints = {
        'min_turn_radius': 1.0,
    }

    lattice_params = {
        'max_lateral_offset': 2.0,
        'longitudinal_step': 2.0,
        'lateral_step': 0.5,
        'connected_layers': [2, 3, 5, 7],
    }

    lattice_planner = StateLatticePlanner(vehicle_constraints, lattice_params)
    fplist = lattice_planner.generate_motion_primitives()

    print(fplist)
    '''

    # start_time = time.time()
    fplist = calc_global_paths(fplist, csp)
    # end_time = time.time()
    # elapsed_time = end_time - start_time
    # print("Time for calc_global_paths(): " + str(elapsed_time))
    

    # start_time = time.time()
    fplist = check_paths(fplist, ob, min_distance_to_obstacles)
    # end_time = time.time()
    # elapsed_time = end_time - start_time
    # print("Time for check_paths(): " + str(elapsed_time))
    
    
    # start_time = time.time()
    # find minimum cost path
    min_cost = float("inf")
    best_path = None
    for fp in fplist:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp
    # end_time = time.time()
    # elapsed_time = end_time - start_time
    # print("Time for finding minimum cost path(): " + str(elapsed_time))

    return best_path




def generate_frenet_paths(Ti, start_state, max_layers, ob, min_distance_to_obstacles, csp):
    """
    Recursively generate Frenet paths for N layers.
    
    Args:
        start_state (tuple): Initial state (c_speed, c_accel, c_d, c_d_d, c_d_dd, s0)
        max_layers (int): Maximum number of layers to generate
    
    Returns:
        list: List of Frenet paths, where each path is a list of FrenetPath objects.
    """
    paths = []
    
    if max_layers == 1:
        paths_first_layer, next_states = calc_frenet_paths(Ti, *start_state, ob, min_distance_to_obstacles, csp)
        paths.append(paths_first_layer)
        return paths
    
    paths_first_layer, next_states = calc_frenet_paths(Ti, *start_state, ob, min_distance_to_obstacles, csp)
    paths.append(paths_first_layer)
    
    for i in range(len(next_states[0])):
        next_start_state = [next_states[j][i] for j in range(len(next_states))]
        paths_deeper = generate_frenet_paths(Ti, next_start_state, max_layers - 1, ob, min_distance_to_obstacles, csp)
        paths.extend(paths_deeper)
    
    return paths


def djikstra_planner(tx, ty, tyaw, tc, csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob, min_distance_to_obstacles):
    
    '''
    # start_time = time.time()
    fplist = calc_frenet_paths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0)
    # end_time = time.time()
    # elapsed_time = end_time - start_time
    # print("Time for calc_frenet_paths(): " + str(elapsed_time))
    

    # start_time = time.time()
    fplist = calc_global_paths(fplist, csp)
    # end_time = time.time()
    # elapsed_time = end_time - start_time
    # print("Time for calc_global_paths(): " + str(elapsed_time))
    '''
    t_values = [MAX_T]#np.arange(DT, MAX_T, DT)
    paths_second_layer = []
    for Ti in t_values:
        paths_first_layer, c_speed_next, c_accel_next, c_d_next, c_d_d_next, c_d_dd_next, s0_next = calc_frenet_paths(Ti, c_speed, c_accel, c_d, c_d_d, c_d_dd, s0, ob, min_distance_to_obstacles)
        print( "Number of vertices in first layer: ", len(c_speed_next))
        
        for i in range(len(c_speed_next)):
            path_to_append, c_speed_next2, c_accel_next2, c_d_next2, c_d_d_next2, c_d_dd_next2, s0_next2 = calc_frenet_paths(Ti, c_speed_next[i], c_accel_next[i], c_d_next[i], c_d_d_next[i], c_d_dd_next[i], s0_next[i], ob, min_distance_to_obstacles)
            paths_second_layer.extend(path_to_append)
            # for j in range(len(c_accel_next2)):
            #     paths_third_layer, c_speed_next3, c_accel_next3, c_d_next3, c_d_d_next3, c_d_dd_next3, s0_next3 = calc_frenet_paths(Ti, c_speed_next2[j], c_accel_next2[j], c_d_next2[j], c_d_d_next2[j], c_d_dd_next2[j], s0_next2[j])


    # start_time = time.time()
    print(len((paths_second_layer)))
    # first_layer = calc_global_paths(paths_first_layer, csp)
    # for i in range(len(paths_first_layer)):
        
    #     print(first_layer[i].x)
    #     plt.plot(first_layer[i].x,first_layer[i].y)
    # plt.show()
    all_paths = paths_first_layer.copy()
    all_paths.extend(paths_second_layer)
    fplist = calc_global_paths(all_paths, csp)
    for i in range(len(fplist)):
        plt.plot(fplist[i].x,fplist[i].y)
    plt.show()
    # end_time = time.time()
    # elapsed_time = end_time - start_time
    # print("Time for check_paths(): " + str(elapsed_time))
    

    # start_time = time.time()

    # find minimum cost path
    
    
    min_cost = float("inf")
    best_path = None
    for fp in fplist:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp
    # end_time = time.time()
    # elapsed_time = end_time - start_time
    # print("Time for finding minimum cost path(): " + str(elapsed_time))

    return best_path

def generate_target_course(x, y):
    csp = cubic_spline_planner.CubicSpline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp

def generate_points_between(point1, point2, num_points=5):
    x1, y1 = point1
    x2, y2 = point2

    delta_x = (x2 - x1) / (num_points - 1)
    delta_y = (y2 - y1) / (num_points - 1)

    points = [(x1 + i * delta_x, y1 + i * delta_y) for i in range(num_points)]
    
    return points


def read_csv(filename):
    """Get waypoints from csv file."""
    with open(filename, 'r') as csvfile:
        # Skip the headers.
        next(csvfile)
        reader = csv.reader(csvfile)
        nav_path = []
        # CSV contains x,y,z,qx,qy,qz,qw in columns 5-11 and longitudinal
        # velocity in column 48.
        for row in reader:
            nav_path.append([float(row[5]), float(row[6]), float(row[7]),
                             float(row[8]), float(row[9]), float(row[10]),
                             float(row[11]), float(row[48])])
    return nav_path

def space_path(navigation, spatial_resolution=2, stopping_distance=None):
    """Resample the path.

    Converts the provided path to UTM coordinates usable for closed-loop control
    with GPS feedback.

    Args:
        navigation: list(list(float)) a 8 x N list of lists containing the
            global X, Y, and Z position [UTM]; X, Y, Z, and W
            quaternion orientation, and the longitudinal velocity [m/s].
        spatial_resolution: the desired distance [m] between each waypoint.
        stopping_distance: the desired distance [m] at the end of the path to 
            come to a full stop.

    Returns:
        numpy array of M x 5, where M is the number of points after
        down-sampling to the desired spatial resolution. The columns are global
        X and Y position [UTM], yaw angle, longitudinal velocity, and distance
        along path.
    """
    # Convert X,Y to latitude and longitude from UTM coordinates. This is
    # specific to College Station, Texas.
    path_latlng = []
    for waypoint in navigation:
        latlon = utm.to_latlon(waypoint[0], waypoint[1], 10, 'U')
        yaw = yaw_from_quat(waypoint[3], waypoint[4], waypoint[5], waypoint[6])
        path_latlng.append([latlon[0], latlon[1], yaw, waypoint[7]])
    prev_point = (path_latlng[0][0], path_latlng[0][1])

    # Down-sample path to the desired distance between points.
    x, y,_,_ = utm.from_latlon(prev_point[0],prev_point[1])
    new_path_latlng = [[x, y, path_latlng[0][2], path_latlng[0][3], 0]]
    total_distance = 0
    for waypoint in path_latlng:
        curr_point = (waypoint[0], waypoint[1])
        # Compute distance using ellipsoidal model of the Earth.
        # TODO: Re-evaluate if this is necessary. Without considering the z
        # coordinate, this distance may not be very accurate. For flat surfaces
        # this is probably acceptable, but is the additional computational cost
        # worth the improvement on distance accuracy?
        distance = geopy.distance.geodesic(curr_point, prev_point).m
        if spatial_resolution and distance >= spatial_resolution:
            # Include distance between points to help with scheduling.
            total_distance += distance
            x, y,_ ,_ = utm.from_latlon(curr_point[0],curr_point[1])
            new_path_latlng.append(
                [x, y, waypoint[2], waypoint[3], total_distance])
            prev_point = curr_point
        elif not spatial_resolution:
            # Include distance between points to help with scheduling.
            total_distance += distance
            x, y,_ ,_ = utm.from_latlon(curr_point[0],curr_point[1])
            new_path_latlng.append(
                [x, y, waypoint[2], waypoint[3], total_distance])
            prev_point = curr_point

    new_path_latlng = np.array(new_path_latlng)

    # Overwrite the velocity of the last of waypoints to command the
    # vehicle to come to a full stop.
    if stopping_distance:
        total_distance = new_path_latlng[-1, 4]
        begin_stop_distance = total_distance - stopping_distance
        i_stop = np.argmin(abs(new_path_latlng[:, 4] - begin_stop_distance))
        begin_stop_velocity = new_path_latlng[i_stop, 3]
        decel_per_i = begin_stop_velocity / (len(new_path_latlng) - i_stop)
        for i in range(i_stop, len(new_path_latlng)):
            new_path_latlng[i, 3] -= (i - i_stop + 1) * decel_per_i
        new_path_latlng[-1, 3] = 0

    return new_path_latlng

def yaw_from_quat(x, y, z, w):
    _, _, yaw = euler_from_quaternion([x, y, z, w])
    return yaw

def main():
    print(__file__ + " start!!")

    # way points
    # wx = [100000.0, 100001.0, 100002.0, 100004, 100005.0, 100007.0, 100010.0]
    # wy = [200000.0, 200002.0, 200005.0, 200007.0, 200020.0, 200023.0, 200030.0]
    # obstacle lists
    # ob = np.array([ [75,-135] , [25,-60]]) 
    # ob = np.array([[741060, 3391100], [741059, 3391200]]) ### for pr_0409
    ob = np.array([[741688, 3391700], [741686.5, 3391800]]) ### for 0322_straight.csv

    # ob = np.array([[45,-85] , [25,-58]])
    # plt.plot(ob[:,0], ob[:,1], '*')
    # plt.show()
    # ob = np.concatenate(ob, rectangle_corners)
    # np.array([[50.0, 2.5],
    #             #    [50.0, 0],
    #                [50.0, -2.5],
    #                [55.0, 2.5],
    #             #    [55.0, 0],
    #                [55.0, -2.5],
    #             #    [0.0, 5.0], [10.0, 5.0], [20.0, 5.0], [30.0, 5.0], [40.0, 5.0],
    #             #    [0.0, -10.0], [10.0, -10.0], [20.0, -10.0], [30.0, -10.0], [40.0, -10.0]
    #                ])
    '''
    rectangle_corners = np.array([[1,2], [1, -2], [-1, 2], [-1,-2]])
    
    tmp = []
    for i in rectangle_corners:
        tmp.append([np.cos(np.pi/4) * i[0] + np.sin(np.pi/4) * i[1], -np.sin(np.pi/4) * i[0] + np.cos(np.pi/4) * i[1]])
    rectangle_corners = np.array([[5,4], [5, 0], [3, 4], [3,0]])
    for i in rectangle_corners:
        tmp.append(i)
    rectangle_corners = np.array(tmp)
    '''

    # filename = '/home/avalocal/AVA_Planner-main/data/gravel.csv'
    filename = '/home/avalocal/AVA_Planner-main/data/Straight322.csv'
    # filename = '/home/avalocal/AVA_Planner-main/data/pr_0409.csv'
    if os.path.isfile(filename):
        path_utm = read_csv(filename)

    g_global_path = space_path(path_utm)

    refx = np.array(g_global_path)[0, 0].tolist()
    refy = np.array(g_global_path)[0, 1].tolist()
    wx = [x  for x in np.array(g_global_path)[:, 0].tolist()]
    wy = [y  for y in np.array(g_global_path)[:, 1].tolist()]
    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)
        #np.array(path_utm)[1:10, 0].tolist(), np.array(path_utm)[1:10, 1].tolist())
    # print(csp)

    # initial states
    c_speed = 10.0 #0 / 3.6  # current speed [m/s]
    c_accel = -0.0  # current acceleration [m/ss]
    c_d = 0.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.0  # current course position

    area = 20.0  # animation area length [m]




    # for i in range(SIM_LOOP):


    # distances = [ np.sqrt(np.sum((position - obstacle)**2)) for obstacle in ob]
    # min_distance_index = np.argmin(distances)
    min_distance_to_obstacles = 0 # distances[min_distance_index]
    number_of_layers = 4 #6
    start_time = time.time()

    # path = djikstra_planner(
    #     tx, ty, tyaw, tc, csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob, min_distance_to_obstacles)
    t_values = [Planning_time]#np.arange(DT, MAX_T, DT)

    
    for Ti in t_values:
        ### path will be a list of lists. [ [first_layer_paths] , [second_layer_paths] , ...... ]
        path = generate_frenet_paths(Ti, (c_speed, c_accel, c_d, c_d_d, c_d_dd, s0) , number_of_layers, ob, min_distance_to_obstacles, csp)
    
    fig, ax = plt.subplots(figsize=(12, 8))
    layer = 0

    last_nodes = []
    # print(np.shape(path))
    path_mapping = {}
    start_end_pairs = []
    for p in path:
        layer += 1
        global_paths = calc_global_paths(p,csp)
        fp = check_paths(global_paths, ob, min_distance_to_obstacles)



        if layer == 1:
            start_node = (round(fp[0].x[0],1), round(fp[0].y[0],1))

        for path_inner in fp:
            ## Adding end points of the path in the Graph 
            
            ### W/o rounding
            # start_point = (path_inner.x[0], path_inner.y[0])
            # end_point = (path_inner.x[-1], path_inner.y[-1])
            
            ### With rounding
            start_point = (round(path_inner.x[0], 1), round(path_inner.y[0], 1))
            end_point = (round(path_inner.x[-1], 1), round(path_inner.y[-1], 1))
            
            # start_end_pairs.append((start_point, end_point, path_inner))
            path_mapping[(start_point, end_point)] = path_inner
            
            

            if layer == number_of_layers:
                
                ## We're in the last layer

                # Add start_point to the graph if it's not already there
                if not G.has_node(start_point):
                    print("Anomaly")
                    G.add_node(start_point)
                    # ax.plot(start_point[0],start_point[1],'bo', markersize=5)

                
                # Add end_point to the graph if it's not already there
                if not G.has_node(end_point):
                    
                    # # Calculate squared distances from the point to each point on the trajectory
                    # squared_distances = (end_point[0] - tx)**2 + (end_point[1] - ty)**2
                    
                    # # Find the minimum squared distance
                    # min_squared_distance_point = np.min(squared_distances)
                    
                    # # Update the closest point if the distance is smaller
                    # if min_squared_distance_point < D_ROAD_W:
                    #     print(min_squared_distance_point)
                    G.add_node(end_point)
                    # ax.plot(end_point[0],end_point[1], 'bo', markersize=5)
                    # print(end_point)
                    last_nodes.append(end_point)
                    
                
                # Add edge if it doesn't already exist in the graph
                if not G.has_edge(start_point, end_point) and G.has_node(end_point) and G.has_node(start_point):
                    G.add_edge(start_point, end_point, weight=path_inner.cf)
                    
                    # ax.plot(path_inner.x, path_inner.y, color='gray',LineWidth = 1 , alpha=0.5)
                

            else:
                # Add start_point to the graph if it's not already there
                if not G.has_node(start_point):
                    G.add_node(start_point)
                    # ax.plot(start_point[0],start_point[1],'bo', markersize=5)
                
                # Add end_point to the graph if it's not already there
                if not G.has_node(end_point):
                    # print(end_point)
                    G.add_node(end_point)
                    if end_point[0] < 100 and end_point[0] > 75:
                        print("Anomaly here", layer)
                    # ax.plot(end_point[0],end_point[1], 'bo', markersize=5)
                
                # Add edge if it doesn't already exist in the graph
                if not G.has_edge(start_point, end_point) and G.has_node(end_point) and G.has_node(start_point):
                    G.add_edge(start_point, end_point, weight=path_inner.cf) 

                    # ax.plot(path_inner.x, path_inner.y, color='gray',LineWidth = 0.7 , alpha=0.5)
            
            
            # text_x = path_inner.x[len(path_inner.x)//2]  # X-coordinate for the text annotation (middle of the path)
            # text_y = path_inner.y[len(path_inner.y)//2]  # Y-coordinate for the text annotation (middle of the path)
            # ax.text(text_x, text_y, str(round(path_inner.cf,2)), color='black', fontsize=8, ha='center', va='center')


    points = np.array(last_nodes)
    tx = np.array(tx)
    ty = np.array(ty)
    
    # Initialize variables to store the closest point and its squared distance
    closest_point = None
    min_squared_distance = float('inf')

    # Iterate over each point in the list
    for point in points:
        # Calculate squared distances from the point to each point on the trajectory
        squared_distances = (point[0] - tx)**2 + (point[1] - ty)**2
        
        # Find the minimum squared distance
        min_squared_distance_point = np.min(squared_distances)
        
        # Update the closest point if the distance is smaller
        if min_squared_distance_point < min_squared_distance:
            min_squared_distance = min_squared_distance_point
            closest_point = point
    closest_point = tuple(closest_point)
    end_node = closest_point

    # nodes_to_remove = [node for node in last_nodes if node != closest_point]
    # G.remove_nodes_from(nodes_to_remove)


    print("Number of nodes in G: ", G.number_of_nodes())


    # for obstacle in ob:
    if len(ob)!=0:
        ax.plot(ob[:,0], ob[:,1], 'ro', markersize=10, label = "Obstacles")
    ax.plot(tx[0:2200], ty[0:2200], color='orange', label = "Global Trajectory")
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Shortest Path using Dijkstra')
    ax.legend()
    ax.axis('equal')
    ax.grid()
    # plt.show()
    

    ### Plotting the Graph
    # plt.figure(figsize=(8, 6))
    # nx.draw(G, with_labels=True)
    # plt.title('Directed Graph of Frenet Paths')
    

    min_cost = float('inf')

    print(last_nodes)
    
    
    '''
    ## 1. closest end_node to target trajectory
    points = np.array(last_nodes)
    tx = np.array(tx)
    ty = np.array(ty)
    
    # Initialize variables to store the closest point and its squared distance
    closest_point = None
    min_squared_distance = float('inf')

    # Iterate over each point in the list
    for point in points:
        # Calculate squared distances from the point to each point on the trajectory
        squared_distances = (point[0] - tx)**2 + (point[1] - ty)**2
        
        # Find the minimum squared distance
        min_squared_distance_point = np.min(squared_distances)
        
        # Update the closest point if the distance is smaller
        if min_squared_distance_point < min_squared_distance:
            min_squared_distance = min_squared_distance_point
            closest_point = point
    end_node = tuple(closest_point)
    '''


    
    # print(closest_node)
    shortest_path = nx.dijkstra_path(G, start_node, end_node)


    '''
    for end_node in last_nodes:
        # print(type(nx.dijkstra_path_length(G, start_node, end_node)))
        short_path = nx.dijkstra_path(G, start_node, end_node)
        current_cost = 0

        
        ## 2. cf value of the last segment of shortest path
        # path_object = path_mapping.get((short_path[-2], short_path[-1]))
        # current_cost = path_object.cf
        
        
        ## 3. sum of length of shortest path segments
        for i in range(len(short_path) - 1):
            path_object = path_mapping.get((short_path[i], short_path[i+1]))
            
            # Calculate differences between consecutive coordinates
            dx = np.diff(path_object.x)
            dy = np.diff(path_object.y)
            # Calculate Euclidean distances between consecutive points
            distances = np.sqrt(dx**2 + dy**2)
            # Sum up the distances to get the total path length
            current_cost += np.sum(distances)
        




        # current_cost = nx.astar_path_length(G, start_node, end_node)
        if current_cost < min_cost:
            shortest_path = short_path
            # shortest_path = nx.astar_path(G, start_node, end_node)
            min_cost = current_cost
    '''
    

    print(shortest_path)

    x_values = []
    y_values = []
    yaw_values = []
    velocity_values = []
    s_values = []
    c_values = []

    previous_s = 0.0
    for i in range(len(shortest_path)-1):
        # print(shortest_path[i+1])
        path_object = path_mapping.get((shortest_path[i], shortest_path[i+1]))
        ax.plot(path_object.x,path_object.y, LineWidth = 2 ,color = 'green')#, label= "Shortest Local Path")
        
        # Extend values to lists if path_object is not None
        if path_object is not None:
            x_values.extend(path_object.x)
            y_values.extend(path_object.y)
            yaw_values.extend(path_object.yaw)
            velocity_values.extend(path_object.s_d)
            s_values_current = [s + previous_s for s in path_object.s]
            s_values.extend(s_values_current)
            c_values.extend(path_object.c)
            previous_s = path_object.s[-1]


    from scipy.interpolate import interp1d

    # Define the number of points to interpolate between each pair of elements
    num_interpolation_points = 100

    # Interpolate x, y, yaw, velocity, s, and c values
    x_interpolated = []
    y_interpolated = []
    yaw_interpolated = []
    velocity_interpolated = []
    s_interpolated = []
    c_interpolated = []
    min_length = min(len(x_values), len(y_values),len(yaw_values), len(velocity_values), len(s_values), len(c_values))
    for i in range(min_length - 1):
        x_interp_func = interp1d([0, 1], [x_values[i], x_values[i+1]])
        y_interp_func = interp1d([0, 1], [y_values[i], y_values[i+1]])
        yaw_interp_func = interp1d([0, 1], [yaw_values[i], yaw_values[i+1]])
        velocity_interp_func = interp1d([0, 1], [velocity_values[i], velocity_values[i+1]])
        s_interp_func = interp1d([0, 1], [s_values[i], s_values[i+1]])
        c_interp_func = interp1d([0, 1], [c_values[i], c_values[i+1]])

        t_values = np.linspace(0, 1, num_interpolation_points + 2)[1:-1]  # Exclude endpoints to avoid duplicating points
        x_interp = x_interp_func(t_values)
        y_interp = y_interp_func(t_values)
        yaw_interp = yaw_interp_func(t_values)
        velocity_interp = velocity_interp_func(t_values)
        s_interp = s_interp_func(t_values)
        c_interp = c_interp_func(t_values)

        # Append interpolated values to the lists
        x_interpolated.extend(x_interp)
        y_interpolated.extend(y_interp)
        yaw_interpolated.extend(yaw_interp)
        velocity_interpolated.extend(velocity_interp)
        s_interpolated.extend(s_interp)
        c_interpolated.extend(c_interp)

    # Convert the lists to numpy arrays
    # x_values_interpolated = np.array(x_interpolated)
    # y_values_interpolated = np.array(y_interpolated)
    # yaw_values_interpolated = np.array(yaw_interpolated)
    # velocity_values_interpolated = np.array(velocity_interpolated)
    # s_values_interpolated = np.array(s_interpolated)
    # c_values_interpolated = np.array(c_interpolated)

    # print(np.shape(x_values_interpolated))


    # Write values to CSV file
    with open('local_trajectory_output_Dijkstra.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
    
        # Write header row
        writer.writerow(['x', 'y', 'yaw', 'velocity', 's', 'c'])  # Add more column headers if needed
        print(x_interpolated)
        # print(min(len(x_values), len(y_values), len(yaw_values), len(velocity_values), len(s_values), len(c_values)))

        # Write data rows
        # for i in range(min(len(x_values), len(y_values), len(yaw_values), len(velocity_values), len(s_values), len(c_values))):
        writer.writerow([x_interpolated, y_interpolated, yaw_interpolated, velocity_interpolated, s_interpolated, c_interpolated])
        # writer.writerow([x_values, y_values, yaw_values, velocity_values, s_values, c_values])



    '''
    flat_list = [item for sublist in path for item in sublist]
    global_paths = calc_global_paths(flat_list,csp)

    fp = check_paths(global_paths, ob, min_distance_to_obstacles)
    
    
    ## Plotting
    
    
    # Iterate through the flattened list and plot each path
    # start_index = 0
    for path in fp:
        # end_index = start_index + len(path.s)
        ax.plot(path.x, path.y, color='gray', alpha=0.5)
        # start_index = end_index
    '''


    
    


    '''
    path = frenet_optimal_planning(
        csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob, min_distance_to_obstacles)
    '''
    end_time = time.time()
    elapsed_time = end_time - start_time
    print("Elapsed_Time: ", elapsed_time)
    
    ax.figure.savefig('Shortest_Path_Dijsktra.png')
    plt.show()


    # s0 = path.s[1]
    # c_d = path.d[1]
    # c_d_d = path.d_d[1]
    # c_d_dd = path.d_dd[1]
    # c_speed = path.s_d[1]
    # c_accel = path.s_dd[1]
    ## print(c_speed, c_d, c_d_d, c_accel, c_d_dd)
    # if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
    #     print("Goal")
    #     break

    if show_animation:  # pragma: no cover
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(tx, ty)
        if len(ob) != 0:
            plt.plot(ob[:, 0], ob[:, 1], "xk")
        plt.plot(path.x[1:], path.y[1:], "-or")
        plt.plot(path.x[1], path.y[1], "vc")
        # plt.xlim(path.x[1] - area, path.x[1] + area)
        # plt.ylim(path.y[1] - area, path.y[1] + area)
        plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
        plt.grid(True)
        plt.pause(0.0001)
        # file_name = './plots/simulation' + str(f"{i:02d}") + '.png'
        # plt.savefig(file_name)

    print("Finish")
    if show_animation:  # pragma: no cover
        plt.grid(True)
        plt.pause(0.0001)
        plt.show()


if __name__ == '__main__':
    main()
