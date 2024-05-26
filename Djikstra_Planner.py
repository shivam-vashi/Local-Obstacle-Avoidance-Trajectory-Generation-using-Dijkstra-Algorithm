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


MAX_SPEED = 50  # maximum speed [m/s]
MAX_ACCEL = 100.0  # maximum acceleration [m/ss]  ------------ Change as desired
MAX_CURVATURE = 10  # maximum curvature [1/m]  
MAX_ROAD_WIDTH = 6  # maximum road width [m]  -------------- Can be decreased to reduce search space of calc_frenet_paths() and thus reduce computation_time
D_ROAD_W = 1  # road width sampling length [m] --------------- Can be increased to reduce search space of calc_frenet_paths() and thus reduce computation_time
DT = 0.3  # time tick [s]
TARGET_SPEED = 10  # target speed [m/s]    ------------ Change as desired
Planning_time = 6.0  # max prediction time [m]
D_T_S = 1  # target speed sampling length [m/s]      
N_S_SAMPLE = 0.1  # sampling number of target speed   --------------Decrease to reduce computation time
ROBOT_RADIUS = 5.0  # robot radius [m]
G = nx.DiGraph()


# cost weights 
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


import cython

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
    tv = TARGET_SPEED

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


    return not np.any(distances < Collision_Allowance_Parameter)


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


def calc_global_paths(fplist, csp):

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

    # obstacle lists
    # ob = np.array([ [75,-135] , [25,-60]]) 
    # ob = np.array([[741060, 3391100], [741059, 3391200]]) ### for pr_0409
    ob = np.array([[741688, 3391700], [741686.5, 3391800]]) ### for 0322_straight.csv

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


    # initial states
    c_speed = 10.0 #0 / 3.6  # current speed [m/s]
    c_accel = -0.0  # current acceleration [m/ss]
    c_d = 0.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.0  # current course position

    area = 20.0  # animation area length [m]

    min_distance_to_obstacles = 0 # distances[min_distance_index]
    number_of_layers = 4 #6
    start_time = time.time()

    t_values = [Planning_time]

    
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
    
    # print(closest_node)
    shortest_path = nx.dijkstra_path(G, start_node, end_node)
    

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

    end_time = time.time()
    elapsed_time = end_time - start_time
    print("Elapsed_Time: ", elapsed_time)
    
    ax.figure.savefig('Shortest_Path_Dijsktra.png')
    plt.show()


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
