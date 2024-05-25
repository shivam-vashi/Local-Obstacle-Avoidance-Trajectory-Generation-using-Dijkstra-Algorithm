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
from geopy.distance import geodesic
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
MAX_CURVATURE = 0.05  # maximum curvature [1/m]  
MAX_ROAD_WIDTH = 6.0  # maximum road width [m]  -------------- Can be decreased to reduce search space of calc_frenet_paths() and thus reduce computation_time
D_ROAD_W = 1  # road width sampling length [m] --------------- Can be increased to reduce search space of calc_frenet_paths() and thus reduce computation_time
DT = 0.1  # time tick [s]
# TARGET_SPEED = 30 # 75 / 3.6  # target speed [m/s]    ------------ Change as desired
MAX_T = 9.0  # max prediction time [m]
MIN_T = 8.0  # min prediction time [m]  
D_T_S = 1 #TARGET_SPEED/4 # 5.0 / 3.6  # target speed sampling length [m/s]      
N_S_SAMPLE = 0.1  # sampling number of target speed   --------------Decrease to reduce computation time
ROBOT_RADIUS = 2.0  # robot radius [m]



# cost weights from Atsushi's Original Code
K_J = 0.1
K_T = 0.1
K_D = 1.0
K_LAT = 1.0
K_LON = 1.0

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
def calc_frenet_paths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []
    d_values = np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W)
    t_values = np.arange(MIN_T, MAX_T, DT)
    tv_values = np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S)

    for di in d_values:
        for Ti in t_values:
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            t_values_inner = np.arange(0.0, Ti, DT)
            d_values_inner = lat_qp.calc_point(t_values_inner)
            d_d_values_inner = lat_qp.calc_first_derivative(t_values_inner)
            d_dd_values_inner = lat_qp.calc_second_derivative(t_values_inner)
            d_ddd_values_inner = lat_qp.calc_third_derivative(t_values_inner)

            for tv in tv_values:
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

                frenet_paths.append(FrenetPath(t=t_values_inner, d=d_values_inner, d_d=d_d_values_inner,
                                               d_dd=d_dd_values_inner, d_ddd=d_ddd_values_inner,
                                               s=s_values_inner, s_d=s_d_values_inner,
                                               s_dd=s_dd_values_inner, s_ddd=s_ddd_values_inner,
                                               cd=cd, cv=cv, cf=cf))

    return frenet_paths

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

    return fplist


def frenet_optimal_planning(csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob, min_distance_to_obstacles, target_speed):
    
    global TARGET_SPEED 
    TARGET_SPEED = target_speed
    
    
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

def calculate_obj(corners):
    # plt.figure()
    # plt.plot(corners[:,0], corners[:,1], 'r*')
    # print(corners[:,0], corners[:,1])

    l = 1.0 #in meters
    object_points = []
    # Corner 1
    corner = max(corners, key=lambda item: item[1])
    target_corner = max(corners, key=lambda item: item[0])
    theta = -math.atan2((target_corner[1] - corner[1]), (target_corner[0] - target_corner[1]))
    obj_right_1 = [corner[0] + l * np.cos(np.pi / 2 - theta), corner[1] + l * np.sin(np.pi / 2 - theta)]
    obj_left_1 = [corner[0] + l * np.cos(np.pi - theta), corner[1] + l * np.sin(np.pi - theta)]
    obj_center_1 = [corner[0] + l * np.cos(3 * np.pi / 4 - theta), corner[1] + l * np.sin(3 * np.pi / 4 - theta)]
    object_points = object_points + [obj_left_1, obj_right_1, obj_center_1]
    # tmp = np.array(object_points)
    # print(tmp[:,0], tmp[:,1])
    # plt.plot(tmp[:,0], tmp[:,1], '*')
    # plt.show()

    # Corner 2
    corner = max(corners, key=lambda item: item[0])
    target_corner = min(corners, key=lambda item: item[1])
    theta = -math.atan2((target_corner[1] - corner[1]), (target_corner[0] - target_corner[1]))
    obj_right_2 = [corner[0] + l * np.cos(np.pi / 2 - theta), corner[1] + l * np.sin(np.pi / 2 - theta)]
    obj_left_2 = [corner[0] + l * np.cos(np.pi - theta), corner[1] + l * np.sin(np.pi - theta)]
    obj_center_2 = [corner[0] + l * np.cos(3 * np.pi / 4 - theta), corner[1] + l * np.sin(3 * np.pi / 4 - theta)]
    object_points = object_points + [obj_left_2, obj_right_2, obj_center_2]

    # Corner 3
    corner = min(corners, key=lambda item: item[1])
    target_corner = min(corners, key=lambda item: item[0])
    theta = -math.atan2((target_corner[1] - corner[1]), (target_corner[0] - target_corner[1]))
    obj_right_3 = [corner[0] + l * np.cos(np.pi / 2 - theta), corner[1] + l * np.sin(np.pi / 2 - theta)]
    obj_left_3 = [corner[0] + l * np.cos(np.pi - theta), corner[1] + l * np.sin(np.pi - theta)]
    obj_center_3 = [corner[0] + l * np.cos(3 * np.pi / 4 - theta), corner[1] + l * np.sin(3 * np.pi / 4 - theta)]
    object_points = object_points + [obj_left_3, obj_right_3, obj_center_3]

    # Corner 4
    corner = min(corners, key=lambda item: item[0])
    target_corner = max(corners, key=lambda item: item[1])
    theta = -math.atan2((target_corner[1] - corner[1]), (target_corner[0] - target_corner[1]))
    obj_right_4 = [corner[0] + l * np.cos(np.pi / 2 - theta), corner[1] + l * np.sin(np.pi / 2 - theta)]
    obj_left_4 = [corner[0] + l * np.cos(np.pi - theta), corner[1] + l * np.sin(np.pi - theta)]
    obj_center_4 = [corner[0] + l * np.cos(3 * np.pi / 4 - theta), corner[1] + l * np.sin(3 * np.pi / 4 - theta)]
    object_points = object_points + [obj_left_4, obj_right_4, obj_center_4]
    
    object_points = object_points + generate_points_between(obj_right_1, obj_left_2)
    object_points = object_points + generate_points_between(obj_right_2, obj_left_3)
    object_points = object_points + generate_points_between(obj_right_3, obj_left_4)
    object_points = object_points + generate_points_between(obj_right_4, obj_left_1)

    return np.array(object_points)

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
    wx = [100000.0, 100001.0, 100002.0, 100004, 100005.0, 100007.0, 100010.0]
    wy = [200000.0, 200002.0, 200005.0, 200007.0, 200020.0, 200023.0, 200030.0]
    # obstacle lists

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
    ob = np.array([[50,-100]])#rectangle_corners #calculate_obj(rectangle_corners)
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

    filename = '/home/avalocal/AVA_Planner-main/data/gravel.csv'
    if os.path.isfile(filename):
        path_utm = read_csv(filename)

    g_global_path = space_path(path_utm)

    refx = np.array(g_global_path)[0, 0].tolist()
    refy = np.array(g_global_path)[0, 1].tolist()
    wx = [x - refx for x in np.array(g_global_path)[:, 0].tolist()]
    wy = [y - refy for y in np.array(g_global_path)[:, 1].tolist()]
    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)
        #np.array(path_utm)[1:10, 0].tolist(), np.array(path_utm)[1:10, 1].tolist())
    # print(csp)

    # initial state
    c_speed = 0.0 #0 / 3.6  # current speed [m/s]
    c_accel = -0.0  # current acceleration [m/ss]
    c_d = 0.2  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.0  # current course position

    area = 20.0  # animation area length [m]

    for i in range(SIM_LOOP):


        # distances = [ np.sqrt(np.sum((position - obstacle)**2)) for obstacle in ob]
        # min_distance_index = np.argmin(distances)
        min_distance_to_obstacles = 0 # distances[min_distance_index]

        start_time = time.time()
        # print((csp))
        path = frenet_optimal_planning(
            csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob, min_distance_to_obstacles)
        end_time = time.time()
        elapsed_time = end_time - start_time
        print(elapsed_time)
        

        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_speed = path.s_d[1]
        c_accel = path.s_dd[1]
        # print(c_speed, c_d, c_d_d, c_accel, c_d_dd)
        if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
            print("Goal")
            break

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
