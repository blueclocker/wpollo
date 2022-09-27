#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import time
import cubic_spline_planner
import sat_collision_check

import rospy
import matplotlib.pyplot as plt
import random
import bisect

from can_msgs.msg import delphi_msges
from test_msgs.msg import Test
from lattice_planning.msg import Traj
from rs_perception.msg import PerceptionListMsg
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from fsd_common_msgs.msg import Gnss
# from visualization_msgs import  MarkerArray
from fsd_common_msgs.msg import CarState
from fsd_common_msgs.msg import CarStateDt
from fsd_common_msgs.msg import Trajectory
from fsd_common_msgs.msg import TrajectoryPoint

from nav_msgs.msg import Path

from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D

import time
from nav_msgs.msg import Path
import copy
import cubic_spline_planner

# Parameter
MAX_SPEED = 50.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 6.0  # maximum acceleration [m/ss]
MAX_DEC = -6.0  # maximum  deceleration [m/ss]
MAX_CURVATURE = 1 / 3.5  # maximum curvature [1/m]  TOO LARGE  better to set it 0.2
MAX_ROAD_WIDTH = 7.5  # maximum road width [m]
D_ROAD_W = 0.3  # road width sampling length [m]
DT = 0.20  # time tick [s]
MAXT = 4.2  # max prediction time [s]
MINT = 3.  # min prediction time [s]
TARGET_SPEED = 50.0 / 3.6  # target speed [m/s]
D_T_S = 3 / 3.6  # target speed sampling length [m/s]
N_S_SAMPLE = 1  # sampling number of target speed
ROBOT_RADIUS = 2  # robot radius [m]  ONLY USED IN COLLISION CHECK
SAFEDISTANCE = 20  # basic safe distance between two vehicles [m]
veh_width, veh_len = 2., 4.2  # the length and width of ego car [m]
tau = 2.0  # 车头时距时间常数 [s]
SEGMENT_time = 1.8
TOTAL_TIME = 5.4

count = 0.
D0 = 20
# cost weights
KJ = 0.5
KT = 1.0
KD = 1.0
KLAT = 1.0
KLON = 1.0

last_di = 0.
last_best_path = None

show_animation = True
action = None
last_action = None
MAX_COST_TIME = 0.
a_series = []
v_series = []

obstacle_front_select_distance = TARGET_SPEED * MAXT


class quintic_polynomial:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):
        # calc coefficient of quintic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.xe = xe
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[T ** 3, T ** 4, T ** 5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T ** 2,
                      vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3
        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2

        return xt


class quartic_polynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, T):
        # calc coefficient of quintic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * T ** 2, 4 * T ** 3],
                      [6 * T, 12 * T ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * T,
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


class Frenet_path:

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


class ReferencePath:
    def __init__(self, s, x, y, yaw, curvature, csp):
        self.s = s
        self.x = x
        self.y = y
        self.yaw = yaw
        self.curvature = curvature
        self.csp = csp


class Obstacle:
    def __init__(self, vertexes, ref_paths, lane_number):
        self.vertexes = vertexes
        self.lane_number = lane_number
        # average of all vertex x as center x
        # average of all vertex y as center y
        xs = []
        ys = []
        for vertex in vertexes:
            xs.append(vertex[0])
            ys.append(vertex[1])
        self.x = np.mean(xs)
        self.y = np.mean(ys)
        self.s, self.d = cartesian_to_frenet(self.x, self.y, ref_paths, lane_number)


# max errors of s and d are around 0.1 and 0.05 m
# reference_path_x, reference_path_y, reference_path_yaw can be gained from fcn generate_target_course
# the step like 0.1 to generate list s must equal to the step to generate list s in fcn generate_target_course
def cartesian_to_frenet(xx, yx, ref_paths, lane_number):
    reference_path = ref_paths[lane_number].csp
    reference_path_x = ref_paths[lane_number].x
    reference_path_y = ref_paths[lane_number].y
    reference_path_yaw = ref_paths[lane_number].yaw
    index = len(reference_path_x) - 1
    s = np.arange(0, reference_path.s[-1], 0.1)
    for i in range(len(reference_path_x) - 2):
        dot_pro1 = (xx - reference_path_x[i]) * (reference_path_x[i + 1] - reference_path_x[i]) + \
                   (yx - reference_path_y[i]) * (reference_path_y[i + 1] - reference_path_y[i])
        dot_pro2 = (xx - reference_path_x[i + 1]) * (reference_path_x[i + 2] - reference_path_x[i + 1]) + \
                   (yx - reference_path_y[i + 1]) * (reference_path_y[i + 2] - reference_path_y[i + 1])
        if dot_pro1 * dot_pro2 <= 0:
            index = i + 1
            break
    sr = s[index]
    absolute_d = math.sqrt((xx - reference_path_x[index]) ** 2 + (yx - reference_path_y[index]) ** 2)
    d = np.sign((yx - reference_path_y[index]) * math.cos(reference_path_yaw[index])
                - (xx - reference_path_x[index]) * math.sin(reference_path_yaw[index])) * absolute_d
    return sr, d


def frenet_to_cartesian(sr, d, d_d, ref_path):
    s = np.arange(0, ref_path.csp.s[-1], 0.1)
    xr, yr, theta_r, kr = 0., 0., 0., 0.
    for i in range(len(s) - 1):
        if (s[i] - sr) * (s[i + 1] - sr) <= 0.:
            xr = ref_path.x[i]
            yr = ref_path.y[i]
            theta_r = ref_path.yaw[i]
            kr = ref_path.curvature[i]
            break
    xx = xr - d * math.sin(theta_r)
    yx = yr + d * math.cos(theta_r)
    theta_x = math.atan2(d_d, 1 - kr) + theta_r
    return xx, yx, theta_x


class OtherVehicle:
    def __init__(self, width, length, center_x, center_y, orientation, vx, vy, ref_paths,
                 cur_lane_number):
        self.width = width
        self.length = length
        self.center_x = center_x
        self.center_y = center_y
        self.orientation = orientation
        self.vx = vx
        self.vy = vy

        self.cur_lane_number = cur_lane_number
        self.s, self.d = cartesian_to_frenet(self.center_x, self.center_y, ref_paths, cur_lane_number)

    def update_cartesian_parameters(self, vx, vy, sr, d, d_d, ref_paths, delta_t, cur_lane_number):
        sr = sr + self.vx * delta_t
        d = d + self.vy * delta_t
        ref_path = ref_paths[cur_lane_number]
        xx, yx, theta_x = frenet_to_cartesian(sr, d, d_d, ref_path)
        self.vx = vx
        self.vy = vy
        self.center_x = xx
        self.center_y = yx
        self.orientation = theta_x
        self.cur_lane_number = cur_lane_number


class Behaviour:
    def __init__(self, cur_s, cur_d, cur_speed, cur_lane_num, ref_paths):
        self.cur_s = cur_s
        self.cur_d = cur_d
        self.cur_speed = cur_speed
        self.cur_lane_num = cur_lane_num
        self.ref_path = ref_paths[cur_lane_num]

    # point stop, action0; cruising action1; obstacle_avoidance action2;
    def get_driving_mode(self, obstacles, stop_point_s=None):
        driving_mode = 1
        if not stop_point_s:
            if not obstacles:
                driving_mode = 1
            elif obstacles and abs(obstacles[0].s - self.cur_s) < obstacle_front_select_distance:
                driving_mode = 2
            else:
                driving_mode = 1

        if stop_point_s:
            if stop_point_s - self.cur_s < 20:
                driving_mode = 0
            elif abs(stop_point_s - self.cur_s) >= 20 and not obstacles:
                driving_mode = 1
            elif abs(stop_point_s - self.cur_s) >= 20 and abs(
                    obstacles[0].s - self.cur_s) < obstacle_front_select_distance:
                driving_mode = 2
            else:
                driving_mode = 1
        return driving_mode

    def get_target_v_s_d(self, driving_mode, obstacles, front_25_meters_max_curvature, stop_target_s=None):
        radius_threshold_high = 15.
        radius_threshold_low = 6.
        base_speed = 8. / 3.6
        if driving_mode == 0 and stop_target_s is not None:
            target_v, target_s, target_d = 0., stop_target_s, 0.
        if driving_mode == 1:
            # if front_25_meters_max_curvature < 1/radius_threshold_high:
            #     print('front_25_meters_max_curvature < 1/radius_threshold_high')
            #     target_v, target_s, target_d = TARGET_SPEED, self.cur_s+20, 0.
            # if front_25_meters_max_curvature >= 1/radius_threshold_high:
            #     print('front_25_meters_max_curvature >= 1/radius_threshold_high')
            #     print('front_25_meters_max_curvature >= 1/radius_threshold_high')
            #     turing_radius = 1/front_25_meters_max_curvature
            #     print('turing_radius',turing_radius)
            #     if turing_radius >= radius_threshold_low:
            #         target_v = base_speed + (turing_radius - radius_threshold_low) ** 2 / (radius_threshold_high - radius_threshold_low) **2 * (TARGET_SPEED-base_speed)
            #         target_s, target_d = self.cur_s+20, 0
            #     else:
            #         target_v, target_s, target_d = base_speed,  self.cur_s+20, 0
            target_v = min(math.sqrt(3 / (front_25_meters_max_curvature + 0.001)), TARGET_SPEED)
            target_s, target_d = self.cur_s + 20, 0
            print('target_v', target_v)
        if driving_mode == 2:
            ob_ss = []
            ob_ds = []
            ob_nums = len(obstacles)
            for ob in obstacles:
                ob_ss.append(ob.s)
                ob_ds.append(ob.d)
            ob_ss_min = np.min(ob_ss)
            ob_ds_min = np.min(ob_ds)
            ob_ss_max = np.max(ob_ss)
            ob_ds_max = np.max(ob_ds)
            ob_ss_mean = np.mean(ob_ss)
            ob_ds_mean = np.mean(ob_ds)
            ob_ss_var = np.var(ob_ss)
            ob_ds_var = np.var(ob_ds)
            print('obs_avoiding obs_num', ob_nums)
            if front_25_meters_max_curvature >= 1 / radius_threshold_high:
                target_v = min(math.sqrt(3 / (abs(front_25_meters_max_curvature) + 0.001)), TARGET_SPEED)
                target_s, target_d = self.cur_s + 20, 0
            else:
                target_v, target_s, target_d = max(TARGET_SPEED - ob_nums * 10 / 3.6, 8 / 3.6), self.cur_s + 20, 0
        return target_v, target_s, target_d


# motion planning (cruising)
# c_speed, c_d, c_d_d, c_d_dd, s0, cur_lane_num, lane_num
# 分别是本车当前车速，距离参考路径横向距离，横向速度，横向加速度，本车在参考路径下的纵向位置，当前车道编号，所有车道编号组
def calc_frenet_paths_cruising(csp, c_speed, c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd,
                               cur_lane_num, lane_num, action, target_speed,
                               obstacles, other_vehicles, last_di):
    best_path = None
    print('calc_frenet_paths_cruising target_speed ', target_speed)
    # generate path to each offset goal
    sample_di_nums = 15
    if cur_lane_num == lane_num[0] and cur_lane_num == lane_num[-1]:
        di_list = np.linspace(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              0.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, sample_di_nums)
    elif cur_lane_num == lane_num[0]:
        di_list = np.linspace(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              1.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, sample_di_nums)
    elif cur_lane_num == lane_num[-1]:
        di_list = np.linspace(-1.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              0.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, sample_di_nums)
    else:
        di_list = np.linspace(-1.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              1.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, sample_di_nums)
    di_list = sorted(di_list, key=lambda x: abs(x - 0.45 * last_di))
    print('di_list', di_list)

    maxt = max(20 / target_speed, MAXT)
    if maxt > 6.:
        maxt = 6.
    mint = MINT
    print('maxt, mint', maxt, mint)
    for di in di_list:
        # Lateral motion planning
        for Ti in np.linspace(maxt, mint, 6):
            fp = Frenet_path()

            lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Velocity keeping)
            for tv in [target_speed, target_speed - D_T_S]:
                tfp = copy.deepcopy(fp)
                # xs, vxs, axs, vxe, axe, T
                lon_qp = quartic_polynomial(s0, s0_d, s0_dd, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (target_speed - 0.5 * (tfp.s_d[math.floor(0.5 * len(tfp.s_d))] + tfp.s_d[-1])) ** 2
                if action == 31:
                    tfp.cd = KJ * Jp + KT * Ti + KD * (tfp.d[-1] - MAX_ROAD_WIDTH) ** 2
                elif action == 41:
                    tfp.cd = KJ * Jp + KT * Ti + KD * (tfp.d[-1] + MAX_ROAD_WIDTH) ** 2
                else:
                    tfp.cd = KJ * Jp + KT * Ti + KD * tfp.d[-1] ** 2
                tfp.cv = KJ * Js + KT * Ti + KD * ds
                tfp.cf = KLAT * tfp.cd + KLON * tfp.cv

                if action == 31:
                    tfp.d = [di - MAX_ROAD_WIDTH for di in tfp.d]
                if action == 41:
                    tfp.d = [di + MAX_ROAD_WIDTH for di in tfp.d]
                fplist = calc_global_paths([tfp], csp)
                if check_paths(fplist, veh_width, veh_len, obstacles, other_vehicles):
                    best_path = fplist[0]
                    return best_path, di
    return best_path, last_di


# 用处不大
def get_obs_avoiding_di_list(obstacles, s0, s0_d, target_speed, mint, maxt, di_list_original):
    target_s = (s0_d + target_speed) / 2 * (mint + maxt) / 2
    obstacles_s = []
    if obstacles:
        for ob in obstacles:
            obstacles_s.append(ob.s - target_s)
    index = np.argmin(abs(np.asarray(obstacles_s)))
    closest_ob_d = obstacles[index].d
    if abs(obstacles_s[index]) < 5 and len(obstacles) <= 2:
        di_list = sorted(di_list_original, key=lambda x: -abs(x - closest_ob_d))

    else:
        di_list = di_list_original
    return di_list


def calc_frenet_paths_ob_avoiding(csp, c_speed, c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd,
                                  cur_lane_num, lane_num, action, target_speed,
                                  obstacles, other_vehicles, last_di):
    best_path = None
    print('calc_frenet_paths_ob_avoiding target_speed ', target_speed)
    # generate path to each offset goal
    sample_di_nums = 10
    if cur_lane_num == lane_num[0] and cur_lane_num == lane_num[-1]:
        di_list = np.linspace(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              0.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, sample_di_nums)
    elif cur_lane_num == lane_num[0]:
        di_list = np.linspace(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              1.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, sample_di_nums)
    elif cur_lane_num == lane_num[-1]:
        di_list = np.linspace(-1.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              0.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, sample_di_nums)
    else:
        di_list = np.linspace(-1.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              1.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, sample_di_nums)

    maxt = max(20 / target_speed, MAXT)
    if maxt > 6.:
        maxt = 6.
    mint = MINT
    print('maxt, mint', maxt, mint)
    di_list = sorted(di_list, key=lambda x: abs(x - 0.45 * last_di))
    print('di_list', di_list)
    for Ti in np.linspace(maxt, mint, 5):
        # Lateral motion planning
        for tv in np.linspace(target_speed, 1.5, 4):
            tfp = Frenet_path()
            lon_qp = quartic_polynomial(s0, s0_d, s0_dd, tv, 0.0, Ti)
            tfp.t = [t for t in np.arange(0.0, Ti, DT)]
            tfp.s = [lon_qp.calc_point(t) for t in tfp.t]
            tfp.s_d = [lon_qp.calc_first_derivative(t) for t in tfp.t]
            tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in tfp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in tfp.t]

            # Longitudinal motion planning (Velocity keeping)
            for di in di_list:
                fp = copy.deepcopy(tfp)
                lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

                fp.d = [lat_qp.calc_point(t) for t in fp.t]
                fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
                fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
                fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]
                # xs, vxs, axs, vxe, axe, T

                Jp = sum(np.power(fp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(fp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (target_speed - 0.5 * (fp.s_d[math.floor(0.5 * len(fp.s_d))] + fp.s_d[-1])) ** 2
                if action == 31:
                    fp.cd = KJ * Jp + KT * Ti + KD * (fp.d[-1] - MAX_ROAD_WIDTH) ** 2
                elif action == 41:
                    fp.cd = KJ * Jp + KT * Ti + KD * (fp.d[-1] + MAX_ROAD_WIDTH) ** 2
                else:
                    fp.cd = KJ * Jp + KT * Ti + KD * fp.d[-1] ** 2
                fp.cv = KJ * Js + KT * Ti + KD * ds
                fp.cf = KLAT * fp.cd + KLON * fp.cv
                if action == 31:
                    fp.d = [di - MAX_ROAD_WIDTH for di in fp.d]
                if action == 41:
                    fp.d = [di + MAX_ROAD_WIDTH for di in fp.d]
                fplist = calc_global_paths([fp], csp)
                if check_paths(fplist, veh_width, veh_len, obstacles, other_vehicles):
                    best_path = fplist[0]
                    return best_path, di
    return best_path, last_di


# motion planning (Following)
# 本车在参考路径中的纵向位置，速度，加速度以及前车在参考路径中的纵向位置，速度，加速度
def calc_frenet_paths_following(csp, c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, s_fv0, s_fv_d0, s_fv_dd0,
                                cur_lane_num, lane_num, action, obstacles, other_vehicles):
    best_path = None
    # generate path to each offset goal
    if cur_lane_num == lane_num[0] and cur_lane_num == lane_num[-1]:
        di_list = np.linspace(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              0.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, 10)
    elif cur_lane_num == lane_num[0]:
        di_list = np.linspace(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              1.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, 10)
    elif cur_lane_num == lane_num[-1]:
        di_list = np.linspace(-1.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              0.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, 10)
    else:
        di_list = np.linspace(-1.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              1.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, 10)
    di_list = sorted(di_list, key=abs)
    print('di_list', di_list)
    for di in di_list:

        # Lateral motion planning
        for Ti in np.arange(MAXT, MINT - DT, -DT):
            fp = Frenet_path()

            lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Following)
            # Assuming that the acceleration of front vehicle is a const

            s_fv1 = s_fv0 + s_fv_d0 * Ti + 1 / 2 * s_fv_dd0 * Ti ** 2
            s_fv_d1 = s_fv_d0 + s_fv_dd0 * Ti
            s_fv_dd1 = s_fv_dd0
            s_target = s_fv1 - (D0 + tau * s_fv_d1)
            s_target_d = s_fv_d1 - tau * s_fv_dd1
            s_target_dd = 0.5 * s_fv_dd0 + 0.5 * s_fv_dd1

            if s_target > s0 + s0_d * Ti + 1 / 2 * MAX_ACCEL ** 2 * Ti:
                s_target = s0 + 1 / 2 * MAX_ACCEL ** 2 * Ti
            if s_target_d > MAX_SPEED:
                s_target_d = 0.98 * MAX_SPEED
            if s_target_dd > MAX_ACCEL:
                s_target_dd = 0.98 * MAX_ACCEL
            tfp = copy.deepcopy(fp)
            lon_qp = quintic_polynomial(s0, s0_d, s0_dd, s_target, s_target_d, s_target_dd, Ti)
            tfp.s = [lon_qp.calc_point(t) for t in fp.t]
            tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
            tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

            Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
            Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

            # square of diff from target speed
            ds = (0.5 * (tfp.s_d[math.floor(0.5 * len(tfp.s_d))] + tfp.s_d[-1]) - s_fv_d0) ** 2

            if action == 32:
                tfp.cd = KJ * Jp + KT * Ti + KD * (tfp.d[-1] - MAX_ROAD_WIDTH) ** 2
            elif action == 42:
                tfp.cd = KJ * Jp + KT * Ti + KD * (tfp.d[-1] + MAX_ROAD_WIDTH) ** 2
            else:
                tfp.cd = KJ * Jp + KT * Ti + KD * tfp.d[-1] ** 2
            tfp.cv = 0.2 * KJ * Js + 0.3 * KT * Ti + 0.3 * KD * ds
            tfp.cf = KLAT * tfp.cd + KLON * tfp.cv

            if action == 31:
                tfp.d = [di - MAX_ROAD_WIDTH for di in tfp.d]
            if action == 41:
                tfp.d = [di + MAX_ROAD_WIDTH for di in tfp.d]
            fplist = calc_global_paths([tfp], csp)
            if check_paths(fplist, veh_width, veh_len, obstacles, other_vehicles):
                best_path = fplist[0]
                return best_path

    return best_path


# motion planning (point stopping)
# s_stop 停车点在参考路径中的位置
def calc_frenet_paths_stopping(csp, c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, s_stop,
                               cur_lane_num, lane_num, obstacles, other_vehicles, last_di):
    best_path = None

    # generate path to each offset goal
    sample_di_nums = 8
    if cur_lane_num == lane_num[0] and cur_lane_num == lane_num[-1]:
        di_list = np.linspace(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              0.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, sample_di_nums)
    elif cur_lane_num == lane_num[0]:
        di_list = np.linspace(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              1.8 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, sample_di_nums)
    elif cur_lane_num == lane_num[-1]:
        di_list = np.linspace(-1.8 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              0.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, sample_di_nums)
    else:
        di_list = np.linspace(-1.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              1.8 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, sample_di_nums)

    di_list = sorted(di_list, key=abs)
    for di in di_list:

        # Lateral motion planning
        for Ti in np.arange(MAXT, MINT - DT, -DT):
            fp = Frenet_path()

            lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Following)
            # Assuming that the acceleration of front vehicle is a const
            s_target = s_stop
            s_target_d = 0
            s_target_dd = 0

            if s_target > s0 + s0_d * Ti + 1 / 2 * MAX_DEC ** 2 * Ti:
                s_target = s0 + 1 / 2 * MAX_DEC ** 2 * Ti

            tfp = copy.deepcopy(fp)
            lon_qp = quintic_polynomial(s0, s0_d, s0_dd, s_target, s_target_d, s_target_dd, Ti)
            tfp.s = [lon_qp.calc_point(t) for t in fp.t]
            tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
            tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

            Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
            Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

            # square of diff from target speed
            ds = (tfp.s_d[-1] - 0) ** 2

            tfp.cd = 0.2 * KJ * Jp + 0.3 * KT * Ti + 0.3 * KD * tfp.d[-1] ** 2
            tfp.cv = 0.2 * KJ * Js + 0.3 * KT * Ti + 0.3 * KD * ds
            tfp.cf = KLAT * tfp.cd + KLON * tfp.cv

            if action == 31:
                tfp.d = [di - MAX_ROAD_WIDTH for di in tfp.d]
            if action == 41:
                tfp.d = [di + MAX_ROAD_WIDTH for di in tfp.d]
            fplist = calc_global_paths([tfp], csp)
            if check_paths(fplist, veh_width, veh_len, obstacles, other_vehicles):
                best_path = fplist[0]
                return best_path, di

    return best_path, last_di


def calc_frenet_paths_stopping_cru_avo(csp, c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, max_s_stop,
                                       cur_lane_num, lane_num, obstacles, other_vehicles, last_di):
    best_path = None

    # generate path to each offset goal
    sample_di_nums = 8
    if cur_lane_num == lane_num[0] and cur_lane_num == lane_num[-1]:
        di_list = np.linspace(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              0.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, sample_di_nums)
    elif cur_lane_num == lane_num[0]:
        di_list = np.linspace(-0.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              1.8 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, sample_di_nums)
    elif cur_lane_num == lane_num[-1]:
        di_list = np.linspace(-1.8 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              0.5 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, sample_di_nums)
    else:
        di_list = np.linspace(-1.5 * MAX_ROAD_WIDTH + 0.5 * veh_width,
                              1.8 * MAX_ROAD_WIDTH + D_ROAD_W - 0.5 * veh_width, sample_di_nums)

    di_list = sorted(di_list, key=lambda x: abs(x - 0.45 * last_di))
    for Ti in np.arange(MAXT, MINT - DT, -DT):

        # Lateral motion planning
        for s_target in np.linspace(max_s_stop, s0 + 2, 5):
            s_target_d = 0
            s_target_dd = 0

            if s_target > s0 + s0_d * Ti + 1 / 2 * MAX_DEC ** 2 * Ti:
                s_target = s0 + 1 / 2 * MAX_DEC ** 2 * Ti

            tfp = Frenet_path()
            lon_qp = quintic_polynomial(s0, s0_d, s0_dd, s_target, s_target_d, s_target_dd, Ti)
            tfp.t = [t for t in np.arange(0.0, Ti, DT)]
            tfp.s = [lon_qp.calc_point(t) for t in tfp.t]
            tfp.s_d = [lon_qp.calc_first_derivative(t) for t in tfp.t]
            tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in tfp.t]
            tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in tfp.t]
            for di in di_list:
                fp = copy.deepcopy(tfp)
                lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
                fp.d = [lat_qp.calc_point(t) for t in fp.t]
                fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
                fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
                fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

                # Longitudinal motion planning (Following)
                # Assuming that the acceleration of front vehicle is a const

                # lon_qp = quartic_polynomial(s0, s0_d, s0_dd, 0., 0., Ti)

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (tfp.s_d[-1] - 0) ** 2

                fp.cd = 0.2 * KJ * Jp + 0.3 * KT * Ti + 0.3 * KD * fp.d[-1] ** 2
                fp.cv = 0.2 * KJ * Js + 0.3 * KT * Ti + 0.3 * KD * ds
                fp.cf = KLAT * fp.cd + KLON * fp.cv

                if action == 31:
                    fp.d = [di - MAX_ROAD_WIDTH for di in fp.d]
                if action == 41:
                    fp.d = [di + MAX_ROAD_WIDTH for di in fp.d]
                fplist = calc_global_paths([fp], csp)
                if check_paths(fplist, veh_width, veh_len, obstacles, other_vehicles):
                    best_path = fplist[0]
                    return best_path, di

    return best_path, last_di


def calc_global_paths(fplist, csp):
    for fp in fplist:
        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            iyaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(iyaw + math.pi / 2.0)
            fy = iy + di * math.sin(iyaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.sqrt(dx ** 2 + dy ** 2))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


# todo 只检查障碍物存在的一段轨迹
def check_collision(path, veh_width, veh_len, obstacles, other_vehicles):
    obstacles_vertexes, other_vehicles_vertexes = [], []
    obstacles_s, other_vehicles_s = [], []
    if obstacles:
        for ob in obstacles:
            obstacles_vertexes.append(ob.vertexes)
            obstacles_s.append(ob.s)
    if other_vehicles:
        for veh in other_vehicles:
            other_vehicles_vertexes.append(veh.vertexes)
            other_vehicles_s.append(veh.s)

    vehicle_width = veh_width + 0.2
    vehicle_length = veh_len + 0.4
    collision = False
    path_length = len(path.x)

    obs_vertexes = obstacles_vertexes + other_vehicles_vertexes
    print('num_obs_vertexs', len(obs_vertexes))
    if not obs_vertexes:
        return collision
    if obs_vertexes:
        min_s = min(obstacles_s) - 3
        max_s = max(obstacles_s) + 3
        check_index = []
        check_index.append(0)
        for i in range(len(path.s)):
            last_point_index = check_index[-1]
            if math.sqrt(
                    (path.x[i] - path.x[last_point_index]) ** 2 + (path.y[i] - path.y[last_point_index]) ** 2) > 0.6:
                check_index.append(i)
        if len(path.s) - 1 not in check_index:
            check_index.append(len(path.s) - 1)
        check_index.reverse()
        pop_index = []

        for i in range(len(check_index)):
            if path.s[check_index[i]] < min_s or path.s[check_index[i]] > max_s:
                pop_index.append(i)
        for i in range(len(pop_index) - 1, -1, -1):
            check_index.pop(pop_index[i])

        for i in check_index:
            ego_vertexes = sat_collision_check.ego_vertexes(path.x[i], path.y[i], vehicle_width,
                                                            vehicle_length, path.yaw[i]).vertexes
            collision = sat_collision_check.collision_check_multi_obs(ego_vertexes, obs_vertexes)

            if collision:
                break
        return collision
    return collision


# check velocity, acceleration, curvature
def check_paths(fplist, veh_width, veh_len, obstacles, other_vehicles):
    okind = []
    for i in range(len(fplist)):
        if any([v > MAX_SPEED for v in fplist[i].s_d[0:-1:2]]):  # Max speed check
            print("v max over?, yes")
            continue
        elif any([v > 1.1 * math.sqrt(3 / (abs(c) + 0.001)) for v, c in
                  zip(fplist[i].s_d[0:-1:2], fplist[i].c[0:-1:2])]):
            print(fplist[i].s_d)
            print(fplist[i].c)
            print("cur_speed_limit break?, yes")
            continue
        elif any([abs(a) > MAX_ACCEL for a in fplist[i].s_dd[0:-1:2]]):  # Max accel check
            print("a max over?, yes")
            continue
        elif any([abs(c) > MAX_CURVATURE for c in fplist[i].c[0:-1:2]]):  # Max curvature check
            print("curvature max over?,yes")
            continue
        elif check_collision(fplist[i], veh_width, veh_len, obstacles, other_vehicles):
            print("collision?,yes")
            continue

        okind.append(i)
    return [fplist[i] for i in okind]


#  与当前车道对应的参考路径
# veh_width, veh_len,ob_x, ob_y, ob_len, ob_width, ob_ori
# 车辆宽度，车辆长度，障碍物的x坐标，y坐标，长度，宽度和方向
def frenet_optimal_planning_cruising(csp, s0, s0_d, s0_dd, c_speed, c_d, c_d_d, c_d_dd, veh_width, veh_len,
                                     obstacles, other_vehicles, cur_lane_num, lane_num, action, target_speed):
    print('frenet_optimal_planning_cruising target_speed', target_speed)
    fplist = calc_frenet_paths_cruising \
        (c_speed, c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, cur_lane_num, lane_num, action, target_speed)
    if fplist and action == 31:
        for fp in fplist:
            fp.d = [di - MAX_ROAD_WIDTH for di in fp.d]
    if fplist and action == 41:
        for fp in fplist:
            fp.d = [di + MAX_ROAD_WIDTH for di in fp.d]
    fplist = calc_global_paths(fplist, csp)
    best_path = None
    for fp in fplist:
        if check_paths([fp], veh_width, veh_len, obstacles, other_vehicles):
            best_path = fp
            break
    return best_path


def frenet_optimal_planning_ob_avoiding(csp, s0, s0_d, s0_dd, c_speed, c_d, c_d_d, c_d_dd, veh_width, veh_len,
                                        obstacles, other_vehicles, cur_lane_num, lane_num, action, target_speed):
    print('frenet_optimal_planning_ob_avoiding')
    fplist = calc_frenet_paths_ob_avoiding(
        c_speed, c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, cur_lane_num, lane_num, action, target_speed)
    print('len(fplist)', len(fplist))
    if fplist and action == 31:
        for fp in fplist:
            fp.d = [di - MAX_ROAD_WIDTH for di in fp.d]
    if fplist and action == 41:
        for fp in fplist:
            fp.d = [di + MAX_ROAD_WIDTH for di in fp.d]
    fplist = calc_global_paths(fplist, csp)
    best_path = None
    for fp in fplist:
        print('check fp')
        if check_paths([fp], veh_width, veh_len, obstacles, other_vehicles):
            best_path = fp
            break
    return best_path


def frenet_optimal_planning_following(csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd, s_fv0, s_fv_d0, s_fv_dd0,
                                      veh_width, veh_len, obstacles, other_vehicles, cur_lane_num,
                                      lane_num, action):
    fplist = calc_frenet_paths_following(c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, s_fv0, s_fv_d0, s_fv_dd0,
                                         cur_lane_num, lane_num, action)

    if fplist and action == 32:
        for fp in fplist:
            fp.d = [di - MAX_ROAD_WIDTH for di in fp.d]
    if fplist and action == 42:
        for fp in fplist:
            fp.d = [di + MAX_ROAD_WIDTH for di in fp.d]
    fplist = calc_global_paths(fplist, csp)
    best_path = None
    for fp in fplist:
        if check_paths([fp], veh_width, veh_len, obstacles, other_vehicles):
            best_path = fp
            break
    return best_path


# def frenet_optimal_planning_stopping(csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd, s_stop, veh_width, veh_len,
#                                      obstacles, other_vehicles, cur_lane_num, lane_num):
#     fplist = calc_frenet_paths_stopping(c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, s_stop,
#                                         cur_lane_num, lane_num)
#     fplist = calc_global_paths(fplist, csp)
#     fplist = check_paths(fplist, veh_width, veh_len, obstacles, other_vehicles)
#     best_path = None
#     for fp in fplist:
#         if check_paths([fp], veh_width, veh_len, obstacles, other_vehicles):
#             best_path = fp
#             break
#     return best_path


def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rs, rx, ry, ryaw, rk = [], [], [], [], []
    for i_s in s:
        rs.append(i_s)
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rs, rx, ry, ryaw, rk, csp


def find_index_in_ref_path(ref_path, cur_s):
    ref_path_s = np.asarray(ref_path.s) - cur_s
    index = np.argmin(abs(ref_path_s))
    return index


def prediction(x, y, vx, vy, ref_paths, lane_number, pre_time):
    sr0, d0 = cartesian_to_frenet(x, y, ref_paths, lane_number)
    time_series = np.arange(0.1, pre_time + 0.2, 0.2)
    sr = [sr0 + vx * ti for ti in time_series]
    d = [d0 + vy * ti for ti in time_series]
    d_d = [0 * ti for ti in time_series]
    x, y, theta = [], [], []
    for i in range(len(sr)):
        xx, yx, theta_x = frenet_to_cartesian(sr[i], d[i], d_d[i], ref_paths[lane_number])
        x.append(xx)
        y.append(yx)
        theta.append(theta_x)
    return x, y, theta


def path_pop_first_point(path):
    path.t.pop(0)
    path.d.pop(0)
    path.d_d.pop(0)
    path.d_dd.pop(0)
    path.d_ddd.pop(0)
    path.s.pop(0)
    path.s_d.pop(0)
    path.s_dd.pop(0)
    path.s_ddd.pop(0)
    path.x.pop(0)
    path.y.pop(0)
    path.yaw.pop(0)
    path.ds.pop(0)
    path.c.pop(0)
    return path


def merge_obstacles(obstacles1, obstacles2):
    obstacles = []
    i = j = 0
    while i < len(obstacles1) and j < len(obstacles2):
        if obstacles1[i].s <= obstacles2[j].s:
            obstacles.append(obstacles1[i])
            i = i + 1
        else:
            obstacles.append(obstacles2[j])
            j = j + 1
    if i < len(obstacles1):
        for obstacle in obstacles1[i:]:
            obstacles.append(obstacle)
    if j < len(obstacles2):
        for obstacle in obstacles2[j:]:
            obstacles.append(obstacle)
    return obstacles


def sort_obstacles(obstacles):
    if len(obstacles) <= 1:
        return obstacles
    else:
        middle_index = math.floor(len(obstacles) / 2)
        left_obstacles = sort_obstacles(obstacles[:middle_index])
        right_obstacles = sort_obstacles(obstacles[middle_index:])
    return merge_obstacles(left_obstacles, right_obstacles)


# select the obstacles 40m in front of ego_vehicle and 20m behind ego_vehicle
def select_obstacles(obstacles, ego_vehicle_s, front_distance, behind_distance):
    new_obstacles = copy.deepcopy(obstacles)
    obstacles_num = len(new_obstacles)
    for i in range(len(new_obstacles)):
        if new_obstacles[obstacles_num - 1 - i].s <= ego_vehicle_s - behind_distance or \
                new_obstacles[obstacles_num - 1 - i].s >= ego_vehicle_s + front_distance:
            new_obstacles.pop(obstacles_num - 1 - i)
    return new_obstacles


def merge_vehicles(vehicles1, vehicles2):
    vehicles = []
    i = j = 0
    while i < len(vehicles1) and j < len(vehicles2):
        if vehicles1[i].s <= vehicles2[j].s:
            vehicles.append(vehicles1[i])
            i = i + 1
        else:
            vehicles.append(vehicles2[j])
            j = j + 1
    if i < len(vehicles1):
        for vehicle in vehicles1[i:]:
            vehicles.append(vehicle)
    if j < len(vehicles2):
        for vehicle in vehicles2[j:]:
            vehicles.append(vehicle)
    return vehicles


def sort_vehicles(vehicles):
    if len(vehicles) <= 1:
        return vehicles
    else:
        middle_index = math.floor(len(vehicles) / 2)
        left_vehicles = sort_vehicles(vehicles[:middle_index])
        right_vehicles = sort_vehicles(vehicles[middle_index:])
    return merge_vehicles(left_vehicles, right_vehicles)


def select_vehicles(vehicles, ego_vehicle_s, front_distance, behind_distance):
    new_vehicles = vehicles
    vehicles_num = len(new_vehicles)
    for i in range(len(new_vehicles)):
        if new_vehicles[vehicles_num - 1 - i].sr <= ego_vehicle_s - behind_distance or \
                new_vehicles[vehicles_num - 1 - i].sr >= ego_vehicle_s + front_distance:
            new_vehicles.pop(vehicles_num - 1 - i)
    return new_vehicles


def pop_obstacle(obstacles):
    obstacles.xs.pop(0)
    obstacles.ys.pop(0)
    obstacles.widths.pop(0)
    obstacles.lengths.pop(0)
    obstacles.orientations.pop(0)
    return obstacles


def show_vehicle(vehicle_path, color='black', line_width=1.):
    for i in range(len(vehicle_path.x)):
        if i == 0 or i == 1:
            continue
        vehicle_vertexes = sat_collision_check.ego_vertexes(vehicle_path.x[i], vehicle_path.y[i],
                                                            veh_width, veh_len, vehicle_path.yaw[i]).vertexes
        sat_collision_check.plot_polygon(vehicle_vertexes, color=color, linewidth=line_width)


def show_obstacles(obstacles, color="red", linewidth=1.5):
    if obstacles:
        for i in range(len(obstacles)):
            sat_collision_check.plot_polygon(obstacles[i], color=color, linewidth=linewidth)


def get_other_vehicle_paras(behaviour, lane0_veh, lane1_veh):
    cur_lane_closest_car_s = []
    cur_lane_closest_car_v = []
    left_lane_fcar_s = []
    left_lane_fcar_v = []
    left_lane_rcar_s = []
    left_lane_rcar_v = []
    right_lane_fcar_s = []
    right_lane_fcar_v = []
    right_lane_rcar_s = []
    right_lane_rcar_v = []

    s0 = behaviour.cur_s
    # get left lane and right lane vehicles paras
    if behaviour.cur_lane_num == 0:
        right_lane_fcar_s = []
        right_lane_fcar_v = []
        right_lane_rcar_s = []
        right_lane_rcar_v = []
        if len(lane1_veh) == 0:
            left_lane_fcar_s = []
            left_lane_fcar_v = []
            left_lane_rcar_s = []
            left_lane_rcar_v = []
        elif len(lane1_veh) == 1:
            if lane1_veh[0].sr >= s0:
                left_lane_fcar_s.append(lane1_veh[0].sr)
                left_lane_fcar_v.append(lane1_veh[0].vx)
            else:
                left_lane_rcar_s.append(lane1_veh[0].sr)
                left_lane_rcar_v.append(lane1_veh[0].vx)
        else:
            for i in range(len(lane1_veh)):
                if lane1_veh[i].sr <= s0:
                    left_lane_rcar_s.append(lane1_veh[i].sr)
                    left_lane_rcar_v.append(lane1_veh[i].vx)
                else:
                    left_lane_fcar_s.append(lane1_veh[i].sr)
                    left_lane_fcar_v.append(lane1_veh[i].vx)

    if behaviour.cur_lane_num == 1:
        left_lane_fcar_s = []
        left_lane_fcar_v = []
        left_lane_rcar_s = []
        left_lane_rcar_v = []
        if len(lane0_veh) == 0:
            right_lane_fcar_s = []
            right_lane_fcar_v = []
            right_lane_rcar_s = []
            right_lane_rcar_v = []
        elif len(lane0_veh) == 1:
            if lane0_veh[0].sr >= s0:
                right_lane_fcar_s.append(lane0_veh[0].sr)
                right_lane_fcar_v.append(lane0_veh[0].vx)
            else:
                right_lane_rcar_s.append(lane0_veh[0].sr)
                right_lane_rcar_v.append(lane0_veh[0].vx)
        else:
            for i in range(len(lane0_veh)):
                if lane0_veh[i].sr <= s0:
                    right_lane_rcar_s.append(lane0_veh[i].sr)
                    right_lane_rcar_v.append(lane0_veh[i].vx)
                else:
                    right_lane_fcar_s.append(lane0_veh[i].sr)
                    right_lane_fcar_v.append(lane0_veh[i].vx)
    if behaviour.cur_lane_num == 0:
        if lane0_veh:
            for i in range(len(lane0_veh)):
                if lane0_veh[i].sr > s0:
                    cur_lane_closest_car_s.append(lane0_veh[i].sr)
                    cur_lane_closest_car_v.append(lane0_veh[i].vx)
                    break
    if behaviour.cur_lane_num == 1:
        if lane1_veh:
            for i in range(len(lane1_veh)):
                if lane1_veh[i].sr > s0:
                    cur_lane_closest_car_s.append(lane1_veh[i].sr)
                    cur_lane_closest_car_v.append(lane1_veh[i].vx)
                    break
    return cur_lane_closest_car_s, cur_lane_closest_car_v, left_lane_fcar_s, left_lane_fcar_v, \
           left_lane_rcar_s, left_lane_rcar_v, right_lane_fcar_s, right_lane_fcar_v, \
           right_lane_rcar_s, right_lane_rcar_v


def get_best_path(action, behaviour, cur_lane_num, last_best_path, lane_num, ref_paths, obstacles,
                  other_vehicles,
                  target_point_s, target_point_d, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd,
                  c_speed, front_25_meters_max_curvature, last_di):
    best_path = []
    new_behaviour = behaviour
    if action == 0:  # point stop
        print('get_best_path point stop')
        target_speed, target_s, target_d = \
            behaviour.get_target_v_s_d(action, obstacles,
                                       front_25_meters_max_curvature, stop_target_s=target_point_s)
        print('behaviour,target_speed, target_s, target_d:', target_speed, target_s, target_d)
        csp = ref_paths[cur_lane_num].csp
        # best_path= frenet_optimal_planning_stopping(csp, s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd,
        #                                             target_s, veh_width, veh_len, obstacles,
        #                                             other_vehicles,
        #                                             cur_lane_num, lane_num)
        best_path, di = calc_frenet_paths_stopping(csp, c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, target_s,
                                                   cur_lane_num, lane_num, obstacles, other_vehicles, last_di)
        if not best_path:
            print('get_best_path point stop and no best path')
            if len(last_best_path.s) > 1 and \
                    not check_collision(last_best_path, veh_width, veh_len, obstacles,
                                        other_vehicles):
                best_path = path_pop_first_point(last_best_path)

    elif action == 1:  # cruising
        print('get_best_path cruising')
        csp = ref_paths[cur_lane_num].csp
        target_speed, target_s, target_d = \
            behaviour.get_target_v_s_d(action, obstacles,
                                       front_25_meters_max_curvature, stop_target_s=target_point_s)
        print('behaviour,target_speed, target_s, target_d:', target_speed, target_s, target_d)
        best_path, di = calc_frenet_paths_cruising(csp, c_speed, c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd,
                                                   cur_lane_num, lane_num, action, target_speed,
                                                   obstacles, other_vehicles, last_di)

        if not best_path:
            print('get_best_path cruising no best path')
            best_path = []
            for s_i in np.linspace(target_speed * 4.6, target_speed * 3., 5):
                target_s = s0 + s_i
                best_path, di = calc_frenet_paths_stopping_cru_avo(csp, c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd,
                                                                   target_s, cur_lane_num, lane_num,
                                                                   obstacles, other_vehicles, last_di)
                if best_path:
                    break

    elif action == 2:  # obstacle_avoiding
        print('get_best_path obstacle_avoiding')
        csp = ref_paths[cur_lane_num].csp
        target_speed, target_s, target_d = \
            behaviour.get_target_v_s_d(action, obstacles, front_25_meters_max_curvature,
                                       stop_target_s=target_point_s)
        print('behaviour,target_speed, target_s, target_d:', target_speed, target_s, target_d)
        best_path, di = calc_frenet_paths_ob_avoiding(csp, c_speed, c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd,
                                                      cur_lane_num, lane_num, action, target_speed,
                                                      obstacles, other_vehicles, last_di)
        if not best_path:
            print('get_best_path obstacle_avoiding no best path')
            best_path = []
            for s_i in np.linspace(5, 15, 5):
                target_s = s0 + s_i
                best_path, di = calc_frenet_paths_stopping_cru_avo(csp, c_d, c_d_d, c_d_dd, s0, s0_d, s0_dd, target_s,
                                                                   cur_lane_num, lane_num, obstacles, other_vehicles,
                                                                   last_di)
                if best_path:
                    break
    return best_path, new_behaviour, di


def get_global_path_nearest_yaw(xx, yx, ref_paths, lane_number):
    r_p = ref_paths[lane_number].csp
    r_p_x = ref_paths[lane_number].x
    r_p_y = ref_paths[lane_number].y
    r_p_yaw = ref_paths[lane_number].yaw
    index = len(r_p_x) - 1
    s = np.arange(0, r_p.s[-1], 0.1)
    for i in range(len(r_p_x) - 2):
        dot_pro1 = (xx - r_p_x[i]) * (r_p_x[i + 1] - r_p_x[i]) + \
                   (yx - r_p_y[i]) * (r_p_y[i + 1] - r_p_y[i])
        dot_pro2 = (xx - r_p_x[i + 1]) * (r_p_x[i + 2] - r_p_x[i + 1]) + \
                   (yx - r_p_y[i + 1]) * (r_p_y[i + 2] - r_p_y[i + 1])
        if dot_pro1 * dot_pro2 <= 0:
            index = i + 1
            break
    # print('index',index)
    nearst_yaw = r_p_yaw[index]
    return nearst_yaw


def interplating_and_transforming_curnormal(best_path, veh_ve_vx, start_time, inter_nums, s0):
    big_interval_time = DT
    small_interval_time = big_interval_time / (inter_nums + 1)
    s1 = best_path.s[0]
    best_path.s = list(np.array(best_path.s) - s1 + s0 + 1e-5)
    tx22, ty22, tc22 = best_path.x, best_path.y, best_path.c
    min_len = min(len(tx22), len(ty22), len(tc22))
    tyaw22 = []
    for i in range(min_len - 1):
        tyaw22.append(math.atan2(ty22[i + 1] - ty22[i], tx22[i + 1] - tx22[i]))
    tyaw22.append(tyaw22[-1])
    pub_veh_x = []
    pub_veh_y = []
    pub_veh_yaw = []
    pub_curvature = []
    pub_velocity = []
    pub_yaw_rate = []
    pub_acc = []
    for i in range(min_len - 1):
        pub_veh_x.append(tx22[i])
        pub_veh_y.append(ty22[i])
        pub_veh_yaw.append(tyaw22[i])
        pub_curvature.append(tc22[i])
        for j in range(inter_nums):
            pub_veh_x.append(tx22[i] + (j + 1) / (inter_nums + 1) * (tx22[i + 1] - tx22[i]))
            pub_veh_y.append(ty22[i] + (j + 1) / (inter_nums + 1) * (ty22[i + 1] - ty22[i]))
            pub_veh_yaw.append(tyaw22[i] + (j + 1) / (inter_nums + 1) * (tyaw22[i + 1] - tyaw22[i]))
            pub_curvature.append(tc22[i] + (j + 1) / (inter_nums + 1) * (tc22[i + 1] - tc22[i]))
    pub_veh_x.append(tx22[-1])
    pub_veh_y.append(ty22[-1])
    pub_veh_yaw.append(tyaw22[-1])
    pub_curvature.append(tc22[-1])
    for i in range(min_len - 1):
        pub_velocity.append(
            math.sqrt((tx22[i + 1] - tx22[i]) ** 2 + (ty22[i + 1] - ty22[i]) ** 2) / small_interval_time)
    pub_velocity.append(pub_velocity[-1])
    for i in range(min_len - 1):
        pub_yaw_rate.append((pub_veh_yaw[i + 1] - pub_veh_yaw[i]) / small_interval_time)
    pub_yaw_rate.append(pub_yaw_rate[-1])
    for i in range(min_len - 1):
        pub_acc.append((pub_velocity[i + 1] - pub_velocity[i]) / small_interval_time)
    pub_acc.append(pub_acc[-1])
    pub_best_path_s = []
    for i in range(min_len - 1):
        pub_best_path_s.append(best_path.s[i])
        for j in range(inter_nums):
            pub_best_path_s.append((j + 1) / (inter_nums + 1) * (best_path.s[i + 1] - best_path.s[j]))
    pub_best_path_s.append(best_path.s[-1])
    pub_sum_dist = pub_best_path_s
    # pub_time_diff = list(small_interval_time* np.array(range(len(pub_best_path_s))))
    header = Header(0, rospy.Time.now(), 'world')
    trajectory_points = []
    for i in range(len(pub_velocity)):
        trajectory_points.append(
            TrajectoryPoint(header, Pose2D(pub_veh_x[i], pub_veh_y[i], pub_veh_yaw[i]), pub_curvature[i],
                            pub_velocity[i], pub_yaw_rate[i], pub_acc[i], pub_sum_dist[i], small_interval_time))
    trajectory = Trajectory(header, trajectory_points)

    pub_x = best_path.x
    pub_y = best_path.y
    pub_th = best_path.yaw
    pub_t = list(DT * np.arange(len(best_path.x)))
    distances = [0]
    pub_speeds = [veh_ve_vx]
    for i in range(len(best_path.x) - 1):
        distances.append(
            math.sqrt((best_path.x[i + 1] - best_path.x[i]) ** 2 + (best_path.y[i + 1] - best_path.y[i]) ** 2))
    for i in range(len(distances) - 2):
        pub_speeds.append((distances[i + 1] + distances[i + 2]) / (2 * DT))
    pub_speeds.append(pub_speeds[-1])

    end_time = time.time()
    drop_frame = math.ceil((end_time - start_time) / DT)
    rviz_pub_x = pub_x[drop_frame:-1]
    rviz_pub_y = pub_y[drop_frame:-1]
    rviz_pub_speed = pub_speeds[drop_frame:-1]
    rviz_pub_t = pub_t[drop_frame:-1]
    rviz_pub_th = pub_th[drop_frame:-1]

    return rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th, trajectory


class sub_and_pub():
    def __init__(self):

        self.global_x = []
        self.global_y = []
        self.cur_x = 0.
        self.cur_y = 0.
        self.yaw = 0.
        self.yawrate = 0.
        # self.last_x = 0
        # self.last_y = 0
        self.ve_vx = 0.
        self.ve_vy = 0.
        self.ve_v = 0.
        self.ve_th = 0.
        self.Ax = 0.
        self.Ay = 0.
        self.obs = []
        self.s0 = 0.
        self.c_d = 0.
        self.s0_d = 0.
        self.c_d_d = 0.
        self.s0_dd = 0.
        self.c_d_dd = 0.

        self.path_subscriber = rospy.Subscriber('/path_points', Test, self.global_callback, queue_size=1)
        self.loca_subscriber = rospy.Subscriber('/gnss_odometry', Odometry, self.gps_callback, queue_size=1)
        self.yaw_subscriber = rospy.Subscriber('/gnss_odom', Gnss, self.odom_callback, queue_size=1)
        self.acc_subscriber = rospy.Subscriber('/gnss_imu', Imu, self.acc_callback, queue_size=1)
        self.obs_subscriber = rospy.Subscriber('/rs_percept_result', PerceptionListMsg, self.ob_callback, queue_size=1)
        # self.obs_subscriber = rospy.Subscriber('/delphi_0/delphi_esr', delphi_msges, self.ob_callback, queue_size=1)
        self.road_width_subscriber = rospy.Subscriber('road_width', Test, self.road_width_callback, queue_size=1)

        # FOR RVIZ
        self.traj_publisher = rospy.Publisher('/planning/traj_pts', Traj, queue_size=1)
        # FOR CONTROL
        self.carstate_publisher = rospy.Publisher('/estimation/slam/state', CarState, queue_size=1)
        # FOR CONTROL
        self.trajectory_publisher = rospy.Publisher('/planning/TrajectoryPoint', Trajectory, queue_size=1)

        self.time = time.time()

        self.pub_x = []
        self.pub_y = []
        self.pub_speed = []
        self.pub_t = []
        self.pub_th = []

    def global_callback(self, data_gl):
        assert isinstance(data_gl, Test)
        self.global_x = data_gl.data_x[0:40]
        self.global_y = data_gl.data_y[0:40]

    def odom_callback(self, odom_data):
        assert isinstance(odom_data, Gnss)
        # print('odom_data', odom_data)
        self.yaw = odom_data.yaw + 0.5 * math.pi
        # print('self.yaw', self.yaw)

    def acc_callback(self, imu_data):
        assert isinstance(imu_data, Imu)
        self.Ax = imu_data.linear_acceleration.x
        self.Ay = imu_data.linear_acceleration.y
        self.yawrate = imu_data.angular_velocity.z

    def gps_callback(self, data_gps):
        assert isinstance(data_gps, Odometry)

        x_pos = data_gps.pose.pose.position.x
        y_pos = data_gps.pose.pose.position.y
        t = time.time()
        self.ve_vx = data_gps.twist.twist.linear.x  # (x_pos-self.cur_x)/(t-self.time)
        self.ve_vy = data_gps.twist.twist.linear.y  # data_gps.(y_pos-self.cur_y)/(t-self.time)
        self.ve_v = math.sqrt(self.ve_vx ** 2 + self.ve_vy ** 2)
        self.cur_x = x_pos
        self.cur_y = y_pos
        self.ve_th = math.atan2(self.ve_vy, self.ve_vx)
        self.time = t

        # print('len of time_series', len(TIME_SERIES))

        carstate_header = Header(0., rospy.Time.now(), 'world')
        carstate0 = Pose2D(self.cur_x, self.cur_y, self.yaw)
        carstatedt = CarStateDt(carstate_header, Pose2D(self.ve_vx, self.ve_vy, self.yawrate),
                                Pose2D(self.Ax, self.Ay, 0.))
        self.carstate_publisher.publish(carstate_header, carstate0, carstatedt)

    def road_width_callback(self, data_road_width):
        assert isinstance(data_road_width, Test)
        self.left_width = data_road_width.data_x[0]
        self.right_width = data_road_width.data_y[0]

    def ob_callback(self, data_ob):

        assert isinstance(data_ob, PerceptionListMsg)

        # params initialization
        time1 = time.time()
        # get vehicle params under global frame
        veh_yaw = self.yaw
        veh_cur_x = self.cur_x
        veh_cur_y = self.cur_y
        print('veh_cur_x, veh_cur_y', veh_cur_x, veh_cur_y)
        veh_ve_vx = self.ve_vx
        veh_ve_vy = self.ve_vy
        veh_ve_v = self.ve_v
        c_speed = self.ve_v
        veh_ve_th = self.ve_th
        veh_ax = self.Ax
        veh_ay = self.Ay
        # get ref path
        global_x = self.global_x
        global_y = self.global_y
        ts1, tx1, ty1, tyaw1, tc1, csp1 = generate_target_course(global_x, global_y)

        ref_path = ReferencePath(ts1, tx1, ty1, tyaw1, tc1, csp1)
        ref_paths = [ref_path]
        best_path = None
        lane_num = np.arange(len(ref_paths))
        cur_lane_num = 0

        # get vehicle params under frenet frame
        self.s0, self.c_d = cartesian_to_frenet(veh_cur_x, veh_cur_y, ref_paths, cur_lane_num)
        global_path_yaw = get_global_path_nearest_yaw(veh_cur_x, veh_cur_y, ref_paths, cur_lane_num)
        self.s0_d = veh_ve_vx * math.cos(global_path_yaw) + veh_ve_vy * math.sin(global_path_yaw)
        self.c_d_d = veh_ve_vy * math.cos(global_path_yaw) - veh_ve_vx * math.sin(global_path_yaw)
        self.s0_dd = veh_ax * math.cos(global_path_yaw) + veh_ay * math.sin(global_path_yaw)
        self.c_d_dd = veh_ay * math.cos(global_path_yaw) - veh_ax * math.sin(global_path_yaw)

        s0, c_d, s0_d, c_d_d, s0_dd, c_d_dd = self.s0, self.c_d, self.s0_d, self.c_d_d, self.s0_dd, self.c_d_dd

        if s0_d < 2.5:
            s0_d = 2.5

        c_d_dd = 0.
        print('s0, c_d, s0_d, c_d_d, s0_dd, c_d_dd:', s0, c_d, s0_d, c_d_d, s0_dd, c_d_dd)

        # get road width
        left_road_width = 5.0
        right_road_width = 1.5

        # lateral offset in left side is positive, in right side is negative
        left_sample_di = list(np.arange(0, left_road_width, 0.3))
        right_sample_di = list(np.arange(-0.2, right_road_width, -0.3))
        total_sample_di = sorted(right_sample_di + left_sample_di, key=abs)
        # get obstacles
        self.obs = []

        self.obs = []
        for per_msg in data_ob.perceptions:
            obi_points = per_msg.polygon_point
            vertexes = []
            for obi_point in obi_points:
                x = obi_point.x
                y = obi_point.y
                transfer_x = x * math.cos(veh_yaw) - y * math.sin(veh_yaw) + veh_cur_x
                transfer_y = x * math.sin(veh_yaw) + y * math.cos(veh_yaw) + veh_cur_y
                vertexes.append((transfer_x, transfer_y))
            vertexes.pop(-1)
            print(vertexes)
            obi = Obstacle(vertexes, ref_paths, 0)
            self.obs.append(obi)

        print("num of obstacles", len(self.obs))
        obstacles = self.obs

        other_vehicles = []

        target_point_s = []
        target_point_d = []

        behaviour = Behaviour(s0, c_d, c_speed, cur_lane_num, ref_paths)
        action = behaviour.get_driving_mode(obstacles, target_point_s)
        print('action', action)
        # todo clac front_25_meters_max_curvature
        index1 = find_index_in_ref_path(ref_paths[cur_lane_num], s0)
        index2 = find_index_in_ref_path(ref_paths[cur_lane_num], s0 + 45.)

        curvature_series = abs(np.asarray(ref_paths[cur_lane_num].curvature[index1:index2]))
        front_25_meters_max_curvature = np.max(curvature_series)
        global last_best_path
        global last_di
        global count
        if last_best_path and count < 2 and action == last_action:
            print('last_best_path and count < 6')
            if not check_collision(last_best_path, veh_width, veh_len, obstacles, other_vehicles):
                best_path = path_pop_first_point(last_best_path)
                last_best_path = best_path
                new_behaviour = behaviour
                count = count + 1

            else:
                print('else')
                best_path, new_behaviour, di = get_best_path(action, behaviour, cur_lane_num, last_best_path, lane_num,
                                                             ref_paths, obstacles,
                                                             other_vehicles,
                                                             target_point_s, target_point_d,
                                                             s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd, c_speed,
                                                             front_25_meters_max_curvature, last_di)
                last_best_path = best_path
                last_di = di
                count = 0

        print('behaviour.cur_s', behaviour.cur_s)
        # print('cur_lane_closest_car_s[0]', cur_lane_closest_car_s[0])
        if not last_best_path or count >= 2 or action != last_action:
            print('not last_best_path or count >= 6')
            best_path, new_behaviour, di = get_best_path(action, behaviour, cur_lane_num, last_best_path, lane_num,
                                                         ref_paths, obstacles,
                                                         other_vehicles,
                                                         target_point_s, target_point_d,
                                                         s0, s0_d, s0_dd, c_d, c_d_d, c_d_dd, c_speed,
                                                         front_25_meters_max_curvature, last_di)
            last_best_path = best_path
            last_di = di
            count = 0

        time2 = time.time()
        print('cost time:', time2 - time1)
        start_time = time1

        rviz_pub_x = best_path.x
        rviz_pub_y = best_path.y
        rviz_pub_th = best_path.yaw
        rviz_pub_speed = best_path.s_d
        rviz_pub_t = []
        for i in range(len(rviz_pub_x)):
            rviz_pub_t.append(i * DT)

        rviz_pub_t.append(rviz_pub_t[-1] + DT)
        self.traj_publisher.publish(rviz_pub_x, rviz_pub_y, rviz_pub_speed, rviz_pub_t, rviz_pub_th)
        # plt.clf()
        # for ob in obstacles:
        #     sat_collision_check.plot_polygon(ob.vertexes)
        # plt.plot(best_path.x, best_path.y)
        # plt.axis('equal')
        # plt.show()
        # plt.pause(0.0001)
        # rate = rospy.Rate(10)
        # rate.sleep()


def main():
    rospy.init_node('lattice_node_mbs', anonymous=True)
    sub_and_pub()
    rospy.spin()


if __name__ == '__main__':
    main()
