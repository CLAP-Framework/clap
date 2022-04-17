
import numpy as np
import rospy
import matplotlib.pyplot as plt
import copy
import math

from zzz_planning_decision_continuous_models.Werling.cubic_spline_planner import Spline2D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from zzz_navigation_msgs.msg import Lane
from zzz_driver_msgs.utils import get_speed
from zzz_cognition_msgs.msg import RoadObstacle
from zzz_common.kinematics import get_frenet_state
from zzz_common.geometry import dense_polyline2d, dense_polyline2d_withvelocity
from zzz_planning_msgs.msg import DecisionTrajectory

from zzz_planning_decision_continuous_models.common import rviz_display, convert_ndarray_to_pathmsg, convert_path_to_ndarray
from zzz_planning_decision_continuous_models.predict import predict
from zzz_common.geometry import dist_from_point_to_polyline2d


# Parameter
MAX_SPEED = 50.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 10.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 500.0  # maximum curvature [1/m]
MAX_ROAD_WIDTH = 6.0   # maximum road width [m] # related to RL action space
D_ROAD_W = 3.0  # road width sampling length [m]
DT = 0.3  # time tick [s]
MAXT = 4.6  # max prediction time [m]
MINT = 4.3  # min prediction time [m]
TARGET_SPEED = 20.0 / 3.6  # target speed [m/s]
D_T_S = 10.0 / 3.6  # target speed sampling length [m/s]
N_S_SAMPLE = 2  # sampling number of target speed

# RL paramter
MIN_SPEED_RL = 3/3.6 # min speed for rl planning [m/s]
KICK_IN_TIME = 3 # kick in time for rl action [s]
KICK_IN_NUM = int(KICK_IN_TIME / DT)

# collision check
OBSTACLES_CONSIDERED = 4
ROBOT_RADIUS = 2.0  # robot radius [m]
RADIUS_SPEED_RATIO = 0.5 # higher speed, bigger circle
MOVE_GAP = 1.5
ONLY_SAMPLE_TO_RIGHT = False

# Cost weights
KJ = 0.1
KT = 0.1
KD = 1.0
KLAT = 1.0
KLON = 1.0
KRLS = 1.0


class Werling(object):

    def __init__(self, target_line = 0):

        self.last_trajectory_array = np.c_[0, 0]
        self.last_trajectory = Frenet_path()
        self.last_trajectory_array_rule = np.c_[0, 0]
        self.last_trajectory_rule = Frenet_path()
        self.reference_path = None
        self.ref_path_rviz = None

        self.rviz_all_trajectory = None

        self._dynamic_map = None
        self.ref_path = None
        self.ref_path_tangets = None
        self.ob = None
        self.csp = None
        self.dist_to_end = [0,0,0,0,0]
        self.target_line = target_line

        self.rivz_element = rviz_display()
    
    def clear_buff(self, dynamic_map):

        if self.csp is None:
            return

        self.last_trajectory_array = np.c_[0, 0]
        self.last_trajectory = Frenet_path()
        self.last_trajectory_array_rule = np.c_[0, 0]
        self.last_trajectory_rule = Frenet_path()
        self.reference_path = None
        self.csp = None

        self.rivz_element.candidates_trajectory = None
        self.rivz_element.prediciton_trajectory = None
        self.rivz_element.collision_circle = None
    
    def build_frenet_path(self, dynamic_map,clean_current_csp = False):

        if self.csp is None or clean_current_csp:
            self.reference_path = dynamic_map.jmap.reference_path.map_lane.central_path_points
            ref_path_ori = convert_path_to_ndarray(self.reference_path)
            self.ref_path = dense_polyline2d(ref_path_ori, 2)
            self.ref_path_tangets = np.zeros(len(self.ref_path))
            self.ref_path_rviz = convert_ndarray_to_pathmsg(self.ref_path)

            Frenetrefx = self.ref_path[:,0]
            Frenetrefy = self.ref_path[:,1]
            tx, ty, tyaw, tc, self.csp = self.generate_target_course(Frenetrefx,Frenetrefy)
    
    def trajectory_update(self, dynamic_map):
        if self.initialize(dynamic_map):
            
            start_state = self.calculate_start_state(dynamic_map)
            generated_trajectory = self.frenet_optimal_planning(self.csp, self.c_speed, start_state)
            
            if generated_trajectory is not None:
                k = min(len(generated_trajectory.s_d),5)-1
                desired_speed = generated_trajectory.s_d
                trajectory_array_ori = np.c_[generated_trajectory.x, generated_trajectory.y]
                trajectory_array = trajectory_array_ori#dense_polyline2d(trajectory_array_ori,1)
                self.last_trajectory_array_rule = trajectory_array
                self.last_trajectory_rule = generated_trajectory              
                rospy.logdebug("Planning (continuous): ----> Werling Successful Planning")
            
            elif len(self.last_trajectory_array_rule) > 5 and self.c_speed > 1:
                trajectory_array = self.last_trajectory_array_rule
                generated_trajectory = self.last_trajectory_rule
                desired_speed =  [0] * len(generated_trajectory.s_d)
                rospy.logdebug("Planning (continuous): ----> Werling Fail to find a solution")

            else:
                generated_trajectory =  self.all_trajectory[0][0]
                trajectory_array = np.c_[generated_trajectory.x, generated_trajectory.y]
                desired_speed = [0] * len(trajectory_array)
                rospy.logdebug("Planning (continuous): ----> Werling Output ref path")

            msg = DecisionTrajectory()
            # desired spped is calculate from frenet path, but sometimes frenet path is much longger than real path(spline2D), so we need to cut value in frenet according to th length of spline2D
            desired_speed = desired_speed[:len(trajectory_array)]
            trajectory, velocity_trajectory = dense_polyline2d_withvelocity(trajectory_array, np.array(desired_speed), 0.2)
            msg.trajectory = convert_ndarray_to_pathmsg(trajectory)
            msg.desired_speed = self.ref_tail_speed(dynamic_map, velocity_trajectory)
            self.rivz_element.candidates_trajectory = self.rivz_element.put_trajectory_into_marker(self.all_trajectory)
            self.rivz_element.prediciton_trajectory = self.rivz_element.put_trajectory_into_marker(self.obs_prediction.obs_paths)
            self.rivz_element.collision_circle = self.obs_prediction.rviz_collision_checking_circle
            return msg
        else:
            return None


    def trajectory_update_RLS(self, dynamic_map, RLS_action):
        fplist = self.all_trajectory      
        bestpath = fplist[int(RLS_action - 1)][0]
        trajectory_array = np.c_[bestpath.x, bestpath.y]

        # find minimum cost path
        # mincost = float("inf")
        # bestpath = None
        # for fp in fplist:
        #     RLS_cost = KRLS * (math.fabs(RLS_action[0] - fp.d[KICK_IN_NUM]) + math.fabs(RLS_action[1] - fp.s_d[KICK_IN_NUM]))
        #     if mincost >= (fp.cf + RLS_cost):
        #         mincost = fp.cf + RLS_cost
        #         bestpath = fp

        msg = DecisionTrajectory()
        msg.trajectory = convert_ndarray_to_pathmsg(trajectory_array)
        msg.desired_speed = bestpath.s_d[:len(trajectory_array)]
        print("----> VEG: Successful Planning")           

        return msg

    def initialize(self, dynamic_map):
        self._dynamic_map = dynamic_map

        try:
            # calculate dist to the end of ref path 
            if self.ref_path is not None:
                self.dist_to_end = dist_from_point_to_polyline2d(dynamic_map.ego_state.pose.pose.position.x, dynamic_map.ego_state.pose.pose.position.y, self.ref_path, return_end_distance=True)
                
            # estabilish frenet frame
            if self.csp is None: # or self.dist_to_end[4] < 10 or self.dist_to_end[0] > 20:
                self.build_frenet_path(dynamic_map, True)
                
            # initialize prediction module
            self.obs_prediction = predict(dynamic_map, OBSTACLES_CONSIDERED, MAXT, DT, ROBOT_RADIUS, RADIUS_SPEED_RATIO, MOVE_GAP,
                                        get_speed(dynamic_map.ego_state))
            
            return True

        except:
            rospy.logdebug("Planning (continuous): ------> Werling Initialize fail ")
            return False

    def ref_tail_speed(self, dynamic_map, desired_speed):

        velocity = []
        for i in range(len(desired_speed)):

            if self.ref_path is None:
                velocity.append(0)
                continue

            if self.dist_to_end[4] <= 5:
                velocity.append(0)
                continue

            dec = 0.1

            available_speed = math.sqrt(2*dec*self.dist_to_end[4]) # m/s
            ego_v = get_speed(dynamic_map.ego_state)

            if available_speed > ego_v:
                velocity.append(desired_speed[i])
                continue

            dt = 0.2
            vehicle_dec = (ego_v - available_speed)*10
            tail_speed = ego_v - vehicle_dec*dt

            if desired_speed[i] > tail_speed:
                velocity.append(max(0, tail_speed))

        return velocity

    def calculate_start_state(self, dynamic_map):
        start_state = Frenet_state()

        if len(self.last_trajectory_array_rule) > 5:
            # find closest point on the last trajectory
            mindist = float("inf")
            bestpoint = 0
            for t in range(len(self.last_trajectory_rule.x)):
                pointdist = (self.last_trajectory_rule.x[t] - dynamic_map.ego_state.pose.pose.position.x) ** 2 + (self.last_trajectory_rule.y[t] - dynamic_map.ego_state.pose.pose.position.y) ** 2
                if mindist >= pointdist:
                    mindist = pointdist
                    bestpoint = t
            start_state.s0 = self.last_trajectory_rule.s[bestpoint]
            start_state.c_d = self.last_trajectory_rule.d[bestpoint]
            start_state.c_d_d = self.last_trajectory_rule.d_d[bestpoint]
            start_state.c_d_dd = self.last_trajectory_rule.d_dd[bestpoint]
            # self.c_speed = self.last_trajectory_rule.s_d[bestpoint]
            ego_state = dynamic_map.ego_state
            self.c_speed = get_speed(ego_state) 
        else:
            ego_state = dynamic_map.ego_state
            self.c_speed = get_speed(ego_state)       # current speed [m/s]
            ffstate = get_frenet_state(dynamic_map.ego_state, self.ref_path, self.ref_path_tangets)

            start_state.s0 = ffstate.s 
            start_state.c_d = -ffstate.d # current lateral position [m]
            start_state.c_d_d = ffstate.vd # current lateral speed [m/s]
            start_state.c_d_dd = 0   # current latral acceleration [m/s]
            
        return start_state

    def frenet_optimal_planning(self, csp, c_speed, start_state):
        t0 = rospy.get_rostime().to_sec()

        fplist = self.calc_frenet_paths(c_speed, start_state)
        t1 = rospy.get_rostime().to_sec()
        time_consume1 = t1 - t0
        candidate_len1 = len(fplist)


        fplist = self.calc_global_paths(fplist, csp)
        t2 = rospy.get_rostime().to_sec()
        time_consume2 = t2 - t1
        candidate_len2 = len(fplist)

        # sorted fplist with cost
        path_tuples = []
        for fp in fplist:
            one_path = [fp, fp.cf]
            path_tuples.append(one_path)
        sorted_fplist = sorted(path_tuples, key=lambda path_tuples: path_tuples[1])
        
        self.all_trajectory = sorted_fplist


        sorted_fplist = self.check_paths(sorted_fplist)
        t3 = rospy.get_rostime().to_sec()
        time_consume3 = t3 - t2
        candidate_len3 = len(sorted_fplist)

        rospy.logdebug("Planning (continuous): frenet time consume step1 %.1f(candidate: %d), step2: %.1f(candidate: %d), step3: %.1f(candidate: %d)",
                                            time_consume1, candidate_len1,
                                            time_consume2, candidate_len2,
                                            time_consume3, candidate_len3)

        
        for fp, score in sorted_fplist:
            if self.obs_prediction.check_collision(fp):
                return fp
        
        return None

    def generate_target_course(self, x, y):
        csp = Spline2D(x, y)
        s = np.arange(0, csp.s[-1], 0.1)

        rx, ry, ryaw, rk = [], [], [], []
        for i_s in s:
            ix, iy = csp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(csp.calc_yaw(i_s))
            rk.append(csp.calc_curvature(i_s))

        return rx, ry, ryaw, rk, csp

    def calc_frenet_paths(self, c_speed, start_state): # input state

        frenet_paths = []

        s0 = start_state.s0
        c_d = start_state.c_d
        c_d_d = start_state.c_d_d
        c_d_dd = start_state.c_d_dd

        # generate path to each offset goal
        if ONLY_SAMPLE_TO_RIGHT:
            left_sample_bound = D_ROAD_W
        else:
            left_sample_bound = MAX_ROAD_WIDTH 
        for di in np.arange(-MAX_ROAD_WIDTH, left_sample_bound, D_ROAD_W):

            # Lateral motion planning
            for Ti in np.arange(MINT, MAXT, DT):
                fp = Frenet_path()

                lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti) 

                fp.t = np.arange(0.0, Ti, DT).tolist() # [t for t in np.arange(0.0, Ti, DT)]
                fp.d = [lat_qp.calc_point(t) for t in fp.t]                        
                fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
                fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
                fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

                # Loongitudinal motion planning (Velocity keeping)
                for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                    tfp = copy.deepcopy(fp)
                    lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                    tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                    tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                    tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                    tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                    Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                    Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                    # square of diff from target speed
                    ds = (TARGET_SPEED - tfp.s_d[-1])**2

                    tfp.cd = KJ * Jp + KT * Ti + KD * tfp.d[-1]**2
                    tfp.cv = KJ * Js + KT * Ti + KD * ds
                    tfp.cf = KLAT * tfp.cd + KLON * tfp.cv

                    frenet_paths.append(tfp)

        return frenet_paths

    def calc_global_paths(self, fplist, csp):

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
            dx = np.diff(np.array(fp.x))
            dy = np.diff(np.array(fp.y))
            fp.yaw = np.arctan2(dy,dx).tolist()
            fp.ds = np.sqrt(dx**2+dy**2).tolist()
            
            try:
                fp.yaw.append(fp.yaw[-1])
                fp.ds.append(fp.ds[-1])
            except:
                fp.yaw.append(0.1)
                fp.ds.append(0.1)

            # calc curvature
            # fp.c = (np.diff(fp.yaw) / np.array(fp.ds)).tolist()

            for i in range(len(fp.yaw) - 1):
                # carla simulation bug
                if fp.ds[i]<0.00001:
                    fp.ds[i] = 0.1
                fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

        return fplist

    def check_paths(self, fplist):
        okind = []
        for i, _ in enumerate(fplist):

            if any([v > MAX_SPEED for v in fplist[i][0].s_d]):  # Max speed check
                # rospy.logdebug("exceeding max speed")
                continue
            elif any([abs(a) > MAX_ACCEL for a in fplist[i][0].s_dd]):  # Max accel check
                # rospy.logdebug("exceeding max accel")
                continue
            elif any([abs(c) > MAX_CURVATURE for c in fplist[i][0].c]):  # Max curvature check
                # rospy.logdebug("exceeding max curvature")
                continue

            okind.append(i)

        return [fplist[i] for i in okind]


class quintic_polynomial:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, T): # s=start  e=end

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

        A = np.array([[T**3, T**4, T**5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T**2,
                      vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2 + 20 * self.a5 * t**3

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t**2

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
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class Frenet_path:

    def __init__(self):
        self.t = [] # time
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

class Frenet_state:

    def __init__(self):
        self.t = 0.0
        self.d = 0.0
        self.d_d = 0.0
        self.d_dd = 0.0
        self.d_ddd = 0.0
        self.s = 0.0
        self.s_d = 0.0
        self.s_dd = 0.0
        self.s_ddd = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.ds = 0.0
        self.c = 0.0

        