
import numpy as np
import rospy
import matplotlib.pyplot as plt
import copy
import math

from cubic_spline_planner import Spline2D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from zzz_navigation_msgs.msg import Lane
from zzz_driver_msgs.utils import get_speed
from zzz_cognition_msgs.msg import RoadObstacle
from zzz_common.kinematics import get_frenet_state
from zzz_common.geometry import dense_polyline2d
from zzz_planning_msgs.msg import DecisionTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from zzz_planning_decision_continuous_models.common import rviz_display, convert_ndarray_to_pathmsg, convert_path_to_ndarray
from zzz_planning_decision_continuous_models.predict import predict


SIM_LOOP = 500

# Parameter
MAX_SPEED = 50.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 10.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 500.0  # maximum curvature [1/m]
MAX_ROAD_WIDTH = 6.0   # maximum road width [m] # related to RL action space
D_ROAD_W = 1.5  # road width sampling length [m]
DT = 0.3  # time tick [s]
MAXT = 4.6  # max prediction time [m]
MINT = 4.0  # min prediction time [m]
TARGET_SPEED = 15.0 / 3.6  # target speed [m/s]
D_T_S = 5.0 / 3.6  # target speed sampling length [m/s]
N_S_SAMPLE = 2  # sampling number of target speed

# collision check
OBSTACLES_CONSIDERED = 5
ROBOT_RADIUS = 3.5  # robot radius [m]
RADIUS_SPEED_RATIO = 0.25 # higher speed, bigger circle
MOVE_GAP = 1.0
ONLY_SAMPLE_TO_RIGHT = True

# Cost weights
KJ = 0.1
KT = 0.1
KD = 1.0
KLAT = 1.0
KLON = 1.0


class Werling(object):

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

        self.s0 = None
        self.c_d = None
        self.c_d_d = None
        self.c_d_dd = None
        self.c_speed = None

        self.last_trajectory_array = []
        self.last_trajectory = []
        self.last_trajectory_array_rule = []
        self.last_trajectory_rule = []
        self.reference_path = None

        self.rviz_all_trajectory = None

        self._dynamic_map = None
        self.ref_path = None
        self.ref_path_tangets = None
        self.ob = None
        self.csp = None

        self.rivz_element = rviz_display()
    
    def clear_buff(self, dynamic_map):
        self.last_trajectory_array = []
        self.last_trajectory = []
        self.last_trajectory_array_rule = []
        self.last_trajectory_rule = []
        self.reference_path = None
        self.csp = None

        self.rivz_element.candidates_trajectory = None
        self.rivz_element.prediciton_trajectory = None
        self.rivz_element.collision_circle = None
        return None
    
    def trajectory_update(self, dynamic_map):
        if self.initialize(dynamic_map):
            trajectory_rule, desired_speed_rule = self.calculate_trajectory(dynamic_map)
            msg = DecisionTrajectory()
            msg.trajectory = convert_ndarray_to_pathmsg(trajectory_rule)
            msg.desired_speed = desired_speed_rule

            self.rivz_element.candidates_trajectory = self.rivz_element.put_trajectory_into_marker(self.all_trajectory)
            self.rivz_element.prediciton_trajectory = self.rivz_element.put_trajectory_into_marker(self.obs_prediction.obs_paths)
            self.rivz_element.collision_circle = self.obs_prediction.rviz_collision_checking_circle
            return msg
        else:
            return None

    def initialize(self, dynamic_map):
        self._dynamic_map = dynamic_map
        try:
            # estabilish frenet frame
            if self.csp is None:
                self.reference_path = dynamic_map.jmap.reference_path.map_lane.central_path_points
                ref_path_ori = convert_path_to_ndarray(self.reference_path)
                self.ref_path = dense_polyline2d(ref_path_ori, 2)
                self.ref_path_tangets = np.zeros(len(self.ref_path))

                Frenetrefx = self.ref_path[:,0]
                Frenetrefy = self.ref_path[:,1]
                tx, ty, tyaw, tc, self.csp = self.generate_target_course(Frenetrefx,Frenetrefy)
            # initialize prediction module
            self.obs_prediction = predict(dynamic_map, OBSTACLES_CONSIDERED, MAXT, DT, ROBOT_RADIUS, RADIUS_SPEED_RATIO, MOVE_GAP,
                                        get_speed(dynamic_map.ego_state))
            return True

        except:
            print("initialize fail")
            return False

    def calculate_start_state(self, dynamic_map):
        if len(self.last_trajectory_array_rule) > 5:
            # find closest point on the last trajectory
            mindist = float("inf")
            bestpoint = 0
            for t in range(len(self.last_trajectory_rule.t)):
                pointdist = (self.last_trajectory_rule.x[t] - dynamic_map.ego_state.pose.pose.position.x) ** 2 + (self.last_trajectory_rule.y[t] - dynamic_map.ego_state.pose.pose.position.y) ** 2
                if mindist >= pointdist:
                    mindist = pointdist
                    bestpoint = t
            self.s0 = self.last_trajectory_rule.s[bestpoint]
            self.c_d = self.last_trajectory_rule.d[bestpoint]
            self.c_d_d = self.last_trajectory_rule.d_d[bestpoint]
            self.c_d_dd = self.last_trajectory_rule.d_dd[bestpoint]
            self.c_speed = self.last_trajectory_rule.s_d[bestpoint]
        else:
            ego_state = dynamic_map.ego_state
            self.c_speed = get_speed(ego_state)       # current speed [m/s]
            ffstate = get_frenet_state(dynamic_map.ego_state, self.ref_path, self.ref_path_tangets)

            self.c_d = -ffstate.d #- dynamic_map.ego_ffstate.d #ffstate.d  # current lateral position [m]
            self.c_d_d = ffstate.vd #dynamic_map.ego_ffstate.vd #ffstate.vd  # current lateral speed [m/s]
            self.c_d_dd = 0   # current latral acceleration [m/s]
            self.s0 = ffstate.s #+ c_speed * 0.5      # current course position
 
    def calculate_trajectory(self, dynamic_map):
        self.calculate_start_state(dynamic_map)

        generated_trajectory = self.frenet_optimal_planning(self.csp, self.s0, self.c_speed, self.c_d, self.c_d_d, self.c_d_dd, self.ob)

        if generated_trajectory is not None:
            desired_speed = generated_trajectory.s_d[-1]
            trajectory_array_ori = np.c_[generated_trajectory.x, generated_trajectory.y]
            trajectory_array = dense_polyline2d(trajectory_array_ori,1)
            # trajectory_array = trajectory_array_ori
            self.last_trajectory_array_rule = trajectory_array
            self.last_trajectory_rule = generated_trajectory              
            print("----> Werling: Successful Planning")
        
        elif len(self.last_trajectory_array_rule) > 5 and self.c_speed > 1:
            trajectory_array = self.last_trajectory_array_rule
            generated_trajectory = self.last_trajectory_rule
            desired_speed =  0 #generated_trajectory.s_d[-1] 
            print("----> Werling: Fail to find a solution")

        else:
            trajectory_array =  self.ref_path
            desired_speed = 0
            print("----> Werling: Output ref path")

        return trajectory_array, desired_speed    

    def frenet_optimal_planning(self, csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob):
        now00 = rospy.get_rostime()

        fplist = self.calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
        now0 = rospy.get_rostime()
        print("-----------------------------frenet time consume inside111",now0.to_sec() - now00.to_sec())
        print("-----------------------------fplist",len(fplist))
        fplist = self.calc_global_paths(fplist, csp)
        now1 = rospy.get_rostime()
        print("-----------------------------frenet time consume inside222",now1.to_sec() - now0.to_sec())
        print("-----------------------------fplist",len(fplist))

        fplist = self.check_paths(fplist, ob)
        now2 = rospy.get_rostime()
        print("-----------------------------frenet time consume inside333",now2.to_sec() - now1.to_sec())
        print("-----------------------------fplist",len(fplist))
        self.all_trajectory = fplist

        # find minimum cost path
        mincost = float("inf")
        bestpath = None
        for fp in fplist:
            if mincost >= fp.cf:
                mincost = fp.cf
                if self.obs_prediction.check_collision(fp):
                    bestpath = fp
        return bestpath

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

    def calc_frenet_paths(self, c_speed, c_d, c_d_d, c_d_dd, s0): # input state

        frenet_paths = []

        # generate path to each offset goal
        if ONLY_SAMPLE_TO_RIGHT:
            left_sample_bound = D_ROAD_W
        else:
            left_sample_bound = MAX_ROAD_WIDTH 
        for di in np.arange(-MAX_ROAD_WIDTH, left_sample_bound, D_ROAD_W):# sampling -7-7   1

            # Lateral motion planning
            for Ti in np.arange(MINT, MAXT, DT):#4 5  0.2
                fp = Frenet_path()

                lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti) 

                fp.t = [t for t in np.arange(0.0, Ti, DT)]
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
            for i in range(len(fp.x) - 1):
                dx = fp.x[i + 1] - fp.x[i]
                dy = fp.y[i + 1] - fp.y[i]
                fp.yaw.append(math.atan2(dy, dx))
                fp.ds.append(math.sqrt(dx**2 + dy**2))

            
            try:
                fp.yaw.append(fp.yaw[-1])
                fp.ds.append(fp.ds[-1])
            except:
                fp.yaw.append(0.1)
                fp.ds.append(0.1)

            # calc curvature
            for i in range(len(fp.yaw) - 1):
                fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

        return fplist

    def check_paths(self, fplist, ob):
        okind = []
        for i, _ in enumerate(fplist):

            if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
                print("exceeding max speed")
                continue
            elif any([abs(a) > MAX_ACCEL for a in fplist[i].s_dd]):  # Max accel check
                print("exceeding max accel")
                continue
            elif any([abs(c) > MAX_CURVATURE for c in fplist[i].c]):  # Max curvature check
                print("exceeding max curvature")
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

        


#bak code


    #predition
    # def found_closest_obstacles(self, num, dynamic_map):
    #     closest_obs = []
    #     obs_tuples = []
        
    #     for obs in self._dynamic_map.jmap.obstacles: 
    #         # calculate distance
    #         p1 = np.array([self._dynamic_map.ego_state.pose.pose.position.x , self._dynamic_map.ego_state.pose.pose.position.y])
    #         p2 = np.array([obs.state.pose.pose.position.x , obs.state.pose.pose.position.y])
    #         p3 = p2 - p1
    #         p4 = math.hypot(p3[0],p3[1])


    #         obs_ffstate = get_frenet_state(obs.state, self.ref_path, self.ref_path_tangets)
            
    #         one_obs = (obs.state.pose.pose.position.x , obs.state.pose.pose.position.y , obs.state.twist.twist.linear.x , obs.state.twist.twist.linear.y , p4 , obs.state.accel.accel.linear.x, obs.state.accel.accel.linear.y)
    #         # if obs.ffstate.s > 0.01:
    #         obs_tuples.append(one_obs)
        
    #     sorted_obs = sorted(obs_tuples, key=lambda obs: obs[4])   # sort by distance
    #     i = 0
    #     for obs in sorted_obs:
    #         if i < num:
    #             closest_obs.append(obs)
    #             i = i + 1
    #         else:
    #             break
        
    #     return closest_obs

    # def prediction_obstacle(self, ob, prediction_model): # we should do prediciton in driving space
        
    #     obs_paths = []

    #     if prediction_model == 1:
    #         for one_ob in ob:
    #             obsp = Frenet_path()
    #             obsp.t = [t for t in np.arange(0.0, MAXT, DT)]
    #             ax = 0#one_ob[5]
    #             ay = 0#one_ob[6]

    #             for i in range(len(obsp.t)):
    #                 vx = one_ob[2] + ax * DT * i
    #                 vy = one_ob[3] + ax * DT * i
    #                 obspx = one_ob[0] + i * DT * vx
    #                 obspy = one_ob[1] + i * DT * vy
    #                 obsp.x.append(obspx)
    #                 obsp.y.append(obspy)
                    
    #             obs_paths.append(obsp)

    #     return obs_paths

    # def check_collision(self, fp, ob):

    #     CHECK_RADIUS = ROBOT_RADIUS + RADIUS_SPEED_RATIO * self.c_speed
        

    #     # two circles for a vehicle
    #     fp_front = fp
    #     fp_back = fp

       
    #     self.obs_paths = self.prediction_obstacle(ob,1)
    #     if len(self.obs_paths) == 0:
    #         return True
    #     try:
    #         for t in range(len(fp.t)):
    #             fp_front.x[t] = fp.x[t] + math.cos(fp.yaw[t]) * 2 * MOVE_GAP
    #             fp_front.y[t] = fp.y[t] + math.sin(fp.yaw[t]) * 2 * MOVE_GAP
    #             fp_back.x[t] = fp.x[t] - math.cos(fp.yaw[t]) * MOVE_GAP
    #             fp_back.y[t] = fp.y[t] - math.sin(fp.yaw[t]) * MOVE_GAP


    #         for obsp in self.obs_paths:
    #             for t in range(len(fp.t)):
    #                 d = (obsp.x[t] - fp_front.x[t])**2 + (obsp.y[t] - fp_front.y[t])**2
    #                 if d <= CHECK_RADIUS**2: 
    #                     return False
    #                 d = (obsp.x[t] - fp_back.x[t])**2 + (obsp.y[t] - fp_back.y[t])**2
    #                 if d <= CHECK_RADIUS**2: 
    #                     return False
    #     except:
    #         pass

    #     return True

    #common
    # def convert_ndarray_to_pathmsg(self, path):
    #         msg = Path()
    #         for wp in path:
    #             pose = PoseStamped()
    #             pose.pose.position.x = wp[0]
    #             pose.pose.position.y = wp[1]
    #             msg.poses.append(pose)
    #         msg.header.frame_id = "map" 

    #         return msg


    # def convert_path_to_ndarray(self, path):
    #     point_list = [(point.position.x, point.position.y) for point in path]
    #     return np.array(point_list)
    
    