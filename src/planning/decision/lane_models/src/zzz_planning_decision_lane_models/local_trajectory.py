import rospy
import numpy as np
import math
from cvxopt import solvers, matrix
from zzz_common.geometry import dense_polyline2d, dist_from_point_to_polyline2d
from zzz_cognition_msgs.msg import MapState
from zzz_driver_msgs.utils import get_speed, get_yaw

from convert_frenet_to_world import calc_global_paths


from geometry_msgs.msg import AccelWithCovariance
from nav_msgs.msg import Odometry
from zzz_driver_msgs.msg import RigidBodyState, RigidBodyStateStamped, FrenetSerretState2D
from zzz_common.geometry import dist_from_point_to_polyline2d, wrap_angle

import tf.transformations as tft
import tf2_ros as tf2

import time
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

class MPCTrajectory(object):
    def __init__(self):
        #mpc paramters: Np is prediction horizon; N1 is pre-lane change horizon; 
        # N2 is lane change finish horizon
        # DT is discrete time step; TL is one-order delay of vehicle speed control
        # L is the wheelbase
        self.Np=20
        self.N1=4
        self.N2=16
        self.DT=0.2
        self.TL=0.45
        self.L=2.5

        #weight coefficients
        #differential state weighting  
        self.wx=0.0
        self.wy=100.0
        self.wtheta=100000.0
        self.wv=0.0
        #control input weighting
        self.wvd=100000000.0
        self.wdelta=1000000.0

        #control input constraints
        self.vmax=5.0
        self.vmin=-5.0
        self.dmax=0.02
        self.dmin=-0.02
        self._local_trajectory_generator_ref = PolylineTrajectory()

    def get_trajectory(self, dynamic_map, target_lane_index, desired_speed,resolution=0.5):
        rospy.loginfo("----\n using mpc trajectory")
        start = time.time()
        ego_lane_index_rounded = int(round(dynamic_map.mmap.ego_lane_index))
        ego_lane = dynamic_map.mmap.lanes[ego_lane_index_rounded].map_lane
        center_line=self.convert_path_to_ndarray(ego_lane.central_path_points)
        frenet_state=self.get_frenet(dynamic_map.ego_state, center_line)
        ego_x = frenet_state.s
        ego_y = frenet_state.d
        ego_vx = frenet_state.vs 
        ego_vy = frenet_state.vd
        ego_v = math.sqrt(ego_vx*ego_vx + ego_vy*ego_vy)
        ego_theta = math.atan2(ego_vy,ego_vx)

        print("ego state in frenet: ", ego_x, ego_y, ego_vx, ego_vy, ego_theta)        
        print("ego state in world : ", dynamic_map.ego_state.pose.pose.position.x, dynamic_map.ego_state.pose.pose.position.y, dynamic_map.ego_state.twist.twist.linear.x,dynamic_map.ego_state.twist.twist.linear.y)


        if target_lane_index == -1:
            target_lane = dynamic_map.jmap.reference_path
        else:
            target_lane = dynamic_map.mmap.lanes[int(target_lane_index)]
        # destination y position is the centre of the target lane 
        # FIXME(ksj)   
        y_des=(target_lane_index-ego_lane_index_rounded)*target_lane.map_lane.width
        rospy.logdebug("target lateral position %f, desired speed %f", y_des,desired_speed)
        # control input
        vdr = desired_speed
        delta_r = 0.0

        #reference model
        xr,yr,vr,theta_r = self.get_reference_model(ego_x, ego_y,ego_v,ego_theta,y_des,vdr,delta_r)
        
        #differential model
        s0=np.array([ego_x-xr[0],ego_y-yr[0],ego_theta-theta_r[0],ego_v-vr[0]]).reshape(4,1)

        # set MPC matrix
        Aex, Bex, Qex, Rex = self.get_mpc_matrix(xr,yr,vr,theta_r,delta_r)

        QB=np.dot(Qex,Bex)
        H=np.dot(Bex.T,QB)+Rex
        f=np.dot(np.dot(QB.T,Aex),s0)

        #get state constraints
        xmax,xmin,ymax,ymin = self.get_state_constraints(dynamic_map,target_lane_index,desired_speed,frenet_state)
        print("x max : ",xmax)
        print("x min : ",xmin)
        print("y max : ",ymax)
        print("y min : ",ymin)

        A,b,Cx,Cy = self.get_constraint_matrix(Aex,Bex,xmax,xmin,ymax,ymin,xr,yr,s0)

        # solve qp with cvxopt
        P=matrix(H)
        q=matrix(f)
        G=matrix(A)
        h=matrix(b)
        solve_s=time.time()
        sv=solvers.qp(P,q,G,h)
        solve_end=time.time()
        print("solve time is ", (solve_end-solve_s))
        print(sv['status'])

        if (sv['status']=="optimal"):
            rospy.loginfo("--------------------------------------------\n Optimal Found")
            uopt=sv['x']
            Xex=np.dot(Aex,s0)+np.dot(Bex,uopt)
            xp=np.dot(Cx,Xex)+xr
            yp=np.dot(Cy,Xex)+yr
            fplist = []  
            for i in np.arange(len(xp)):
                fp = Frenet_path()
                fp.s = xp[i]
                fp.d = yp[i]
                fplist.append(fp)

            ego_lane = dynamic_map.mmap.lanes[ego_lane_index_rounded].map_lane
            wx = [pos.position.x for pos in ego_lane.central_path_points]
            wy = [pos.position.y for pos in ego_lane.central_path_points]

            fplist = calc_global_paths(fplist,wx,wy)

            xw = [fp.x for fp in fplist if np.isnan(fp.x)==False]
            yw = [fp.y for fp in fplist if np.isnan(fp.y)==False]

            trajectory = np.hstack((xw,yw))

            print("trajectory=",trajectory)
        else:
            rospy.loginfo("=====================================\n Using reference")
            trajectory = self._local_trajectory_generator_ref.get_trajectory(dynamic_map, target_lane_index, desired_speed)
        end = time.time()
        print("run time is ", (end-start))
        return trajectory


    def convert_path_to_ndarray(self, path):
        point_list = [(point.position.x, point.position.y) for point in path]
        return np.array(point_list)

    def get_reference_model(self,x0,y0,v0,theta0,yd,vd,delta):
        xr = np.zeros(self.Np).reshape(self.Np,1)
        yr = np.zeros(self.Np).reshape(self.Np,1)
        vr = np.zeros(self.Np).reshape(self.Np,1)
        theta_r = np.zeros(self.Np).reshape(self.Np,1)

        xr[0] = x0
        yr[0] = yd
        vr[0] = v0
        theta_r[0] = 0.0
        
        for i in np.arange(1,self.Np):
            xr[i]=xr[i-1]+math.cos(theta_r[i-1])*vr[i-1]*self.DT
            yr[i]=yr[i-1]+math.sin(theta_r[i-1])*vr[i-1]*self.DT
            theta_r[i]=theta_r[i-1]+math.tan(delta)/self.L*vr[i-1]*self.DT
            vr[i]=vr[i-1]+(vd-vr[i-1])/self.TL*self.DT

        return xr,yr,vr,theta_r
    def get_mpc_matrix(self,xr,yr,vr,theta_r,delta_r):
        Aex=np.zeros([4*self.Np,4])
        Bex=np.zeros([4*self.Np,2*self.Np])
        Qex=np.zeros([4*self.Np,4*self.Np])
        Rex=np.zeros([2*self.Np,2*self.Np])

        Q=np.zeros([4,4])
        Q[0,0]=self.wx
        Q[1,1]=self.wy
        Q[2,2]=self.wtheta
        Q[3,3]=self.wv

        R=np.zeros([2,2])
        R[0,0]=self.wvd
        R[1,1]=self.wdelta

        for i in np.arange(self.Np):
            Ad=np.array([[1.0, 0.0, -vr[i]*math.sin(theta_r[i])*self.DT, math.cos(theta_r[i])],
                        [0.0, 1.0, vr[i]*math.cos(theta_r[i])*self.DT, math.sin(theta_r[i])],
                        [0.0, 0.0, 1.0, self.DT*math.tan(delta_r)/self.L],
                        [0.0, 0.0, 0.0, 1.0-self.DT/self.TL]])
            Bd=np.array([[0.0, 0.0],
                        [0.0, 0.0],
                        [0.0, vr[i]*self.DT/self.L/math.cos(delta_r)/math.cos(delta_r)],
                        [self.DT/self.TL, 0.0]])

            Bex[4*i:4*(i+1),2*i:2*(i+1)]=Bd
            Qex[4*i:4*(i+1),4*i:4*(i+1)]=Q
            Rex[2*i:2*(i+1),2*i:2*(i+1)]=R

            if (i==0):
                Aex[0:4,0:4]=Ad
                Bex[0:4,0:2]=Bd
            else:
                Aex[4*i:4*(i+1),0:4]=np.dot(Ad,Aex[4*(i-1):4*i])
                for j in np.arange(i):
                    Bex[4*i:4*(i+1),2*j:2*(j+1)]=np.dot(Ad,Bex[4*(i-1):4*i,2*j:2*(j+1)])
        return Aex, Bex, Qex, Rex

    def get_state_constraints(self,dynamic_map,target_lane_index,desired_speed,ego_ffstate):
        xmax=np.zeros(self.Np).reshape(self.Np,1)
        xmin=np.zeros(self.Np).reshape(self.Np,1)
        ymax=np.zeros(self.Np).reshape(self.Np,1)
        ymin=np.zeros(self.Np).reshape(self.Np,1)

        ego_lane_index_rounded = int(round(dynamic_map.mmap.ego_lane_index))

        ego_lane = dynamic_map.mmap.lanes[ego_lane_index_rounded].map_lane
        ego_lane_right_boundary = (-0.5)*ego_lane.width
        ego_lane_left_boundary = (0.5)*ego_lane.width

        target_lane = dynamic_map.mmap.lanes[int(target_lane_index)].map_lane
        target_lane_left_boundary =  (target_lane_index-ego_lane_index_rounded+0.5)*target_lane.width
        target_lane_right_boundary =  (target_lane_index-ego_lane_index_rounded-0.5)*target_lane.width

        # rospy.logdebug("lanes number = %d, ego lane index = %d, ego lane width = %f, target lane index = %d, target lane width = %f",
        #             len(dynamic_map.mmap.lanes),ego_lane_index_rounded,ego_lane.width, target_lane_index, target_lane.width)

        # rospy.logdebug("ego lane left boundary = %f, ego lane right boundary = %f, target lane left boundary = %f, target lane right boundary = %f",
        #             ego_lane_left_boundary,ego_lane_right_boundary,target_lane_left_boundary,target_lane_right_boundary)

        if len(dynamic_map.mmap.lanes[ego_lane_index_rounded].front_vehicles)>0:
            front_vehicle = dynamic_map.mmap.lanes[ego_lane_index_rounded].front_vehicles[0]
            front_vehicle_exist_flag = 1
        else:
            front_vehicle_exist_flag = 0
        
        if len(dynamic_map.mmap.lanes[ego_lane_index_rounded].rear_vehicles)>0:
            rear_vehicle = dynamic_map.mmap.lanes[ego_lane_index_rounded].rear_vehicles[0]
            rear_vehicle_exist_flag = 1
        else:
            rear_vehicle_exist_flag = 0
        
        if len(dynamic_map.mmap.lanes[target_lane_index].front_vehicles)>0:
            target_front_vehicle = dynamic_map.mmap.lanes[target_lane_index].front_vehicles[0]
            target_front_vehicle_exist_flag = 1
        else:
            target_front_vehicle_exist_flag = 0
        
        if len(dynamic_map.mmap.lanes[target_lane_index].rear_vehicles)>0:
            target_rear_vehicle = dynamic_map.mmap.lanes[target_lane_index].rear_vehicles[0]
            target_rear_vehicle_exist_flag = 1
        else:
            target_rear_vehicle_exist_flag = 0
        
        for i in np.arange(self.Np):
            if (i<self.N1):
                if front_vehicle_exist_flag==1:
                    front_gap = self.safeGap(desired_speed)
                    front_predict_state = self.predictCA(front_vehicle.ffstate,i*self.DT)
                    xmax[i]=front_predict_state.s - front_gap
                else:
                    xmax[i]=ego_ffstate.s+1000.0
                
                if rear_vehicle_exist_flag==1:  
                    rear_predict_state = self.predictCA(rear_vehicle.ffstate,i*self.DT)
                    rear_gap = self.safeGap(rear_predict_state.vs)
                    xmin[i]=rear_predict_state.s + rear_gap
                else:
                    xmin[i]=ego_ffstate.s-1000.0
    
                ymax[i]=ego_lane_left_boundary
                ymin[i]=ego_lane_right_boundary

            elif (i<self.N2):
                if front_vehicle_exist_flag==1 and target_front_vehicle_exist_flag==1:
                    front_gap = self.safeGap(desired_speed)
                    front_predict_state = self.predictCA(front_vehicle.ffstate, i*self.DT)
                    target_front_predict_state = self.predictCA(target_front_vehicle.ffstate,i*self.DT)
                    xmax[i]=min(front_predict_state.s, target_front_predict_state.s)-front_gap
                elif front_vehicle_exist_flag==1 and target_front_vehicle_exist_flag==0:
                    front_gap = self.safeGap(desired_speed)
                    front_predict_state = self.predictCA(front_vehicle.ffstate,i*self.DT)
                    xmax[i]=front_predict_state.s - front_gap
                elif front_vehicle_exist_flag==0 and target_front_vehicle_exist_flag==1:
                    front_gap = self.safeGap(desired_speed)
                    target_front_predict_state = self.predictCA(target_front_vehicle.ffstate,i*self.DT)
                    xmax[i]=target_front_predict_state.s - front_gap
                else:
                    xmax[i]=ego_ffstate.s+1000.0
                
                if rear_vehicle_exist_flag==1 and target_rear_vehicle_exist_flag==1:
                    rear_predict_state = self.predictCA(rear_vehicle.ffstate, i*self.DT)
                    target_rear_predict_state = self.predictCA(target_rear_vehicle.ffstate,i*self.DT)
                    rear_gap = self.safeGap(rear_predict_state.vs)
                    target_rear_gap = self.safeGap(target_rear_predict_state.vs)
                    xmin[i]=max(rear_predict_state.s+rear_gap, target_rear_predict_state.s+target_rear_gap)
                elif rear_vehicle_exist_flag==1 and target_rear_vehicle_exist_flag==0:
                    rear_predict_state = self.predictCA(rear_vehicle.ffstate,i*self.DT)
                    rear_gap = self.safeGap(rear_predict_state.vs)
                    xmin[i]=rear_predict_state.s + rear_gap
                elif rear_vehicle_exist_flag==0 and target_rear_vehicle_exist_flag==1:
                    target_rear_predict_state = self.predictCA(target_rear_vehicle.ffstate,i*self.DT)
                    target_rear_gap = self.safeGap(target_rear_predict_state.vs)
                    xmin[i]=target_rear_predict_state.s + target_rear_gap
                else:
                    xmin[i]=ego_ffstate.s-1000.0

                ymax[i]=max(ego_lane_left_boundary, target_lane_left_boundary)
                ymin[i]=min(ego_lane_right_boundary, target_lane_right_boundary)

            else:
                if target_front_vehicle_exist_flag==1:
                    front_gap = self.safeGap(desired_speed)
                    target_front_predict_state = self.predictCA(target_front_vehicle.ffstate,i*self.DT)
                    xmax[i]=target_front_predict_state.s - front_gap
                else:
                    xmax[i]=ego_ffstate.s+1000.0
                
                if target_rear_vehicle_exist_flag==1:  
                    target_rear_predict_state = self.predictCA(target_rear_vehicle.ffstate,i*self.DT)
                    target_rear_gap = self.safeGap(target_rear_predict_state.vs)
                    xmin[i]=target_rear_predict_state.s + target_rear_gap
                else:
                    xmin[i]=ego_ffstate.s-1000.0

                ymin[i]=target_lane_right_boundary
                ymax[i]=target_lane_left_boundary
                
        return xmax,xmin,ymax,ymin

    def predictCA(self,init_state,dt):
        pred_state = FrenetSerretState2D()
        pred_state.vs = init_state.vs + init_state.sa*dt
        pred_state.vd = init_state.vd + init_state.ad*dt
        pred_state.s = init_state.s + init_state.vs*dt+0.5*init_state.sa*dt*dt
        pred_state.d = init_state.d + init_state.vd*dt+0.5*init_state.ad*dt*dt
        return pred_state

    def safeGap(self,desired_speed,time_ahead=5, distance_ahead=10):
        return distance_ahead+desired_speed*time_ahead

    def get_constraint_matrix(self,Aex,Bex,xmax,xmin,ymax,ymin,xr,yr,s0):
        Cx=np.zeros([self.Np,4*self.Np])
        Cy=np.zeros([self.Np,4*self.Np])
        ub=np.zeros(2*self.Np).reshape(2*self.Np,1)
        lb=np.zeros(2*self.Np).reshape(2*self.Np,1)

        for i in np.arange(self.Np):
            Cx[i,4*i]=1.0
            Cy[i,4*i+1]=1.0
            ub[2*i]=self.vmax
            ub[2*i+1]=self.dmax
            lb[2*i]=self.vmin
            lb[2*i+1]=self.dmin
        
        MBy=np.dot(Cy,Bex)
        MBx=np.dot(Cx,Bex)
        bymax=ymax-yr-np.dot(np.dot(Cy,Aex),s0)
        bymin=-ymin+yr+np.dot(np.dot(Cy,Aex),s0)
        bxmax=xmax-xr-np.dot(np.dot(Cx,Aex),s0)
        bxmin=-xmin+xr+np.dot(np.dot(Cx,Aex),s0)
        #FIXME(ksj):ignore longitudinal constraints
        A=np.vstack((MBy,-MBy,MBx,-MBx,np.eye(2*self.Np),-np.eye(2*self.Np)))
        b=np.vstack((bymax,bymin,bxmax,bxmin,ub,-lb))
        # A=np.vstack((MBy,-MBy,np.eye(2*self.Np),-np.eye(2*self.Np)))
        # b=np.vstack((bymax,bymin,ub,-lb))
        return A, b, Cx, Cy
    
    def get_frenet(self,cartesian_state,polyline):

        dist, nearest_idx, nearest_type, dist_start, dist_end = dist_from_point_to_polyline2d(
            cartesian_state.pose.pose.position.x,
            cartesian_state.pose.pose.position.y,
            polyline, return_end_distance=True)

        if nearest_type == 1:
            psi = math.atan2(
                polyline[nearest_idx+1, 1] - polyline[nearest_idx, 1],
                polyline[nearest_idx+1, 0] - polyline[nearest_idx, 0])
        elif nearest_type == -1:
            psi = math.atan2(
                polyline[nearest_idx, 1] - polyline[nearest_idx-1, 1],
                polyline[nearest_idx, 0] - polyline[nearest_idx-1, 0],
            )
        else:
            psi = 0

        ori = cartesian_state.pose.pose.orientation
        _,_,yaw = tft.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        frenet = FrenetSerretState2D()
        frenet.s = dist_start
        frenet.d = dist
        frenet.psi = wrap_angle(yaw - psi)
        v = np.array([
            cartesian_state.twist.twist.linear.x,
            cartesian_state.twist.twist.linear.y])
        
        rot = np.array([
            [math.cos(frenet.psi), math.sin(frenet.psi)],
            [-math.sin(frenet.psi), math.cos(frenet.psi)]
        ])
            
        # frenet.vs, frenet.vd = v.dot(rot.T)
        frenet.vs, frenet.vd = ((v.T).dot(rot)).T
        frenet.omega = cartesian_state.twist.twist.angular.z        
        a = np.array([
            cartesian_state.accel.accel.linear.x,
            cartesian_state.accel.accel.linear.y])
        frenet.sa, frenet.ad = a.dot(rot.T)
        frenet.epsilon = cartesian_state.accel.accel.angular.z

        return frenet


        
class PolylineTrajectory(object):
    def get_trajectory(self, dynamic_map, target_lane_index, desired_speed,
                resolution=0.2, time_ahead=7, distance_ahead=15, rectify_thres=2,
                lc_dt = 1.5, lc_v = 2.67):
        # TODO: get smooth spline (write another module to generate spline)
        ego_x = dynamic_map.ego_state.pose.pose.position.x
        ego_y = dynamic_map.ego_state.pose.pose.position.y

        if target_lane_index == -1:
            target_lane = dynamic_map.jmap.reference_path
        else:
            target_lane = dynamic_map.mmap.lanes[int(target_lane_index)]

        central_path = self.convert_path_to_ndarray(target_lane.map_lane.central_path_points)

        # if ego vehicle is on the target path
        # works for lane change, lane follow and reference path follow
        dense_centrol_path = dense_polyline2d(central_path, resolution)
        nearest_dis, nearest_idx, _ = dist_from_point_to_polyline2d(ego_x, ego_y, dense_centrol_path)
        nearest_dis = abs(nearest_dis)

        if nearest_dis > rectify_thres:
            if dynamic_map.model == MapState.MODEL_MULTILANE_MAP and target_lane_index != -1:
                rectify_dt = abs(dynamic_map.mmap.ego_lane_index - target_lane_index)*lc_dt
            else:
                rectify_dt = nearest_dis/lc_v

            return self.generate_smoothen_lane_change_trajectory(dynamic_map, target_lane, rectify_dt, desired_speed)
        else:
            front_path = dense_centrol_path[nearest_idx:]
            dis_to_ego = np.cumsum(np.linalg.norm(np.diff(front_path, axis=0), axis = 1))

            trajectory = front_path[:np.searchsorted(dis_to_ego, desired_speed*time_ahead+distance_ahead)-1]
            trajectory02 = dense_polyline2d(trajectory, 0.2)

            # print('@@@@dist ', np.linalg.norm(trajectory02[0] - trajectory02[1]))
            return trajectory02

    # TODO(zyxin): Add these to zzz_navigation_msgs.utils
    def convert_path_to_ndarray(self, path):
        point_list = [(point.position.x, point.position.y) for point in path]
        return np.array(point_list)

    def generate_smoothen_lane_change_trajectory(self, dynamic_map, target_lane,
        rectify_dt, desired_speed, lc_dt = 1.5, rectify_min_d = 12, resolution=0.5, time_ahead=5, distance_ahead=10):

        target_lane_center_path = self.convert_path_to_ndarray(target_lane.map_lane.central_path_points)

        ego_x = dynamic_map.ego_state.pose.pose.position.x
        ego_y = dynamic_map.ego_state.pose.pose.position.y

        # Calculate the longitudinal distance for lane Change
        # Considering if the ego_vehicle is in a lane Change
        lc_dis = max(rectify_dt*desired_speed,rectify_min_d)

        dense_target_centrol_path = dense_polyline2d(target_lane_center_path, resolution)
        _, nearest_idx, _ = dist_from_point_to_polyline2d(ego_x, ego_y, dense_target_centrol_path)
        front_path = dense_target_centrol_path[nearest_idx:]
        dis_to_ego = np.cumsum(np.linalg.norm(np.diff(front_path, axis=0), axis = 1))

        start_point = np.array([ego_x,ego_y])
        end_point = front_path[np.searchsorted(dis_to_ego, lc_dis)]

        # calculate start direction and end direction for control

        ego_direction = get_yaw(dynamic_map.ego_state)
        _, nearest_end_idx, _ = dist_from_point_to_polyline2d(end_point[0], end_point[1], target_lane_center_path)
        end_point_direction = target_lane.map_lane.central_path_points[nearest_end_idx].tangent

        start_tangent = np.array([np.cos(ego_direction),np.sin(ego_direction)])
        end_tangent = np.array([np.cos(end_point_direction),np.sin(end_point_direction)])

        lc_path = self.cubic_hermite_spline(p0 = start_point, p1 = end_point,
                                            m0 = start_tangent, m1 = end_tangent)

        # get ahead Path
        ahead_dis = desired_speed*time_ahead+distance_ahead
        path_after_lc = front_path[np.searchsorted(dis_to_ego, lc_dis):np.searchsorted(dis_to_ego, ahead_dis)-1]

        # replace lane change path into ahead path
        smoothen_lc_path = np.concatenate((lc_path,path_after_lc),axis = 0)

        return smoothen_lc_path

    def cubic_hermite_spline(self, p0, p1, m0, m1, resolution = 20):
        """
        Generate cubic hermit spline
        p0: start point np.array(2)
        p1: end point np.array(2)
        m0: start tangent np.array(2)
        m1: end tangent np.array(2)
        return path from start point to end point
        """

        t = np.linspace(0,1,num = resolution)
        h00 = (2*t*t*t-3*t*t+1).reshape(len(t),1) #p0
        h10 = (t*t*t-2*t*t+t).reshape(len(t),1) #m0
        h01 = (-2*t*t*t+3*t*t).reshape(len(t),1) #p1
        h11 = (t*t*t-t*t).reshape(len(t),1) #m1

        p0 = p0.reshape(1,2)
        p1 = p1.reshape(1,2)
        m0 = m0.reshape(1,2)
        m1 = m1.reshape(1,2)

        return np.matmul(h00,p0) + np.matmul(h10,m0) + np.matmul(h01,p1) + np.matmul(h11,m1)