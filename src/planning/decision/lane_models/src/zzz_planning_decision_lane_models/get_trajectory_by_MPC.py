import rospy
import numpy as np
import math
from cvxopt import solvers, matrix
from zzz_common.geometry import dense_polyline2d, dist_from_point_to_polyline2d
from zzz_cognition_msgs.msg import MapState
from zzz_driver_msgs.utils import get_speed, get_yaw

class MPCTrajectory(object):
    def __init__(self):
        #mpc paramters: Np is prediction horizon; N1 is pre-lane change horizon; 
        # N2 is lane change finish horizon
        # DT is discrete time step; TL is one-order delay of vehicle speed control
        # L is the wheelbase
        self.Np=20
        self.N1=5
        self.N2=15
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

    def get_trajectory(self, dynamic_map, target_lane_index, desired_speed,resolution=0.5):
        # TODO: get smooth spline (write another module to generate spline)
        ego_x = dynamic_map.ego_state.pose.pose.position.x
        ego_y = dynamic_map.ego_state.pose.pose.position.y
        ego_vx = dynamic_map.ego_state.twist.twist.linear.x 
        ego_vy = dynamic_map.ego_state.twist.twist.linear.y
        ego_v = math.sqrt(ego_vx*ego_vx + ego_vy*ego_vy)
        ego_theta = math.atan2(ego_vy,ego_vx)

        if target_lane_index == -1:
            target_lane = dynamic_map.jmap.reference_path
        else:
            target_lane = dynamic_map.mmap.lanes[int(target_lane_index)]
        # destination y position is the centre of the target lane    
        y_des=target_lane.map_lane.central_path_points[0].position.y
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
        H=np.dot(Bex.transpose(),QB)+Rex
        f=np.dot(np.dot(QB.transpose(),Aex),s0)

        #get state constraints
        xmax,xmin,ymax,ymin = self.get_state_constraints(dynamic_map,target_lane_index,desired_speed)

        A,b = self.get_constraint_matrix(Aex,Bex,xmax,xmin,ymax,ymin,xr,yr,s0)

        # solve qp with cvxopt
        P=matrix(H)
        q=matrix(f)
        G=matrix(A)
        h=matrix(b)

        sv=solvers.qp(P,q,G,h)

        uopt=sv['x']
        Xex=np.dot(Aex,s0)+np.dot(Bex,uopt)
        xp=np.dot(Cx,Xex)+xr
        yp=np.dot(Cy,Xex)+yr

        trajectory = np.hstack((xp,yp))

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
                        [0.0, vr[i]*self.DT/self.L*math.cos(delta_r)/math.cos(delta_r)],
                        [self.DT/self.TL, 0.0]])

            Bex[4*i:4*(i+1),2*i:2*(i+1)]=Bd
            Qex[4*i:4*(i+1),4*i:4*(i+1)]=Q
            Rex[2*i:2*(i+1),2*i:2*(i+1)]=R

            if (i==0):
                Aex[0:4,0:4]=Ad
                Bex[0:4,0:2]=Bd
            else:
                Aex[4*i:4*(i+1),0:4]=np.dot(Ad,Aex[4*(i-1):4*i])
                for j in np.arange(i-1):
                    Bex[4*i:4*(i+1),2*j:2*(j+1)]=np.dot(Ad,Bex[4*(i-1):4*i,2*j:2*(j+1)])
        return Aex, Bex, Qex, Rex

    def get_state_constraints(self,dynamic_map,target_lane_index,desired_speed):
        xmax=np.zeros(self.Np).reshape(self.Np,1)
        xmin=np.zeros(self.Np).reshape(self.Np,1)
        ymax=np.zeros(self.Np).reshape(self.Np,1)
        ymin=np.zeros(self.Np).reshape(self.Np,1)

        ego_lane_index_rounded = int(round(dynamic_map.mmap.ego_lane_index))
        rospy.logdebug("ego lane index = %d, lanes number = %d, target lane index = %d", 
                    ego_lane_index_rounded,len(dynamic_map.mmap.lanes),target_lane_index)
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
            ego_lane_central_y = dynamic_map.mmap.lanes[ego_lane_index_rounded].map_lane.central_path_points[0].position.y
            ego_lane_width = dynamic_map.mmap.lanes[ego_lane_index_rounded].map_lane.width
            ego_lane_left_boundary = ego_lane_central_y - ego_lane_width/2.0
            ego_lane_right_boundary = ego_lane_central_y + ego_lane_width/2.0

            target_lane_central_y = dynamic_map.mmap.lanes[int(target_lane_index)].map_lane.central_path_points[0].position.y
            target_lane_width = dynamic_map.mmap.lanes[int(target_lane_index)].map_lane.width
            target_lane_left_boundary = target_lane_central_y - target_lane_width/2.0
            target_lane_right_boundary = target_lane_central_y + target_lane_width/2.0

            if (i<self.N1):
                if front_vehicle_exist_flag==1:
                    front_gap = safeGap(desired_speed)
                    front_predict_state = predictCA(front_vehicle.state,i*self.DT)
                    xmax[i]=front_predict_state.pose.pose.position.x - front_gap
                else:
                    xmax[i]=1000.0
                
                if rear_vehicle_exist_flag==1:  
                    rear_predict_state = predictCA(rear_vehicle.state,i*self.DT)
                    rear_gap = safeGap(rear_predict_state.twist.twist.linear.x)
                    xmin[i]=rear_predict_state.pose.pose.position.x + rear_gap
                else:
                    xmin[i]=-1000.0
    
                ymin[i]=ego_lane_left_boundary
                ymax[i]=ego_lane_right_boundary

            elif (i<self.N2):
                if front_vehicle_exist_flag==1 and target_front_vehicle_exist_flag==1:
                    front_gap = safeGap(desired_speed)
                    front_predict_state = predictCA(front_vehicle.state, i*self.DT)
                    target_front_predict_state = predictCA(target_front_vehicle.state,i*self.DT)
                    xmax[i]=min(front_predict_state.pose.pose.position.x, target_front_predict_state.pose.pose.position.x)-front_gap
                elif front_vehicle_exist_flag==1 and target_front_vehicle_exist_flag==0:
                    front_gap = safeGap(desired_speed)
                    front_predict_state = predictCA(front_vehicle.state,i*self.DT)
                    xmax[i]=front_predict_state.pose.pose.position.x - front_gap
                elif front_vehicle_exist_flag==0 and target_front_vehicle_exist_flag==1:
                    front_gap = safeGap(desired_speed)
                    target_front_predict_state = predictCA(target_front_vehicle.state,i*self.DT)
                    xmax[i]=target_front_predict_state.pose.pose.position.x - front_gap
                else:
                    xmax[i]=1000.0
                
                if rear_vehicle_exist_flag==1 and target_rear_vehicle_exist_flag==1:
                    rear_predict_state = predictCA(rear_vehicle.state, i*self.DT)
                    target_rear_predict_state = predictCA(target_rear_vehicle.state,i*self.DT)
                    rear_gap = safeGap(rear_predict_state.twist.twist.linear.x)
                    target_rear_gap = safeGap(target_rear_predict_state.twist.twist.linear.x)
                    xmin[i]=max(rear_predict_state.pose.pose.position.x+rear_gap, target_rear_predict_state.pose.pose.position.x+target_rear_gap)
                elif rear_vehicle_exist_flag==1 and target_rear_vehicle_exist_flag==0:
                    rear_predict_state = predictCA(rear_vehicle.state,i*self.DT)
                    rear_gap = safeGap(rear_predict_state.twist.twist.linear.x)
                    xmin[i]=rear_predict_state.pose.pose.position.x + rear_gap
                elif rear_vehicle_exist_flag==0 and target_rear_vehicle_exist_flag==1:
                    target_rear_predict_state = predictCA(target_rear_vehicle.state,i*self.DT)
                    target_rear_gap = safeGap(target_rear_predict_state.twist.twist.linear.x)
                    xmin[i]=target_rear_predict_state.pose.pose.position.x + target_rear_gap
                else:
                    xmin[i]=-1000.0

                ymin[i]=min(ego_lane_left_boundary, target_lane_left_boundary)
                ymax[i]=max(ego_lane_right_boundary, target_lane_right_boundary)

            else:
                if target_front_vehicle_exist_flag==1:
                    front_gap = safeGap(desired_speed)
                    target_front_predict_state = predictCA(target_front_vehicle.state,i*self.DT)
                    xmax[i]=target_front_predict_state.pose.pose.position.x - front_gap
                else:
                    xmax[i]=1000.0
                
                if target_rear_vehicle_exist_flag==1:  
                    target_rear_predict_state = predictCA(target_rear_vehicle.state,i*self.DT)
                    target_rear_gap = safeGap(target_rear_predict_state.twist.twist.linear.x)
                    xmin[i]=target_rear_predict_state.pose.pose.position.x + target_rear_gap
                else:
                    xmin[i]=-1000.0

                ymax[i]=target_lane_right_boundary
                ymin[i]=target_lane_left_boundary
                
        return xmax,xmin,ymax,ymin

    def predictCA(init_state,dt):
        pre_state = init_state
        pre_state.twist.twist.linear.x = init_state.twist.twist.linear.x + init_state.accel.accel.linear.x*dt
        pre_state.twist.twist.linear.y = init_state.twist.twist.linear.y + init_state.accel.accel.linear.y*dt
        pre_state.pose.pose.position.x = init_state.pose.pose.position.x + init_state.twist.twist.linear.x*dt+0.5*init_state.accel.accel.linear.x*dt*dt
        pre_state.pose.pose.position.y = init_state.pose.pose.position.y + init_state.twist.twist.linear.y*dt+0.5*init_state.accel.accel.linear.y*dt*dt
        return pred_state

    def safeGap(desired_speed,time_ahead=5, distance_ahead=10):
        return distance_ahead+desired_speed*time_ahead

    def get_constraint_matrix(self,Aex,Bex,xmax,xmin,ymax,ymin,xr,yr,s0):
        Cx=np.zeros([self.Np,4*self.Np])
        Cy=np.zeros([self.Np,4*self.Np])
        ub=np.zeros(2*self.Np).reshape(self.Np,1)
        lb=np.zeros(2*self.Np).reshape(self.Np,1)

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
        print("array size are: ",bymax.shape,"--",bymin.shape,"--",bxmax.shape,"--",bxmin.shape,"--",ub.shape,"--",lb.shape)
        A=np.vstack((MBy,-MBy,MBx,-MBx,np.eye(2*self.Np),-np.eye(2*self.Np)))
        b=np.vstack((bymax,bymin,bxmax,bxmin,ub,-lb))
        return A, b


        
    