
#ifndef ICV_CARLA_CONTROL_H
#define ICV_CARLA_CONTROL_H


#include <stdio.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
// #include "waypoint_follower/libwaypoint_follower.h"
#include <thread>
#include <tf/tf.h>
#include <math.h>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/QR>
// inlcude iostream and string libraries
#include <string>
#include <time.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/QR>
//carla shiyong
// #include"CarlaEgoVehicleStatus.h"  //实车屏蔽
// #include"CarlaEgoVehicleControl.h" //实车屏蔽
#include <zzz_driver_msgs/RigidBodyState.h>
#include <zzz_driver_msgs/RigidBodyStateStamped.h>

#include"nav_msgs/Path.h" 
#include <yaml-cpp/yaml.h>

#include <geographic_msgs/GeoPointStamped.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "xpmotors_can_msgs/AutoCtlReq.h"
#include "xpmotors_can_msgs/AutoStateEx.h"
#include "xpmotors_can_msgs/AutoState.h"
#include "xpmotors_can_msgs/ESCStatus.h"
#include "xpmotors_can_msgs/EPSStatus.h"
#include "zzz_planning_msgs/DecisionTrajectory.h"

#include "icv_carla_rviz.h"

using namespace Eigen;
using namespace std;

namespace icv
{
    class icvCarlaControl
    {
    private:
        /* data */
        float X,Y,theta;
        // int len_waypoint=300;//路径为固定300个点
        float v=0;//车辆速度m/s
        //vehicle mode param
        float lf=1.5;//
        float lr=1.5;//
        float L=lr+lf;
        float m=1818.2;//mass of Vehicle
        float Ccf=66900;//前轮侧偏刚度
        float Ccr=62700;//后轮侧偏刚度
        float Clf=66900;//前轮纵向刚度
        float Clr=62700;//后轮纵向刚度
        float delta_f=0;//前轮偏角度
        float Sf=0.2;//前轮纵向滑移率
        float Sr=0.2;//后轮纵向滑移率
        float I=4175;//车辆质心转动惯量
        float wheel_base=2.7;//车辆轴距
        float g=9.8;
        float R_min=5.13;//最小转向半径
        float K=15.7;//方向盘传动比
        /*状态约束程序*/
        float r=0;//横摆角速度
        float x_dot=0.000001,y_dot,phi_dot,phi;
        float dt=0.05;
        float Acc_max=3;//加速度上限？按标准来
        float jerk_max=5;//jerk_max加速度变化率上限
     
        //yan
        int _flag=0;
        double _wheel_angle_degree_last=0;
        double _cal_wheel_angle=0;
        int _record_cnt=0;
        float _vehicle_wheel_angle;//车辆实时方向盘角度
        float _keep_wheel_angle=0;//记录上一时刻方向盘角度值
        float _delta_T=0.05;
        vector<double> _path_x;
        vector<double>  _path_y;       
        float _p1_neg = -0.001498; //轮胎转角为负的情况
        float _p2_neg = -0.02241;
        float _p3_neg = 14.99;
        float _p1_post = -0.0008035; //轮胎转角为正的情况
        float _p2_post = -0.01031;
        float _p3_post = 15.33;
        float __distance_pre=0.7;
        float _dis_pre_max=20;//最大预瞄距离
        float _dis_pre_min=2;//最小预瞄距离
        float _wheel_angle_degree_max = 470; //方向盘最大的转角
        float _wheel_angle_degree_sec = 200; //一秒中方向盘最大的转角
        float _t_preview = 2.8; //预瞄时间s
        int nextpoint_num=0;//最近索引点
        
        double PI=3.141592654;        
        int Waypoints_size=0;
        /**通信flag*/
        bool current_pose_flag=false;
        bool waypoints_flag=false;
        bool setVehiclestauts_flag=false;

        /****数据采集param****/        
        float ctedata[3];
        double sample_time=0;//单周期运行时间
        /**数据传递参数*/       
        float target_speed=0;//目标车速输出
        /**PID参数定义****/
        double error_p=0;		//kp误差
        double error_i = 0;		//ki误差
        double error_d = 0;		//kd误差
        double error_dp = 0;	//上一次的kp误差、用于计算kd误差
        double out_insturct=0;  //输出指令
        /***/
        double lookahead_distance_=0;       
        double Beta=0;     

        /**/
        float _x_preview_rviz;        //预瞄处的x坐标
        float _y_preview_rviz;        //预瞄处的y坐标   
        /*结构体定义*/
        typedef struct Path
        {
            float x=0;
            float y=0;
            float z=0;
            float theta=0;//度。
            float pitch=0;
            float roll=0;
            float v=0;//车辆速度m/s
            float x_dot=0;//车辆纵向速度
            float y_dot=0;//车辆横向速度
            float phi_dot=0;//车辆横向偏转角速度
            float steer_rec=0;
        }Pathpoint;
        typedef struct PID
        {
            float Kp=0;
            float Ki=0;
            float Kd=0;
        }pid_; 
        PID _v_pid;  //速度的PID系数
        Pathpoint Pose_start;//Vehicle 初始点
        Pathpoint Waypoints[10240];//的矩阵，用于存放路径点，车速  定义结构体数组        
	    vector <Pathpoint> Waypointss; //创建点的数组
	    Pathpoint Waypoint; //实例化路径

        Pathpoint Current_Point;//车辆实时坐标点
        geometry_msgs::Point next_target_position_;//当前车辆坐标下一个点
        geometry_msgs::Pose current_pose_;//车辆当前姿态
        // carla_msgs::CarlaEgoVehicleStatus vehicle_status;//车辆当前底盘状态
        nav_msgs::Path Path;//路径点信息

        void controlInit();        
        double PIDcontrol(double error,PID pid );              
              
        // Pathpoint Vehiclemodeupdate1(Pathpoint coordinate,float delta_f,float T);
        // Pathpoint Vehiclemodeupdate2(Pathpoint coordinate,float delta_f,float T);  
       
        inline double deg2rad(double deg);
        /**openICV***/
        double  ControlStr(double _vehicle_heading_theta,double _vehicle_v, double _vehicle_x,double _vehicle_y);
        float  find_min_point(float _preview_x_rec,float _preview_y_rec,int *count_min);   
        float  cal_transfer_val(float _d_dis_rec);
        float limit_wheel_val(float _wheel_last_ref_rec, float _wheel_angle_degree_rec);    
        //多项式拟合
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
        double polyeval(Eigen::VectorXd coeffs, double x);
        
        void pathdeal();//计算路径偏差
        bool StateConstraint(float v,float angle,float T);
        float tiremodel(float beita,float cetia,float v,float r);
        bool PathSmoothCheck();//求解path各曲率处,有问题返回bool判断标志
        double Velocitytopredict(float velocity,float distance_pre,float velocity_pre);//小鹏速度模块，输入当前车速为km/h,预瞄距离，期望速度
        double limitParamChangeUnit(double param,double max_param ,double min_param);//限制幅值函数
        double CalculateCur(geometry_msgs::Point P1,geometry_msgs::Point P2,geometry_msgs::Point P3);//计算曲率
  

    public:
        icvCarlaControl(/* args */);
        ~icvCarlaControl();
        /*函数申明*/        
        void run_follower(double *out_steerangle);
        void run_speedcontrol(double *out_throttle,double *out_brake,double tar_v);        
        //数据input函数
        void  data2file(); 
        //更新车辆坐标，车辆底盘反馈（车速，方向盘角度）
        void SetVehicleStatus(double _x,double _y,double _theta,double _speed,double _vehicle_steerangle)
        {
            Current_Point.x=_x;
            Current_Point.y=_y;
            Current_Point.theta=_theta;
            Current_Point.v=_speed;
            _vehicle_wheel_angle=_vehicle_steerangle;              
            current_pose_flag=true;
            setVehiclestauts_flag=true;                       
        }
        //更新路径信息
        void setWaypoints(const nav_msgs::Path &msg_path)
        {
            _path_x.clear();
            _path_y.clear();
            for(int i=0; i<msg_path.poses.size();i++)
            {
                _path_x.push_back(msg_path.poses[i].pose.position.x);
                _path_y.push_back(msg_path.poses[i].pose.position.y);
                //用于误差计算存储
                Waypoints[i].x=msg_path.poses[i].pose.position.x;
                Waypoints[i].y=msg_path.poses[i].pose.position.y;
                // cout<<"_path_x= "<<_path_x[i]<<" "<<_path_y[i]<<endl;
            }
            // cout<<"_path_x= "<<_path_x[0]<<" "<<_path_y[0]<<endl;
            cout<<"msg_path.poses.size()   "<<msg_path.poses.size()<<endl;
            waypoints_flag=true;
        }  
        /*传输预瞄点坐标显示rviz*/
        void sendXYpre(double *x,double *y)
        {
            *x=_x_preview_rviz;
            *y=_y_preview_rviz;                     
        }
        
    };    
    
    
}


#endif  // ICV_CARLA_CONTROL_H