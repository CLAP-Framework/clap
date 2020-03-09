/**
 * @file auto_state_node.cpp
 * @brief sub msgs:
 * 	Auto state, EPS status, ESC status，Imu msgs,
 *       pub msgs:
 *  Crtolmsgs
 * @author xiang.zhang@novauto.com.cn
 * @version 0.0.1
 * @date 2020-1-12
 */
#include <iostream>
#include <fstream>
#include <string>
#include<cmath>
//ros headfile
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <signal.h>
#include "param.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
//persnal headfile
#include "xpmotors_can_msgs/AutoCtlReq.h"
#include "xpmotors_can_msgs/AutoStateEx.h"
#include "xpmotors_can_msgs/AutoState.h"
#include "xpmotors_can_msgs/ESCStatus.h"
#include "xpmotors_can_msgs/EPSStatus.h"
#include "zzz_planning_msgs/DecisionTrajectory.h"

#include <geographic_msgs/GeoPointStamped.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace std;
double PI= 3.141592654;

double vehiclemode;
double SpeedReq ;
//AutoStateEx
int StateTurningLight;
int CurDriveMode;
int StateBraking;
//AutoState
int EPBState;
int GearState;
int BrkPedal;
int AccPedal;
//esc
double RRWheelSpd;
double LFWheelSpd;
double RFWheelSpd;
double LRWheelSpd;
//eps
double AngleSpd;
double Angle;
double StrngWhlTorq;
//IMU
double Imu_accX;
double Imu_accY;
double Imu_accZ;
double Yaw;//航向角
double Roll;
double Pitch;
//GPS
double latitude;//纬度
double longitude;
//轨迹点长度
int m=428;//轨迹点长度   
float cte_D=0;
float cte_A;
//回调函数flag
bool config_flag=false;
bool callback_auto_state_ex_flag=false;
bool callback_auto_state_flag=false;
bool callback_esc_flag=false;
bool callback_eps_flag=false;
bool callback_imu_flag=false;
bool callback_gps_flag=false;
bool callback_Path_flag=false;
//数据采集
ofstream ofile;                  //定义输出文件

//结构体定义
typedef struct Path
{
    float x=0;
    float y=0;
    float z=0;
    float theta=0;//度。
    float pitch=0;
    float roll=0;
    float v=0;//车辆速度m/s
}Pathpoint;
Pathpoint Waypoints[10240];
Pathpoint Current_Point;//车辆当前状态
void    callback_eps(const xpmotors_can_msgs::EPSStatus &msg);
void    callback_Config(const  xpmotors_can_msgs::AutoCtlReq &config);
void    callback_auto_state_ex(const xpmotors_can_msgs::AutoStateEx &msg);
void    callback_auto_state(const xpmotors_can_msgs::AutoState &msg);
void    callback_esc(const xpmotors_can_msgs::ESCStatus &msg);
void    callback_imu(const sensor_msgs::Imu &msg);
void    callback_gpsfix(const sensor_msgs::NavSatFix &msg);
void    callback_Path(const zzz_planning_msgs::DecisionTrajectory &msg);
void    data_file_input();
void    MySigintHandler(int sig);
void    PID_init();
float   D_PID_realize();
float   A_PID_realize();
void    pathdeal();
double qua_to_rpy(geometry_msgs::Quaternion posedata);
int main(int argc, char **argv)
{
    
	ros::init(argc, argv, "Speedtest");	
	ros::NodeHandle n;	
	ros::Publisher pub = n.advertise<xpmotors_can_msgs::AutoCtlReq>("/xp/auto_control", 1000);	//告诉系统要发布话题了，话题名为“str_message”，类型为std_msgs::String，缓冲队列为1000。
  ros::Rate loop_rate(20);	
  ros::Subscriber sub1_ = n.subscribe("config/waypoint_follower", 10, callback_Config);
  ros::Subscriber sub2_ = n.subscribe("/xp/auto_state_ex", 10, callback_auto_state_ex);
  ros::Subscriber sub3_ = n.subscribe("/xp/auto_state", 10, callback_auto_state);
  ros::Subscriber sub4_ = n.subscribe("/xp/esc_status", 10, callback_esc);
  ros::Subscriber sub5_ = n.subscribe("/xp/eps_status", 10, callback_eps);
  ros::Subscriber sub6_ = n.subscribe("/imu/data", 10, callback_imu);
  ros::Subscriber sub7_ = n.subscribe("gps/fix", 10, callback_gpsfix);
  ros::Subscriber sub8_=n.subscribe("/zzz/planning/decision_trajectory",10,callback_Path);
  //getparam from launchfile
  double target_speed;
  float steer_send=0;
  float steer_send_old=0;
  ros::param::get("~target_speed",target_speed);
  
  //数据采集   
  ofile.open("/home/icv/vehicle/Vehicle data/3-3-30kmdata.txt");     //作为输出文件打开   
  PID_init();//PID参数初始化  
  //ofile.close();
  signal(SIGINT, MySigintHandler); 
	while(ros::ok())
	{
    xpmotors_can_msgs::AutoCtlReq ctrmsg;	
    ros::Time begin = ros::Time::now();//获取系统时间
    ofile<<setiosflags(ios::fixed)<<setprecision(3)<<steer_send<<" "<<cte_A<<" "<<cte_D<<" "<<LRWheelSpd<<" "<<Current_Point.x<<" "<<Current_Point.y<<endl;//写入txt数据 
    if(callback_imu_flag&callback_gps_flag&callback_Path_flag)
    // if(1)
    {      
      pathdeal();
      float steer1=D_PID_realize();
      float steer2=A_PID_realize();
      steer_send=steer1+steer2;
      float factor=0.8;
      steer_send=factor*steer_send_old+(1-factor)*steer_send;
      ctrmsg.EPSAngleReq=steer_send;
      steer_send_old=steer_send;
      cout<<"steer1 "<<steer1<<"steer2 "<<steer2<<endl;
      // cout<<"cte_d "<<cte_D<<"   cte_A  "<<cte_A<<"steer"<<steer_send<<endl;
      if(!CurDriveMode)
      {

        ROS_INFO("Vehicle is Humanmode!!!");
        ctrmsg.EPSAngleReq=0;
        ctrmsg.AutoMode=1;//发送自动驾驶模式请求
        ctrmsg.TarSpeedReq=1;
        usleep(1000);       
      }
      else if(EPBState)
      {
          for(int i=0;i<50;i++)
          {
              ROS_INFO("EPB is hold!!!");
              ctrmsg.AutoMode=1;
              ctrmsg.TarSpeedReq=1;   //发送速度冲开EPB
              usleep(50000);
              pub.publish(ctrmsg);	//发布控制信号
                    
          }
          for(int i=0;i<20;i++)
          {
              ROS_INFO("EPB is hold!!!");
              ctrmsg.AutoMode=1;
              ctrmsg.TarSpeedReq=0;   //发送速度冲开EPB
              usleep(5000);
              pub.publish(ctrmsg);	//发布控制信号
                    
          }
      } 
      else
      {
          ROS_INFO("Vehicle is Automode!!!");
          ctrmsg.AutoMode=1;
          ctrmsg.TarSpeedReq=SpeedReq;      //m/s                     
      }  
      cout<<"sterangle= "<<ctrmsg.EPSAngleReq<<endl;       
      pub.publish(ctrmsg);	//发布控制信号
    }    
    loop_rate.sleep();	//按前面设置的10Hz频率将程序挂起
    ros::spinOnce();	   
	} 
	return 0;
}
//定义结构体
struct _pid{
       float SetSteerAngle;//定义方向盘请求值
       float ActualSteerAngle;//实际方向盘角度
       float cte_d;//侧向偏差
       float cte_yaw;//预瞄点角度偏差
       float l_cte_d;//上一个侧向偏差
       float l_cte_yaw;//上一个预瞄点角度偏差
       float Kp,Ki,Kd;//比例、积分、微分系数
       float integral;//定义积分值
       float voltage;//定义值(控制执行器的变量)
       float cte_max;//定义积分开启时的测偏差上限
       float cte_min;//定义积分开启时的角度偏差下限
       int index;// 定义积分环节系数
};
_pid D_pid;
_pid A_pid;


//定义函数            
void    PID_init()
{
  cout<<"PID_init begin \n";
  float D_Kp,D_Ki,D_Kd,A_Kp,A_Ki,A_Kd;
  ros::param::get("~D_Kp",D_Kp);
  ros::param::get("~D_Ki",D_Ki);
  ros::param::get("~D_Kd",D_Kd);
  ros::param::get("~A_Kp",A_Kp);
  ros::param::get("~A_Ki",A_Ki);
  ros::param::get("~A_Kd",A_Kd);
  cout<<D_Kp<<D_Ki<<D_Kd<<A_Kp<<A_Ki<<A_Kd<<endl; 
  D_pid.SetSteerAngle=0.0;
  D_pid.ActualSteerAngle=0.0;
  D_pid.cte_d=0.0;
  D_pid.cte_yaw=0.0;
  D_pid.integral=0.0;
  D_pid.Kp=D_Kp;//=-50.8;
  D_pid.Ki=D_Ki;//0
  D_pid.Kd=D_Kd;//0
  D_pid.cte_max=0.5;
  D_pid.cte_min=-0.5;

  A_pid.SetSteerAngle=0.0;
  A_pid.ActualSteerAngle=0.0;
  A_pid.cte_d=0.0;
  A_pid.cte_yaw=0.0;
  A_pid.integral=0.0;
  A_pid.Kp=A_Kp;//=7;
  A_pid.Ki=A_Ki;//0;
  A_pid.Kd=A_Kd;//;  
  cout<<"PID_init end \n";
 }

float  D_PID_realize(){  
  D_pid.cte_d=cte_D;
  if(abs(D_pid.cte_d)>0.5){
  D_pid.index=0;
  D_pid.integral=0;
  }
  else {
  D_pid.index=1;
  D_pid.integral+=D_pid.cte_d;       
  }                  
  D_pid.voltage=D_pid.Kp*D_pid.cte_d+D_pid.index*D_pid.Ki*D_pid.integral+D_pid.Kd*(D_pid.cte_d-D_pid.l_cte_d);
  D_pid.l_cte_d=D_pid.cte_d;
  D_pid.ActualSteerAngle=D_pid.voltage*1.0;         
  return D_pid.ActualSteerAngle;
}

float    A_PID_realize(){ 
  A_pid.cte_yaw=cte_A;      
  if(abs(A_pid.cte_yaw>4))
  {
  A_pid.integral=0; 
  }          
  A_pid.integral+=A_pid.cte_yaw;         
  A_pid.voltage=A_pid.Kp*A_pid.cte_yaw+A_pid.Ki*A_pid.integral+A_pid.Kd*(A_pid.cte_yaw-A_pid.l_cte_yaw);
  A_pid.l_cte_yaw=A_pid.cte_yaw;
  A_pid.ActualSteerAngle=A_pid.voltage*1.0;       
  return A_pid.ActualSteerAngle;
}

void pathdeal()
{    
  //"pathdeal to calculte cte_d  cte_a "
  int plo_i=0;//曲线拟合的索引点
  float ctd=100;   
  float dis_a=0;  
  float x_len=0;    
  float x_plo_start=0;
  float d_ditance=0;//x的离散补偿
  float sig_d=0;
  int sgn=0; 
  for(int i=0;i<m;i++)
  {
    
      dis_a=sqrt(pow((Current_Point.x-Waypoints[i].x),2)+(pow((Current_Point.y-Waypoints[i].y),2)));        
      if(dis_a>=ctd)
        { 
            ctd=ctd;
            plo_i=plo_i;
        }
      else
        { 
        ctd=dis_a;
        plo_i=i;
        } 
  }   
  // cout<<"ploi "<<plo_i<<" ctd "<<ctd<<endl;
  sig_d=((Waypoints[plo_i].x-Current_Point.x)*(Waypoints[plo_i+1].y-Current_Point.y))-((Waypoints[plo_i].y-Current_Point.y)*(Waypoints[plo_i+1].x-Current_Point.x));  
  if(sig_d>=0)    
  {
    if(sig_d>0)
    {
      sgn=1;    
    }
    else
      {
        sgn=0;
      }                
  }
  else
  {
    sgn=-1;
  }       
  cte_D=ctd*sgn;      
  double ys=fmod(((Waypoints[plo_i+20].theta-Current_Point.theta)/180.0*PI+PI),(2*PI));
  if (ys<0)
  {
    ys=ys+2*PI;
  } 
  cte_A=0-(ys-PI)/PI*180.0;
  SpeedReq=Waypoints[plo_i+1].v;  
}
void    callback_Path(const zzz_planning_msgs::DecisionTrajectory &msg)
{
  for(int i=0;i<msg.trajectory.poses.size();i++)
  {
    Waypoints[i].x=msg.trajectory.poses[i].pose.position.x;
    Waypoints[i].y=msg.trajectory.poses[i].pose.position.y;
    // cout<<"Waypoints.x= "<<Waypoints[i].x<<"Waypoints.y= "<<Waypoints[i].y<<endl;
    Waypoints[i].theta=qua_to_rpy(msg.trajectory.poses[i].pose.orientation); 
    if(Waypoints[i].theta<0)
    {
      Waypoints[i].theta+=360;
    }
    // cout<<"Waypoints[i].theta=  "<<Waypoints[i].theta<<endl;//zx
    Waypoints[i].v=msg.desired_speed;
  }
  callback_Path_flag=true;
}
double qua_to_rpy(geometry_msgs::Quaternion posedata)
{
    float w = posedata.w;
    float x = posedata.x;
    float y = posedata.y;
    float z = posedata.z;

    float R = atan2((2*(w*x+y*z)),(1-2*(x*x+y*y)));
    float P = asin(2*(w*y-z*x));
    float Y = atan2((2*(w*z+x*y)),(1-2*(z*z+y*y)));
    Y=Y*180/PI;
 
    return Y;
}
void callback_Config(const  xpmotors_can_msgs::AutoCtlReq &config)
{
    vehiclemode  = config.AutoMode;
    SpeedReq = config.TarSpeedReq;
    ROS_INFO("ConfigSpeed is %lf",SpeedReq);      
    bool config_flag=false;

}

void callback_auto_state_ex(const xpmotors_can_msgs::AutoStateEx &msg)
{

    StateTurningLight=msg.StateTurningLight;
    CurDriveMode=msg.CurDriveMode;
    StateBraking=msg.StateBraking;
    callback_auto_state_ex_flag=true;
  
}

void callback_auto_state(const xpmotors_can_msgs::AutoState &msg)
{
    EPBState=msg.EPBState;
    GearState=msg.GearState;
    BrkPedal=msg.BrkPedal;
    AccPedal=msg.AccPedal; 
    callback_auto_state_flag=true;
  
}

void callback_esc(const xpmotors_can_msgs::ESCStatus &msg)
{
    RRWheelSpd=msg.RRWheelSpd;
    LFWheelSpd=msg.LFWheelSpd;
    RFWheelSpd=msg.RFWheelSpd;
    LRWheelSpd=msg.LRWheelSpd;
    callback_esc_flag=true;
 
}

void callback_eps(const xpmotors_can_msgs::EPSStatus &msg)
{

    AngleSpd=msg.AngleSpd;
    Angle=msg.Angle;
    StrngWhlTorq=msg.StrngWhlTorq;
    callback_eps_flag=true; 

}

void callback_imu(const sensor_msgs::Imu &msg)
{
    double w,x,y,z;
    Imu_accX=msg.linear_acceleration.x;
    Imu_accY=msg.linear_acceleration.y;
    Imu_accZ=msg.linear_acceleration.z;

    Yaw=qua_to_rpy(msg.orientation);
    
       if(Yaw<0)
    {
        Yaw=-1*Yaw;
    }
    else if (Yaw>0)
    {
        Yaw=360-Yaw;
    } 
    // cout<<"Yaw= "<<Yaw<<endl;//zx
    Current_Point.theta=Yaw;
    callback_imu_flag=true;
}

void callback_gpsfix(const sensor_msgs::NavSatFix &msg)
{
    latitude=msg.latitude;
    longitude=msg.longitude;
    callback_gps_flag=true;
    // double scale[2];
    // const double lat1 = 40.00654925; //GPScede 40.00683544,116.32814736
    // const double lon1 = 116.3281271;    
    
    // float a = 6378137.0;
    // float f = 298.2572;
    // float b = (f - 1) / f * a;
    // float e2 = (a * a - b * b) / (a * a);
    // float A = a * (1 - e2) / pow((1 - e2 * pow(sin(lat1 / 180.0 * PI), 2)), 1.5);
    // float B = a * cos(lat1 / 180.0 * PI) / sqrt(1 - e2 * pow(sin(lat1 / 180.0 * PI), 2));
    // scale[1] = B * 1.0 / 180.0 * PI;
    // scale[2] = A * 1.0 / 180.0 * PI;
    // Current_Point.x = (latitude - lon1) * scale[1]; //转换为坐标x
    // Current_Point.y = (longitude - lat1) * scale[2];
    // double f;
    // double a,b,e,N,M,H,A,B,C,T,FE,X,Y,k0,L,L0;
    // B=PI*latitude/180.0;
    // L=PI*longitude/180.0;
    // a=6378137;
    // b=6356752.3142;
    // k0=1;
    // H=20;
    // L0=PI*117/180.0;
    // f=sqrt((a/b)*(a/b)-1);
    // e=sqrt(1-(b/a)*(b/a));
    // N=(a*a/b)/sqrt(1+f*f*cos(B)*cos(B));
    // M=a*((1-e*e/4-3*pow(e,4)/64-5*pow(e,6)/256)*B-(3*e*e/8+3*pow(e,4)/32+45*pow(e,6)/1024)*sin(2*B)+(15*pow(e,4)/256+45*pow(e,6)/1024)*sin(4*B)-sin(6*B)*35*pow(e,6)/3072);
    // A=(L-L0)*cos(B);
    // C=f*f*cos(B)*cos(B);
    // T=tan(B)*tan(B);
    // FE=500000+H*1000000;
    // Y=k0*(M+N*tan(B)*(A*A/2+(5-T+9*C+4*pow(C,2))*pow(A,4)/24)+(61-58*T+T*T+270*C-330*T*C)*pow(A,6)/720);
    // X=FE+k0*N*(A+(1-T+C)*pow(A,3)/6+(5-18*T+T*T+14*C-58*T*C)*pow(A,5)/120);    

    // //ximenjiayouzhan wei yuandian 
    // X=X-20441065.1238410;
    // Y=Y-4429649.9202231;

    // utm coordinates
    geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
    gps_msg->position.latitude = msg.latitude;
    gps_msg->position.longitude = msg.longitude;
    gps_msg->position.altitude = msg.altitude;

    geodesy::UTMPoint utm;
    geodesy::fromMsg(gps_msg->position, utm);
    Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);

    // X=X-4429649.9202231;
    // Y=Y-20441065.1238410;
    // Current_Point.x=X;
    // Current_Point.y=Y;
    //442867,4427888)
    Current_Point.x=utm.easting-442867;
    Current_Point.y=utm.northing-4427888;

    Current_Point.x=Current_Point.x+0.3*sin(Yaw*PI/180);
    Current_Point.y=Current_Point.y+0.3*cos(Yaw*PI/180);
    //cout<<Current_Point.x<<"   kkkk   "<<Current_Point.y<<endl;

}
void MySigintHandler(int sig)
{
	//这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
    ROS_INFO("stop outfilel data!");
    ofile.close();
	ROS_INFO("shutting down!");
	ros::shutdown();
}

