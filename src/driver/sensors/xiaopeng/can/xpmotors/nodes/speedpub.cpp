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
//ros headfile
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <signal.h>
#include "param.h"
#include <sensor_msgs/Imu.h>
//persnal headfile
#include "xpmotors_can_msgs/AutoCtlReq.h"
#include "xpmotors_can_msgs/AutoStateEx.h"
#include "xpmotors_can_msgs/AutoState.h"
#include "xpmotors_can_msgs/ESCStatus.h"
#include "xpmotors_can_msgs/EPSStatus.h"

using namespace std;


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

//回调函数flag
bool config_flag=false;
bool callback_auto_state_ex_flag=false;
bool callback_auto_state_flag=false;
bool callback_esc_flag=false;
bool callback_eps_flag=false;
bool callback_imu_flag=false;
//数据采集
ofstream ofile;                  //定义输出文件


void callback_eps(const xpmotors_can_msgs::EPSStatus &msg);
void callback_Config(const  xpmotors_can_msgs::AutoCtlReq &config);
void callback_auto_state_ex(const xpmotors_can_msgs::AutoStateEx &msg);
void callback_auto_state(const xpmotors_can_msgs::AutoState &msg);
void callback_esc(const xpmotors_can_msgs::ESCStatus &msg);
void callback_imu(const sensor_msgs::Imu &msg);
void MySigintHandler(int sig);

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
    
    //数据采集    
    ofile.open("/home/icv/vehicle/Vehicle data/myfile.txt");     //作为输出文件打开
    ofile<<"time                RRWheelSpd              LFWheelSpd              FWheelSpd               LRWheelSpd              accX   "<<endl;   //标题#include <sensor_msgs/Imu.h>入文件
    
    //ofile.close();
    signal(SIGINT, MySigintHandler);
    //ros::ok()返回false会停止运行，进程终止。
	while(ros::ok())
	{
		xpmotors_can_msgs::AutoCtlReq ctrmsg;	
        ros::Time begin = ros::Time::now();//获取系统时间
        ofile<<begin<<" "<<RRWheelSpd<<"    "<<LFWheelSpd<<"    "<<RFWheelSpd<<"    "<<LRWheelSpd<<"    "<<Imu_accX<<endl;//写入txt数据
	     
        if(!CurDriveMode)
        {
          ROS_INFO("Vehicle is Humanmode!!!");
          ctrmsg.AutoMode=1;//发送自动驾驶模式请求
          ctrmsg.TarSpeedReq=2;
          usleep(1000);
        }
        else if(EPBState)
        {
           for(int i=0;i<20;i++)
            {
                ROS_INFO("EPB is hold!!!");
                ctrmsg.AutoMode=1;
                ctrmsg.TarSpeedReq=0;   //发送速度冲开EPB
                pub.publish(ctrmsg);	//发布控制信号
                      
            }
        } 
        else
        {
            ROS_INFO("Vehicle is Automode!!!");
            ctrmsg.AutoMode=1;
            ctrmsg.TarSpeedReq=SpeedReq=2; 
            ROS_INFO("TarSpeed is %lf",SpeedReq);                
        }
                   
		ROS_INFO("speed=%lf is publish ", ctrmsg.TarSpeedReq);
		pub.publish(ctrmsg);	//发布控制信号
		ros::spinOnce();	
		loop_rate.sleep();	//按前面设置的10Hz频率将程序挂起
	}
 
	return 0;
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
    Imu_accX=msg.linear_acceleration.x;
    Imu_accY=msg.linear_acceleration.y;
    Imu_accZ=msg.linear_acceleration.z;
    callback_imu_flag=true;
}

void MySigintHandler(int sig)
{
	//这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
    ROS_INFO("stop outfilel data!");
    ofile.close();
	ROS_INFO("shutting down!");
	ros::shutdown();
}
  