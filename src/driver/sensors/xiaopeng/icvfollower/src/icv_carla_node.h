
#ifndef ICV_CARLA_NODE_H
#define ICV_CARLA_NODE_H

#include"icv_carla_control.h"


namespace icv 
{
    class icvCarlaControlNode
    {
    private:
        /* data */
        bool callback_imu_flag=false;
        bool callback_gpsfix_flag=false;
        bool callback_gps_flag=false;
        bool callback_eps_flag=false; 
        bool callback_esc_flag=false;
        bool callback_auto_state_flag=false;
        bool callback_auto_state_ex_flag=false;
        bool callback_Path_flag=false;

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
        double target_speedxp=0;
        double target_speedcarla=0;
        //IMU
       double Yaw_carla=0;
        /*测试路径生成函数使用*/
        double _R_test=0;//路径点输入参数转弯半径m
        double _delta_d=0;//路径点间隔m
        nav_msgs::Path testPath;//测试路径
        

        //切换carla和小鹏实车测试
        int carla_flag=0;
        //参数传递结构体
        typedef struct VehicleStatus
        {
            double x;
            double y;
            double theta;
            double steer_rec;
            double speed_rec;
        }VehicleStatus;
        VehicleStatus XpPoint_rec;
        
        // class
        icvCarlaControl zz;
        //rosiniticvfollower        
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;       

        ros::NodeHandle rviz_n; 



        // publisher
        ros::Publisher pub1_;//车辆控制指令
        ros::Publisher pub_xp_;//发布小鹏车指令
        ros::Publisher pub_vehi_pose = rviz_n.advertise<visualization_msgs::Marker>("/dobot_maker1",100);
         ros::Publisher pub_pre_point = rviz_n.advertise<visualization_msgs::Marker>("/pre_point",100);
        ros::Publisher pub_path = rviz_n.advertise<nav_msgs::Path>("trajectory",1, true); 
        // subscriber
        ros::Subscriber sub1_, sub2_, sub3_,sub_imudata_,sub_zzz_path_,sub_gpsvel_,sub_gpsfix_,sub_autostateex_,sub_autostate_,sub_esc_status_,sub_eps_status_;//车辆状态，gnss，waypoints。

        // constant
        const int LOOP_RATE_;  // processing frequency  
        // carla callbacks
        // void VehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatus &msg);   //carla     实车屏蔽
        // void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);//实车屏蔽
        // void WaypointCallback(const nav_msgs::Path &msg);//实车屏蔽
        //  void publishVehicleinstruction(double steer,double throtle,bool brakeflag);//方向盘、油门值、制动指令实车屏蔽
        // initializer
        void initForROS();

        // functions
       
        void CarconstructionPub(double steer_send,double SpeedReq);
        void VehicelStatusSend();//类接口数据传输
        void data_file_input();//读取txt路径点信息
        void Path_generator(double R,double delta_d,double headingangle);//路径点生成
        void pubVechilepose2rviz(VehicleStatus carpose);

        double qua_to_rpy(geometry_msgs::Quaternion posedata);
        void callback_gpsfix(const sensor_msgs::NavSatFix &msg);
        void callback_imu(const sensor_msgs::Imu &msg);
        void callback_gpsvel(const geometry_msgs::TwistWithCovarianceStamped &msg);
        void callback_eps(const xpmotors_can_msgs::EPSStatus &msg);
        void callback_esc(const xpmotors_can_msgs::ESCStatus &msg);
        void callback_auto_state(const xpmotors_can_msgs::AutoState &msg);
        void callback_auto_state_ex(const xpmotors_can_msgs::AutoStateEx &msg);
        void callback_Path(const zzz_planning_msgs::DecisionTrajectory &msg);
    public:
        icvCarlaControlNode(/* args */);
        ~icvCarlaControlNode();
        void Noderun();
        
    };
    

    
}



#endif  // PURE_PURSUIT_CORE_H
