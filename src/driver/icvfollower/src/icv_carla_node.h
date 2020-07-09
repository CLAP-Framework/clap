
#ifndef ICV_CARLA_NODE_H
#define ICV_CARLA_NODE_H

#include "icv_carla_control.h"

namespace icv
{
class icvCarlaControlNode
{
private:
    // GPSflag
    bool callback_imu_flag = false;
    bool callback_gpsfix_flag = false;
    bool callback_gps_flag = false;
    bool callack_odom_flag = false;

    // chassisflag
    bool callback_eps_flag = false;
    bool callback_esc_flag = false;
    bool callback_auto_state_flag = false;
    bool callback_auto_state_ex_flag = false;

    // Pathflag
    bool callback_Path_flag = false;

    double PI = 3.141592654;

    double vehiclemode;
    double SpeedReq;

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

    //imu/data
    double Imu_linear_accX = 0;
    double Imu_linear_accY = 0;
    double Imu_linear_accZ = 0;
    float Imu_angle_acc_x = 0;
    float Imu_angle_acc_y = 0;
    float Imu_angle_acc_z = 0;

    //taeget speed parameter ,set in launch file
    double target_speedxp = 0;
    double target_speedcarla = 0;

    //IMU
    double Yaw_carla = 0;

#ifdef __DEBUG__SURPPORT_

    //parameters for generate path function
    double _R_test = 0;      //路径点输入参数转弯半径m
    double _delta_d = 0;     //路径点间隔m
    nav_msgs::Path testPath; //测试路径
#endif

    double xx, yy, xxold, yyold;
    //planning mode desire speed
    double speed_plan_desir;

    //feed back  zhongzhiyun car
    double glo_x, glo_y, glo_yaw;

    //VCU feedback
    float speed_fd_, steerangle_fd_, brake_fd_;
    uint8_t gear_fd_;

    //切换程序测试模式:carla miniauto  小鹏车
    int VehicleMode_flag = 0;

    //chassis data struct
    typedef struct VehicleStatus
    {
        double x;
        double y;
        double theta;
        double steer_rec;
        double speed_rec;
    } VehicleStatus;
    VehicleStatus XpPoint_rec;

    // class
    icvCarlaControl zz;

    //rosiniticvfollower
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::NodeHandle rviz_n;

    // publisher
    ros::Publisher pub1_, pub2_;
    ros::Publisher pub_xp_;
    ros::Publisher pub_vehi_pose = rviz_n.advertise<visualization_msgs::Marker>("/dobot_maker1", 1000);
    ros::Publisher pub_pre_point = rviz_n.advertise<visualization_msgs::Marker>("/pre_point", 1000);
    ros::Publisher pub_follow_error = rviz_n.advertise<geometry_msgs::Vector3>("/follow_error", 1000);
    ros::Publisher pub_path = rviz_n.advertise<nav_msgs::Path>("trajectory", 1, true);
    // subscriber
    ros::Subscriber sub1_, sub2_, sub3_, sub4_, 
        sub_imudata_, sub_zzz_path_, 
        sub_autostateex_, sub_autostate_, sub_esc_status_, sub_eps_status_, sub_vcu_; //车辆状态，gnss，waypoints。

    ros::Subscriber sub_gpsvel_, sub_gpsfix_, sub_gpsodom_;

    ros::Subscriber sub_zzz_eogpose_;
    // constant
    const int LOOP_RATE_; // processing frequency

    // carla callbacks
    void VehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatus &msg);
    void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);
    void callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg);
    void callbackFromodom(const nav_msgs::Odometry &msg);
    void WaypointCallback(const nav_msgs::Path &msg); //receive waypoints from carla global Path
    void publishVehicleinstruction(double steer, double throtle, bool brakeflag);

    //receive path from planner in mini_auto system
#if 0
    void callbackFromWayPoints(const autoware_msgs::LaneConstPtr &msg);
#endif

    //just for test, get path from txt file ,or generate path
    void data_file_input();
    void Path_generator(double R, double delta_d, double headingangle);

    // initializer
    void initForROS();
#if 0
    void callback_vcu(const canbus_msgs::VCUFbk &msg);
#endif
    // functions
    vector<float> resultSet; //error 计算方差误差

    void CarconstructionPub(double steer_send, double SpeedReq); //publish control data to xp vehicle
    void VehicelStatusSend();                                    //类接口数据传输 ,translate chassis data and pose to controlmode
    void pubVechilepose2rviz(VehicleStatus carpose);

    geometry_msgs::Quaternion rpy_to_qua(double Yaw, double Pitch, double Roll);
    double qua_to_rpy(geometry_msgs::Quaternion posedata);

    //chassis callback functions
    void callback_eps(const xpmotors_can_msgs::EPSStatus &msg);
    void callback_esc(const xpmotors_can_msgs::ESCStatus &msg);
    void callback_auto_state(const xpmotors_can_msgs::AutoState &msg);
    void callback_auto_state_ex(const xpmotors_can_msgs::AutoStateEx &msg);

//get path from zzz_planning
#ifdef __ZZZ_SURPPORT__
    void callback_Path(const zzz_planning_msgs::DecisionTrajectory &msg);
    void callback_egopose(const zzz_driver_msgs::RigidBodyStateStamped &msg);
#endif

        //oxford IMU and GPS data callback function
        void callback_gpsfix(const sensor_msgs::NavSatFix &msg);
    void callback_imu(const sensor_msgs::Imu &msg);
    void callback_gpsvel(const geometry_msgs::TwistWithCovarianceStamped &msg);
    void callback_gpsodom(const nav_msgs::Odometry &msg);

    //judge sensors and planner
    bool callback_safe();

public:
    icvCarlaControlNode(/* args */);
    ~icvCarlaControlNode();
    void Noderun();
};
} // namespace icv

#endif // PURE_PURSUIT_CORE_H
