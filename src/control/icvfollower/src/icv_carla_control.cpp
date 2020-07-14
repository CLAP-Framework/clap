
/*author :zx
date 2020.2.25
function ： test Vehicle mode 
*/
#include "icv_carla_node.h"
#include "icv_carla_control.h"

using namespace std;
using namespace Eigen;

namespace icv
{
    icvCarlaControl::icvCarlaControl( )

	{
   	 /**init param of control and use yaml  config*/
   	 controlInit();
	}

	icvCarlaControl::~icvCarlaControl()
	{
	}

template <typename T>
T getParam(const YAML::Node &node, const string &name, const T &defaultValue)
{
    T v;
    try
    {
        v = node[name].as<T>();
    }
    catch (std::exception e)
    {
        v = defaultValue;
    }
    return v;
}
#ifndef MAX_PATH
#define MAX_PATH 260
#endif
/**初始化话控制参数，调用yaml文件*/
void icvCarlaControl::controlInit()
{
    //加载参数文件
    char config_path[MAX_PATH] = {0};

    strcpy(config_path, getenv("ZZZ_ROOT"));
   
    strcat(config_path, "/zzz/src/control/icvfollower/config/Controlconfig.yaml");

    // src/driver/icvfollower/config/Controlconfig.yaml
    YAML::Node dset_config = YAML::LoadFile(config_path);

    //Vehiclemodel,Controlparam two key_value
    YAML::Node Vehiclemodel = dset_config["Vehiclemodel"];
    YAML::Node Controlparam = dset_config["Controlparam"];

    /****VehicleModle*****/
    lf = getParam<float>(Vehiclemodel, "_lf", 0);
    lr = getParam<float>(Vehiclemodel, "_lr", 0);
    L = lr + lf;
    m = getParam<float>(Vehiclemodel, "_m", 0);             //mass of Vehicle
    Ccf = getParam<float>(Vehiclemodel, "_Ccf", 0);         //前轮侧偏刚度
    Ccr = getParam<float>(Vehiclemodel, "_Ccr", 0);         //后轮侧偏刚度
    Clf = getParam<float>(Vehiclemodel, "_Clf", 0);         //前轮纵向刚度
    Clr = getParam<float>(Vehiclemodel, "_Clr", 0);         //后轮纵向刚度
    delta_f = getParam<float>(Vehiclemodel, "_delta_f", 0); //前轮偏角度
    Sf = getParam<float>(Vehiclemodel, "_Sf", 0);           //前轮纵向滑移率
    Sr = getParam<float>(Vehiclemodel, "_Sr", 0);           //后轮纵向滑移率
    I = getParam<float>(Vehiclemodel, "_I", 0);             //车辆质心转动惯量
    wheel_base = getParam<float>(Vehiclemodel, "_L", 0);    //车辆轴距
    g = getParam<float>(Vehiclemodel, "_g", 0);
    R_min = getParam<float>(Vehiclemodel, "_Rmin", 0); //最小转向半径
    K = getParam<float>(Vehiclemodel, "_K", 0);        //方向盘传动比

    /********Controlparam***************/
    __distance_pre = getParam<float>(Controlparam, "_distance_pre", 0); //mass of Vehicle

    _p1_post = getParam<float>(Controlparam, "_P1_post", 0);
    _p2_post = getParam<float>(Controlparam, "_P2_post", 0);
    _p3_post = getParam<float>(Controlparam, "_P3_post", 0);
    _p4_post = getParam<float>(Controlparam, "_P4_post", 0);
    _p5_post = getParam<float>(Controlparam, "_P5_post", 0);
    _p6_post = getParam<float>(Controlparam, "_P6_post", 0);
    _p7_post = getParam<float>(Controlparam, "_P7_post", 0);
    _p8_post = getParam<float>(Controlparam, "_P8_post", 0);
    _p9_post = getParam<float>(Controlparam, "_P9_post", 0);

    _dis_pre_max = getParam<float>(Controlparam, "_dis_pre_max", 0);
    _dis_pre_min = getParam<float>(Controlparam, "_dis_pre_min", 0);
    _wheel_angle_degree_max = getParam<float>(Controlparam, "_wheel_angle_degree_max", 0);
    _wheel_angle_degree_sec = getParam<float>(Controlparam, "_wheel_angle_degree_sec", 0);
    _t_preview = getParam<float>(Controlparam, "_t_preview", 2.8);
    _t_preview_turn = getParam<float>(Controlparam, "_t_preview_turn", 2.8);
    _v_pid.Kp = getParam<float>(Controlparam, "_v_pid.Kp", 0.15);
    _v_pid.Kd = getParam<float>(Controlparam, "_v_pid.Kd", 0);
    _v_pid.Ki = getParam<float>(Controlparam, "_v_pid.Ki", 0.01);
    _delta_T = getParam<float>(Controlparam, "_delta_T", 0.05);
    _kappa = getParam<float>(Controlparam, "_kappa", 0);
    _smoothsteernum = getParam<float>(Controlparam, "_smoothsteernum", 0);
    point_distancemax = getParam<float>(Controlparam, "point_distancemax", 0);
    car_path_dis = getParam<float>(Controlparam, "car_path_dis", 0);
    _KK = getParam<float>(Controlparam, "_KK", 0);
    _l1 = getParam<float>(Controlparam, "_l1", 0);
    _l2 = getParam<float>(Controlparam, "_l2", 0);
    _l3 = getParam<float>(Controlparam, "_l3", 0);
    _l4 = getParam<float>(Controlparam, "_l4", 0);
    _T = getParam<float>(Controlparam, "_T", 0);
}
// ////////////////////////////////主程序////////////////////////////////////////////////////////////
/***********************************************************************************************/
void icvCarlaControl::run_follower(double *out_steerangle)
{

    //data2file();
    // pathdeal();
    clock_t begin, end;
    begin = clock();

    //PathSmoothCheck();                 //检测长度和曲率平滑度.
    //follower_flag = Pathfollowcheck(); //轨迹跟踪检测
    follower_flag = true;
    if (follower_flag)
    {
        delta_f = ControlStr(Current_Point.x, Current_Point.y, Current_Point.theta, Current_Point.v) / K /180*PI; //
        // delta_f = ControlStr2(Current_Point.x, Current_Point.y, Current_Point.theta, Current_Point.v); //

        delta_f = smoothfilter(delta_f, _smoothsteernum);
        *out_steerangle = delta_f;
    }
    else
    {
        ROS_ERROR("Pathpoint is error!!! ");
    }
    end = clock();
    sample_time = (end - begin) / CLOCKS_PER_SEC;
}

bool icvCarlaControl::Pathlength(double *pathlength)
{
    //TODO::check distance between points of Path
    //input: Path length parameter
    //output: Path status  ture is good,false is error

    float path_length = 0;
    float dis_points = 0;
    dis_points = sqrt(pow((Current_Point.x - _path_x[1]), 2) + pow((Current_Point.y - _path_y[1]), 2));
    if (dis_points > car_path_dis)
    {

        ROS_ERROR("Path is error!!!!  far away egoVehicle !!!!! distance = %lf ", dis_points);
        return false;
    }
    for (int i = 0; i < (_path_x.size() - 1); i++)
    {
        dis_points = sqrt(pow((_path_x[i + 1] - _path_x[i]), 2) + pow((_path_y[i + 1] - _path_y[i]), 2));
        path_length += dis_points;
        if (dis_points > point_distancemax)
            ROS_WARN("somepoints is too far away !!!!!");
    }
    *pathlength = path_length;
    return true;
}
//check waypoints  Path
bool icvCarlaControl::Pathfollowcheck()
{
    //TODO::check the trajector status
    //input
    //output:: ture or false
    Pathpoint check_point = Current_Point;
    float check_x = Current_Point.x;
    float check_y = Current_Point.y;
    float check_delta_f = 0;
    float check_heading = Current_Point.theta;
    int check_size = Waypoints_size;
    int check_num = 0;
    float check_minidistance = 0; //仿真器中的实时误差

    // if (!PathSmoothCheck())
    //     return false;

    //TODO::check length of waypoints!
    if (check_size < path_size_min)
    {
        ROS_WARN("Waypoint is to short!!!!!path_size = %ld", check_size);
        return false;
    }

    //TODO::check Path distance
    double check_pathlength = 0;
    // if (!Pathlength(&check_pathlength))
    //     return false;
    //TODO::check the useful of Path;
    for (int i = 0; i < 20; i++)
    {
        Vehiclemodeupdate2(check_point, check_delta_f, 0.01, &check_x, &check_y, &check_heading);
        check_delta_f = -ControlStr(check_x, check_y, check_heading, 10 / 3.6) / K / 180 * PI; //速度应使用反馈的车速
        check_minidistance = find_min_point(check_x, check_y, &check_num);
        // pathdeal(); //calculate follow error
        data2file();
        if (check_minidistance > check_maxerrordis)
        {
            ROS_ERROR(" Path is error  !!!!! ");
            return false;
        }
    }

    return true;
}

double icvCarlaControl::smoothfilter(double param, int smoothsize)
{
    //TODO::smooth filter
    //input:: smooth target param, the number of smoothsize
    //output:: parameter after smooth

    if (filterdata.size() > smoothsize)
    {
        for (int i = 0; i < smoothsize - 1; i++)
        {
            filterdata[i] = filterdata[i + 1];
        }
        filterdata[smoothsize - 1] = param;
    }
    else
    {
        filterdata.push_back(param);
    }
    double sum = std::accumulate(std::begin(filterdata), std::end(filterdata), 0.0);
    double mean = sum / filterdata.size(); //均值
    return mean;
}
double icvCarlaControl::ControlStr2(float _vehicle_x, float _vehicle_y, float _vehicle_heading_theta, float _vehicle_v)
{
    //todo::follower
    //parameters defines
    /*
    {X,Y,Z} ENU inertial coordinate frame 
    {xyz}:local coordinate frame 
    (x,y)longtitudinal and lateral  position of the origin of the {xyz} cooordinate to the feont fixed point 
    V: velocity at c.g of vehicle ()
    x_dot = Vx: longitudinal velocity at c.g. of vehicle 
    y_dot = Vy: lateral velocity at c.g. of vehicle
    m: total mass of vehicle 
    Iz: yaw moment of inertia of vehicle
    lf:(lr)loinggitudinal distance from c.g. to front (rear) tires
    phi_d: desired yaw rate from the road
    phi:yaw,heading,angle of vehicle in global axis
    phi_dot = : yaw rate of vehicle
    beta :vechile slip angle at c.g. of vehicle 
    alpha : (alphi_f,alphi_r ) slip angle at (front,rear)wheel tires
    sigma: steering angle 
    Ca:(Caf Car) cornering stiffness of (front rear) tire
    Fy: (Fyf, Fyr) lateral tire force (on front ,rear tires)
    R: turn radius of vehicle or radius of road 
    curvature = 1/R :curvature of road
    L: look-ahead diatance from c.g. to look-ahead point 
    e_y = y -yd: lateral position error w.r.t reference   ,current position lateral error;
    e_phi = phi_d - phi: yaw angle error w.r.t road      
    u : control output parameters
    b1???
    b2???

    X1:误差矩阵4*4
    X1_m:误差估计值矩阵4*4
    X2:前馈项4*1
    X2_m;前馈估计值4*1
    */

    float X, Y, Z;
    static float V = 0, Vx = 0, Vy = 0;
    Vx = _vehicle_v + 0.1; //m/s
    static float x_dot = 0, y_dot = 0;
    static float y = 0, y_d = 0, y_d_dot = 0; //侧向误差
    // float m;
    static float Iz = I;
    // float lf=1.5, lr=1.3;
    static float phi_d = 0, phi_d_dot = 0, phi = 0, phi_dot = 0;
    static float beta = 0, alpha = 0, sigma = 0;
    static float Caf = Ccf;
    static float Car = Ccr;

    static float R, curvature;
    static float LL = 0; //distance from current position to lookahead point,
    static float e_y = 0, e_phi_d = 0;
    // cout <<"1111   "<< Vx << endl;
    float a22 = -((2 * Caf + 2 * Car) / (m * Vx));
    static float a23 = (2 * Caf + 2 * Car) / m;
    float a24 = -((2 * Caf * lf - 2 * Car * lr) / (m * Vx));
    float a42 = -((2 * Caf * lf - 2 * Car * lr) / (Iz * Vx));
    static float a43 = (2 * Caf * lf - 2 * Car * lr) / Iz;
    float a44 = -((2 * Caf * lf * lf + 2 * Car * lr * lr) / (Iz * Vx));
    float b21 = (2 * Caf) / (m * Vx);
    static float b41 = (2 * Caf * lf) / Iz;
    // cout << " B1  B2" << b21 << " " << b41 << endl;
    static float u = 0;
    static float b1 = b21, b2 = b41;
    static float u_last = 0;
    static float T = _T;
    static float KK = _KK; //K取负值

    float l1 = 10, l2 = 20, l3 = 0.01, l4 = 0.01;
    
    //TODO:: !!!!!!!!!need to change find_min_point function

    _vehicle_x = _vehicle_x + 0.6 * cos(_vehicle_heading_theta * PI / 180);
    _vehicle_y = _vehicle_y + 0.6 * sin(_vehicle_heading_theta * PI / 180);
    //control input::当前点侧向误差, 当前点的路径航向,航向偏差
    //横向速度,期望的横向速度?
    //当前点车辆航向角和期望车辆航向角.
    //当前横摆角速度,期望横摆角速度.

    //TODO:: 当前侧向偏差求解,航向角偏差求解模块
    int nextpoint_num = 0;
    e_y = find_min_point(_vehicle_x, _vehicle_y, &nextpoint_num);
    // ctedata[0] = e_y;
    //TODO::计算车辆当前位置与最近的路径点航向角偏差
    static float path_yaw = 0;
    static float phi_error = 0;
    phi_d = atan2(_path_y[nextpoint_num + 1] - _path_y[nextpoint_num], _path_x[nextpoint_num + 1] - _path_x[nextpoint_num]) / PI * 180;
    phi = _vehicle_heading_theta;
    double ys = fmod(((phi - phi_d) / 180.0 * PI + PI), (2 * PI));
    if (ys < 0)
        ys = ys + 2 * PI;
    e_phi_d = 0 - (ys - PI) / PI * 180.0;
    cout << " phi " << phi << "   phi_d  " << phi_d << endl;
    //TODO::计算横摆角速度和期望角速度偏差
    //测量值从oxfordIMU获取
    geometry_msgs::Point p1, p2, p3;
    if (nextpoint_num < (_path_x.size() - 20))
    {
        p1.x = _path_x[nextpoint_num];
        p2.x = _path_x[nextpoint_num + 10];
        p3.x = _path_x[nextpoint_num + 20];
        p1.y = _path_y[nextpoint_num];
        p2.y = _path_y[nextpoint_num + 10];
        p3.y = _path_y[nextpoint_num + 20];
    }
    else
        ROS_WARN("three point kappa is the lastpoint!!!");

    double kappa = CalculateCur(p1, p2, p3) + 0.000001;

    //判断方向时,使用的预瞄点和车辆当前点判断曲率方向.
    float _d_dis_k = _t_preview * _vehicle_v + _d_0;
    _d_dis_k = limitParamChangeUnit(_d_dis_k, _dis_pre_max, _d_dis_min_k);
    double _x_preview_cur = _vehicle_x + _d_dis_k * cos(_vehicle_heading_theta * PI / 180);
    double _y_preview_cur = _vehicle_y + _d_dis_k * sin(_vehicle_heading_theta * PI / 180);
    geometry_msgs::Point pre_point;
    geometry_msgs::Pose current_pose_;
    int pre_number = 0;
    int near_number = 0;
    find_min_point(_x_preview_cur, _y_preview_cur, &pre_number);
    pre_point.x = _path_x[pre_number];
    pre_point.y = _path_y[pre_number];
    pre_point.z = 0;
    find_min_point(_vehicle_x, _vehicle_x, &near_number);
    current_pose_.position.x = _path_x[nextpoint_num];
    current_pose_.position.y = _path_y[nextpoint_num];
    current_pose_.position.z = 0;
    current_pose_.orientation = rpy_to_qua(phi_d, 0, 0);

    double kappa_pure;
    double denominator = pow((pre_point.x - Current_Point.x), 2) + pow((pre_point.y - Current_Point.y), 2);
    double numerator = 2 * calcRelativeCoordinate(pre_point, current_pose_).y;

    if (denominator != 0)
        kappa_pure = numerator / denominator;
    else
    {
        if (numerator > 0)
            // kappa_pure = kappa_min;
            kappa = kappa;
        else
            // kappa_pure = -kappa_min;
            kappa = -kappa;
    }
    cout << kappa << "kappa " << endl;
    //ph_dot=2*curvature*x_dot ??? why 2
    phi_d_dot = - 2 * Current_Point.v * kappa;
    phi_dot = angle_velocity_z;
    // cout << "phi_d_dot  " << phi_d_dot << "phi_dot  " << phi_dot << endl;

    //计算预瞄距离L
    if (kappa > 0.03)
    {
        LL = _t_preview * _vehicle_v + _d_0;
        _d_dis_min = 2.5;
    }
    else
    {
        LL = _t_preview_turn * _vehicle_v + _d_0;
        _d_dis_min = 5;
    }

    LL = limitParamChangeUnit(LL, _d_dis_max, _d_dis_min);

    //TODO:: 当前横向速度,期望的横向速度差值计算
    //transform oxfor linear speed to vehicle's body coordinate
    y_d_dot = 10 * kappa; //期望的横向速度为0; 改成真实的反馈速度??????????????????????
    y_dot = _vehicle_v * sin(u);

    // cout << " y_d_dot " << y_d_dot << " y_dot " << endl;

    static Matrix<float, 4, 4> A;
    static Matrix<float, 4, 1> B;
    static Matrix<float, 4, 1> X1;      //测量量
    static Matrix<float, 4, 1> X1_last; //测量量
    static Matrix<float, 4, 1> X1_dot;  //测量量
    static Matrix<float, 4, 1> X1_m;    //观测量
    static Matrix<float, 4, 1> X2;
    static Matrix<float, 4, 1> X2_last;
    static Matrix<float, 4, 1> X2_m; //
    static Matrix<float, 1, 4> C;
    static Matrix<float, 4, 1> X1_m_last;
    static Matrix<float, 4, 1> X2_m_last;
    static Matrix<float, 4, 4> I;
    static Matrix<float, 1, 4> D;
    
    D << 1, 0, 0, 0;
    cout << "D " << D << endl;
    I << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    A << 0, 1, 0, LL,
        0, a22, a23, a24,
        0, 0, 0, 1,
        0, a42, a43, a44;
    B << 0,
        b21,
        0,
        b41;
    C << 0, b1, 0, b2;
    e_phi_d = e_phi_d;
    float e_y_dot = _vehicle_v * (e_phi_d / 180 * PI - _vehicle_wheel_angle / 15.7 / 180 * PI);

    //角度单位均为弧度,速度输入为m/s
    X1 << e_y + LL * (e_phi_d / 180 * PI),
        e_y_dot,
        e_phi_d / 180 * PI,
        (phi_dot - phi_d_dot) / 180 * PI;
    cout << "e_y     " << e_y << endl;
    // cout << "e_y_dot     " << e_y_dot << endl;
    cout << "e_phi_d    " << e_phi_d << endl;
    // cout << "phi_dot - phi_d_dot    " << phi_dot - phi_d_dot << endl;
    // cout << "y_dot    " << y_dot << endl;
    // cout << "y_d_dot    " << y_d_dot << endl;
    // cout << "phi_d_dot     " << phi_d_dot << "    phi_dot    " << phi_dot << endl;
    X2 = (X1 - X1_last) / T - A * X1_last - B * u_last;
    // cout << "X2 " << endl
    //      << X2 << endl;
    // cout << " (X1-X1_last)/ T  " << endl
    //      << (X1 - X1_last) / T << endl;
    // cout << "2  " << endl
    //      << A << endl;
    // cout << "u_last  " << endl
    //      << u_last << endl;
    // cout << "X1_last  " << endl
    //      << X1_last << endl;
    // cout << "phi - phi_d  " <<e_phi_d<< endl;
    cout << "X1  !!!!!!!!!!!!!!!!!!" << endl
         << X1 << endl;

    X1_m = (A * X1 + B * u_last + X2_m_last - l1 * (X1_m_last - X1) - l2 * (X2_m_last - X2)) * T + X1_m_last;

    // cout << "X1_m  " << X1_m << endl;
    X2_m = (-l3 * (X1_m_last - X1) - l4 * (X2_m_last - X2)) * T + X2_m_last;
    // cout << "X2_m  " << endl
    //      << X2_m << endl;
    // cout << "X2 " << endl
    //      << X2 << endl;
    // cout << "X1_m_last " << endl
    //      << X1_m_last << endl;
    // cout << "X1_m_last - X1 " << endl
    //      << X1_m_last - X1 << endl;

    // u = (1 / (b1 * b1 + b2 * b2)) * C * (-KK * I * X1 -  X2_m);
    u = D * (-KK  * X1 -   X2_m);
    cout << " D *X1  D * X2_m " <<   -KK *X1<< "  " <<  X2_m<< endl;
    // cout << "u = (1 / (b1 * b1 + b2 * b2)) * C  " << (1 / (b1 * b1 + b2 * b2)) * C << endl;

    // b1 = b2 = 1;
    // u = C * (-KK * I * X1 - 0 * X2_m);
    // u =(e_y_dot + (phi_dot - phi_d_dot)/180*PI);
    // cout << "(1 / (b1 * b1 + b2 * b2)) * C   " << (1 / (b1 * b1 + b2 * b2)) * C << endl;
    X1_m_last = X1_m;
    X2_m_last = X2_m;
    X1_last = X1;
    X2_last = X2;
    u = limitParamChangeUnit(u, 0.56, -0.56);
    u_last = u;
    cout << " u " << u << endl;
    return u; //单位前轮转角弧度吗
}
double icvCarlaControl::ControlStr(double _vehicle_x, double _vehicle_y, double _vehicle_heading_theta, double _vehicle_v)
{
    //TODO:: follower model
    //input::current pose (m,m,deg), current speed (m/s)
    //output::target wheelsteer angle (deg)

    float _G; //the steering gear ratio (hand wheel angle/road wheel angle)
    float _t; //time
    float _theta_ss;
    float _d_dis;   //preview distance
    float _d_dis_k; //preview distance

    float _x_preview; //preview point x coordinate
    float _y_preview; //preview point y coordinate
    float _Oss;       //distance error
    float _Anglss;    //angle error
    float _transfer_val;
    float _wheel_angle_degree;
    _vehicle_x = _vehicle_x + _vehicle_v * __distance_pre * cos(_vehicle_heading_theta * PI / 180);
    _vehicle_y = _vehicle_y + _vehicle_v * __distance_pre * sin(_vehicle_heading_theta * PI / 180);
    //when enable follow
    if (_keep_steer == 0 && _path_x.size() > 0)
    {
        //TODO::preview for calclulate kappa to get turn radius
        _d_dis_k = _t_preview * _vehicle_v + _d_0;
        _d_dis_k = limitParamChangeUnit(_d_dis_k, _dis_pre_max, _d_dis_min_k);
        double _x_preview_cur = _vehicle_x + _d_dis_k * cos(_vehicle_heading_theta * PI / 180);
        double _y_preview_cur = _vehicle_y + _d_dis_k * sin(_vehicle_heading_theta * PI / 180);
        // //TODO:choose three points to calculate curvature which is used to choose preview distance
        int three_number = 0;
        find_min_point(_vehicle_x, _vehicle_y, &three_number);
        geometry_msgs::Point p1, p2, p3;
        if (nextpoint_num < (_path_x.size() - 20))
        {
            p1.x = _path_x[nextpoint_num];
            p2.x = _path_x[nextpoint_num + 10];
            p3.x = _path_x[nextpoint_num + 20];
            p1.y = _path_y[nextpoint_num];
            p2.y = _path_y[nextpoint_num + 10];
            p3.y = _path_y[nextpoint_num + 20];
        }
        else
        {
            ROS_WARN("three point kappa is the lastpoint!!!");
        }
        double kappa = CalculateCur(p1, p2, p3) + 0.000001;
        double kappa_three = kappa;
        if (kappa_three > 0.03)
        {
            _d_dis = _t_preview * _vehicle_v + _d_0;
            _d_dis_min = 2.5;
            feedforward_K = 1;
        }
        else
        {
            _d_dis = _t_preview_turn * _vehicle_v + _d_0;
            _d_dis_min = 5;
            feedforward_K = 1;
            feedforward_P = 1;
        }
        _d_dis = limitParamChangeUnit(_d_dis, _dis_pre_max, _d_dis_min);
        _x_preview = _vehicle_x + _d_dis * cos(_vehicle_heading_theta * PI / 180);
        _y_preview = _vehicle_y + _d_dis * sin(_vehicle_heading_theta * PI / 180);
        //RVIZ display
        _x_preview_rviz = _x_preview;
        _y_preview_rviz = _y_preview;
        _Oss = find_min_point(_x_preview, _y_preview, &nextpoint_num);
        find_min_point(_vehicle_x, _vehicle_y, &nextpoint_num);

        // TODO：Add Angle compensation
        // double _angle = atan2(_path_y[nextpoint_num]-_vehicle_y,_path_x[nextpoint_num]-_vehicle_x);
        // double ys = fmod(((_angle-Current_Point.theta)/180.0*PI+PI),(2*PI));
        // if (ys < 0)
        // {
        //     ys = ys + 2 * PI;
        // }
        // _Anglss = 0 - (ys - PI) / PI * 180.0;
        // cout<<" _Anglss  "  <<_Anglss<<endl;

        //TODO:Judge the left and right directions of curvature. In Carla, left hand coordinate system, real world right hand coordinate system
        //kappa puresuit method (l*l)/(2*x)
        geometry_msgs::Point pre_point;
        geometry_msgs::Pose current_pose_;
        int pre_number = 0;
        int near_number = 0;
        find_min_point(_x_preview_cur, _y_preview_cur, &pre_number);
        pre_point.x = _path_x[pre_number];
        pre_point.y = _path_y[pre_number];
        pre_point.z = 0;
        find_min_point(_vehicle_x, _vehicle_x, &near_number);

        current_pose_.position.x = Current_Point.x;
        current_pose_.position.y = Current_Point.y;
        current_pose_.position.z = 0;
        current_pose_.orientation = rpy_to_qua(Current_Point.theta, 0, 0);

        double kappa_pure;
        double denominator = pow((pre_point.x - Current_Point.x), 2) + pow((pre_point.y - Current_Point.y), 2);
        double numerator = 2 * calcRelativeCoordinate(pre_point, current_pose_).y;


        if (denominator != 0)
            kappa_pure = numerator / denominator;
        else
        {
            if (numerator > 0)
                kappa_pure = kappa_min;
            else
                kappa_pure = -kappa_min;
        }
        kappa = kappa_pure;

        _transfer_val = cal_transfer_val(_d_dis);
        _theta_ss = _transfer_val * _Oss;

        selectCoefficient(Current_Point.v, kappa_three, &_G);

        _wheel_angle_degree = pre_K * (0.7 * _theta_ss * _G * 180 / PI + 0.5 * feedforward_K * atan(L * kappa) * 180 / PI * K * _kappa);
        if (_flag < 30)
            _wheel_angle_degree = limit_wheel_val(_vehicle_wheel_angle, _wheel_angle_degree);
        else
            _wheel_angle_degree = limit_wheel_val(_wheel_angle_degree_last, _wheel_angle_degree);
        _wheel_angle_degree_last = _wheel_angle_degree;
        _cal_wheel_angle = _wheel_angle_degree;
        _record_cnt = 0;
    }
    else
    {
        if ((_keep_steer == 1) && (_record_cnt < 2))
        {
            _keep_wheel_angle = _vehicle_wheel_angle;
            _record_cnt++;
        }
        _wheel_angle_degree = limit_wheel_val(_vehicle_wheel_angle, _keep_wheel_angle);
        _cal_wheel_angle = _wheel_angle_degree;
    }
    _flag = _flag + 1;

    return _wheel_angle_degree;
}

void icvCarlaControl::selectCoefficient(double v_s, double kappa_s, float *G)
{
    //TODO::choose  preview param

    v_s = v_s * 3.6;

    if (v_s >= 0 && v_s <= 3)
    {
        *G = _p1_post;
    }
    if (v_s > 3 && v_s <= 5)
    {
        *G = _p2_post;
    }
    if (v_s > 5 && v_s <= 7)
    {
        *G = _p3_post;
    }
    if (v_s > 7 && v_s <= 10)
    {
        *G = _p4_post;
    }
    if (v_s > 10 && v_s <= 15)
    {
        *G = _p5_post;
    }
    if (v_s > 15 && v_s <= 20)
    {
        *G = _p6_post;
    }
    if (v_s > 20 && v_s <= 25)
    {
        *G = _p7_post;
    }
    if (v_s > 25 && v_s <= 30)
    {
        *G = _p8_post;
    }

    //Curvature feedforward parameter selection
    if (v_s < 0.01)
    {
        pre_K = 1;
        // feedforward_K = 1;
    }
    else if (kappa_s < 30)
    {
        pre_K = 1;
    }
    else
    {
        pre_K = 1;
    }
}

void icvCarlaControl::calcYaw(vector<double> pathx, vector<double> pathy, double *yawangle)
{
    //path headingangle estimated
    //input:: path
    //output:: Yaw(deg)

    double yaw = 0;
    double yaw_sum = 0;
    for (int i = 0; i < pathx.size() - 1; i++)
    {
        yaw = atan2((pathy[i + 1] - pathy[i]), (pathx[i + 1] - pathx[i]));
        yaw_sum += yaw;
    }
    *yawangle = yaw_sum / (pathx.size() - 1);
}


geometry_msgs::Point icvCarlaControl::calcRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose)
{
    //TODO:: calculation relative coordinate of point from current_pose frame
    //input:: taget preview point,current pose of vehicle
    //output:: transform coordinate

    tf::Transform inverse;
    tf::poseMsgToTF(current_pose, inverse);
    tf::Transform transform = inverse.inverse();

    tf::Point p;
    pointMsgToTF(point_msg, p);
    tf::Point tf_p = transform * p;
    geometry_msgs::Point tf_point_msg;
    pointTFToMsg(tf_p, tf_point_msg);
    return tf_point_msg;
}

float icvCarlaControl::find_min_point(float _preview_x_rec, float _preview_y_rec, int *count_min)
{
    //TODO::find closest point error (left - right)
    //input:: coordinate (m,m)
    //outpu:: the index of closest point

    float _preview_x_tmp;
    float _preview_y_tmp;
    float _L_min = 20.0;
    float _L;
    float _x_min = -1.0;
    float _y_min = -1.0;
    float _Oss_tmp = 0;
    int _count_min = 0;
    _preview_x_tmp = _preview_x_rec;
    _preview_y_tmp = _preview_y_rec;
    float _vehicle_x_ref = Current_Point.x;
    float _vehicle_y_ref = Current_Point.y;
    for (int _path_ii = 0; _path_ii < _path_x.size(); _path_ii++)
    {

        _L = sqrt(pow((_preview_x_tmp - _path_x[_path_ii]), 2) + pow((_preview_y_tmp - _path_y[_path_ii]), 2));
        if (_L <= _L_min)
        {
            _L_min = _L;
            _x_min = _path_x[_path_ii];
            _y_min = _path_y[_path_ii];
            _count_min = _path_ii;
        }
        else
        {
            _L_min = _L_min;
            _x_min = _x_min;
            _y_min = _y_min;
            _count_min = _count_min;
        }
    }
    //TODO:add closest point index
    *count_min = _count_min;
    if ((_preview_x_tmp - _vehicle_x_ref) * (_y_min - _vehicle_y_ref) - (_preview_y_tmp - _vehicle_y_ref) * (_x_min - _vehicle_x_ref) >= 0)
        _Oss_tmp = _L_min;
    else
        _Oss_tmp = -1.0 * _L_min;

    return _Oss_tmp;
}

float icvCarlaControl::cal_transfer_val(float _d_dis_rec)
{
    //TODO::calculate cofficient of lat_error
    //input::preview distance
    //output::cofficient

    float _m = m;    //the mass of vehicle
    float _a = lf;   //the distance from centre of mass to front axle
    float _b = lr;   //the distance from centre of mass to rear axle
    float _Cf = Ccf; //the front tyre cornering stiffness
    float _Cr = Ccr; //the rear tyre cornering stiffness
    float _denominator;
    float _molecular;
    float _T;
    float _d_dis_tmp;
    float _trans_val;
    float _vehicle_v = Current_Point.v;
    _d_dis_tmp = _d_dis_rec;
    _molecular = 2 * (_a + _b - (_m * (_a * _Cf - _b * _Cr)) / ((_a + _b) * _Cf * _Cr));
    _T = _b - ((_a * _m * (_vehicle_v * _vehicle_v)) / (_Cr * (_a + _b)));
    _denominator = _d_dis_tmp * (_d_dis_tmp + 2 * _T);
    _trans_val = _molecular / _denominator;
    return _trans_val;
}

float icvCarlaControl::limit_wheel_val(float _wheel_last_ref_rec, float _wheel_angle_degree_rec)
{
    //TODO::limit wheelsteer's rate of change
    //input::last wheelangle(deg),current wheelangle(deg)
    //output:: wheelangle(deg) after limit

    float _wheel_now;
    float _wheel_last;

    _wheel_now = _wheel_angle_degree_rec;
    _wheel_last = _wheel_last_ref_rec;
    if (_wheel_now >= _wheel_last)
    {
        if ((_wheel_now - _wheel_last) > _wheel_angle_degree_sec * _delta_T)
        {
            _wheel_now = _wheel_last + _wheel_angle_degree_sec * _delta_T;
        }
        else
        {
            _wheel_now = _wheel_now;
        }
        if (_wheel_now >= _wheel_angle_degree_max)
        {
            _wheel_now = _wheel_angle_degree_max;
        }
        else
        {
            _wheel_now = _wheel_now;
        }
    }
    else
    {
        if ((_wheel_last - _wheel_now) >= _wheel_angle_degree_sec * _delta_T)
        {
            _wheel_now = _wheel_last - _wheel_angle_degree_sec * _delta_T;
        }
        else
        {
            _wheel_now = _wheel_now;
        }
        if (_wheel_now <= -_wheel_angle_degree_max)
        {
            _wheel_now = -_wheel_angle_degree_max;
        }
        else
        {
            _wheel_now = _wheel_now;
        }
    }
    return _wheel_now;
}

bool icvCarlaControl::StateConstraint(float v, float angle, float t)
{
    //TODO::calculate  vehicle constraint
    //input:: speed (km/h),front tire angle(rad),period time (s)
    //output:: the safe flag true or false

    bool Path_flag = false;
    Beta = ((lr - (lf * m * v * v) / (2 * Ccf * (lr + lf))) / (lf + lr + (m * v * v * (lr * Ccr - lf * Ccf) / (2 * Ccf * Ccr * (lr + lf))))) * angle;
    // v = 60; / km/h
    angle = 0.1;
    float beita = Beta;
    float ceita = angle;
    // v = 2;
    // F_yf=u3;
    // F_yr=u4;
    //模型参数初始值：
    float A0 = 1.65;
    float A1 = -34;
    float A2 = 1250, A3 = 3036, A4 = 12.8, A5 = 0.00501, A6 = -0.02103;
    float A7 = 0.77394, A8 = 0.002289, A9 = 0.013442, A10 = 0.003709, A11 = 19.1656, A12 = 1.21356;
    float A13 = 6.26206;
    //车辆初始参数：
    //m=1818.2;lf=1.463;lr=1.585;
    float Q = 0.3; //车轮外倾角；
    //计算两个轮子的侧偏角和垂向压力值：
    //注意魔术公式轮胎模型的车轮垂直载荷为KN，输出的侧向力为N；
    float af = (beita + lf * r / v - ceita) * 180 / PI;
    float ar = (beita - lr * r / v) * 180 / PI;
    float F_zf = m * 9.8 * (lr) / (lr + lf) / 2000;
    float F_zr = m * 9.8 * (lf) / (lf + lr) / 2000;
    //建立轮胎模型：
    float C = A0;
    float D1 = A1 * (F_zf * F_zf) + A2 * F_zf;
    float B1 = A3 * sin(2 * atan(F_zf / A4)) * (1 - A5 * Q) / (C * D1);
    float E1 = A6 * F_zf + A7;
    float Sh1 = A9 * F_zf + A10 + A8 * Q;
    float Sv1 = A11 * F_zf * Q + A12 * F_zf + A13;
    float X1 = (af + Sh1);
    float F_yf = (D1 * sin(C * atan(B1 * X1 - E1 * (B1 * X1 - atan(B1 * X1))))) + Sv1;

    float D2 = A1 * (F_zr * F_zr) + A2 * F_zr;
    float B2 = A3 * sin(2 * atan(F_zr / A4)) * (1 - A5 * Q) / (C * D2);
    float E2 = A6 * F_zr + A7;
    float Sh2 = A9 * F_zr + A10 + A8 * Q;
    float Sv2 = A11 * F_zr * Q + A12 * F_zr + A13;
    float X2 = (ar + Sh2);
    float F_yr = (D2 * sin(C * atan(B2 * X2 - E2 * (B2 * X2 - atan(B2 * X2))))) + Sv2;

    float beita_d = -r + 2 * F_yf / (m * v) * cos(beita - ceita) + 2 * F_yr / (m * v) * cos(beita);
    float r_d = 2 * lf / I * F_yf * cos(ceita) - 2 * lr / I * F_yr;

    // Beta=Beta+beita_d*t;
    r = t * r_d;
    beita = t * beita_d;
    float EE1 = -1 / 9, EE2 = 0;
    EE2 = abs(beita_d * EE1 + Beta);
    if (abs(Beta) > 15)
        Path_flag = false;
    else
        Path_flag = true;

    return Path_flag;
}


float icvCarlaControl::tiremodel(float beita, float cetia, float v, float r)
{
    //TODO::calculate force of Fz and Fr
    //input :: the center of mass angle ,wheelangle ,speed,Yawrate first derivative
    //output:: FZ and Fr

    //模型参数初始值：
    float A0 = 1.65;
    float A1 = -34;
    float A2 = 1250, A3 = 3036, A4 = 12.8, A5 = 0.00501, A6 = -0.02103;
    float A7 = 0.77394, A8 = 0.002289, A9 = 0.013442, A10 = 0.003709, A11 = 19.1656, A12 = 1.21356;
    float A13 = 6.26206;
    //车辆初始参数：
    //m=1818.2;lf=1.463;lr=1.585;
    float Q = 0.3; //车轮外倾角；
    //计算两个轮子的侧偏角和垂向压力值：
    //注意魔术公式轮胎模型的车轮垂直载荷为KN，输出的侧向力为N；
    float af = (beita + lf * r / v - cetia) * 180 / PI;
    float ar = (beita - lr * r / v) * 180 / PI;
    float F_zf = m * 9.8 * (lr) / (lr + lf) / 2000;
    float F_zr = m * 9.8 * (lf) / (lf + lr) / 2000;
    //建立轮胎模型：
    float C = A0;
    float D1 = A1 * (F_zf * F_zf) + A2 * F_zf;
    float B1 = A3 * sin(2 * atan(F_zf / A4)) * (1 - A5 * Q) / (C * D1);
    float E1 = A6 * F_zf + A7;
    float Sh1 = A9 * F_zf + A10 + A8 * Q;
    float Sv1 = A11 * F_zf * Q + A12 * F_zf + A13;
    float X1 = (af + Sh1);
    float F_yf = (D1 * sin(C * atan(B1 * X1 - E1 * (B1 * X1 - atan(B1 * X1))))) + Sv1;

    float D2 = A1 * (F_zr * F_zr) + A2 * F_zr;
    float B2 = A3 * sin(2 * atan(F_zr / A4)) * (1 - A5 * Q) / (C * D2);
    float E2 = A6 * F_zr + A7;
    float Sh2 = A9 * F_zr + A10 + A8 * Q;
    float Sv2 = A11 * F_zr * Q + A12 * F_zr + A13;
    float X2 = (ar + Sh2);
}

/******数据写出*******/
void icvCarlaControl::data2file()
{

    ofstream out;
    out.open("/home/zx/follow_carla/testdata0531.txt", std::ios::out | std::ios::app);
    pathdeal();

    out << setiosflags(ios::fixed) << setprecision(3) << ctedata[0] <<" "<<Current_Point.x<<" "<<Current_Point.y<< endl;
    // out << setiosflags(ios::fixed) << setprecision(3) << ctedata[0] << " " << ctedata[1] << " " << ctedata[2] << " " << ctedata[3] << " " << ctedata[4] << " " << ctedata[5] << " " << ctedata[6] << " " << ctedata[7] << endl;
    // out << setiosflags(ios::fixed) << setprecision(3) << target_speed << " " << Current_Point.v <<" "<<target_speed_limit<< endl;//test for speed
    out.close();
}

// /*函数输入：当前车辆坐标，前轮偏角，车辆速度v，航向角phi，迭代周期T
// 函数输出：坐标（x,y,theta)*/
void icvCarlaControl::Vehiclemodeupdate1(Pathpoint coordinate, float delta_f, float T, float *x_vehicle, float *y_vehicle, float *heading) //不准确
{
    //TODO::vehicle dynamic model.
    //input::coordinate(x,y),fron tire angle, speed(m/s),headingangle, period time;
    //output::coordinate after update

    Pathpoint coordinate_back;
    X = coordinate.x;
    Y = coordinate.y;
    phi = coordinate.theta / 180 * PI;
    x_dot = coordinate.v * cos(delta_f) + 0.00001;
    y_dot = coordinate.v * sin(delta_f);
    cout << "1x_dot  " << x_dot << " y_dot " << y_dot << " phi_dot " << phi_dot << endl;

    phi_dot = coordinate.phi_dot;
    y_dot = y_dot + T * (-x_dot * phi_dot + 2 * (Ccf * (delta_f - (y_dot + lf * phi_dot) / x_dot) + Ccr * (lr * phi_dot - y_dot) / x_dot) / m); //y_dot 初值为0？？
    x_dot = x_dot + T * (y_dot * phi_dot + 2 * (Clf * Sf + Clr * Sr + Ccf * delta_f * (delta_f - (y_dot + phi_dot * lf) / x_dot)) / m);         //x_dot初值为0？？

    phi_dot = phi_dot + T * ((2 * lf * Ccf * (delta_f - (y_dot + lf * phi_dot) / x_dot) - 2 * lr * Ccr * (lr * phi_dot - y_dot) / x_dot) / I); //phi_dot 初值为0
    phi = phi + T * phi_dot;
    // Y=Y+T*(x_dot*sin(phi)+y_dot*cos(phi));
    // X=X+T*(x_dot*cos(phi)-y_dot*sin(phi));
    coordinate.theta = coordinate.theta + T * phi_dot;
    coordinate.y = coordinate.y + T * (x_dot * sin(phi) + y_dot * cos(phi));
    coordinate.x = coordinate.x + T * (x_dot * cos(phi) - y_dot * sin(phi));

    coordinate.x_dot = x_dot;
    coordinate.y_dot = y_dot;

    if (coordinate.theta > 180)
    {
        coordinate.theta = coordinate.theta - 360;
    }
    *x_vehicle = coordinate.x;
    *y_vehicle = coordinate.y;
}

void icvCarlaControl::Vehiclemodeupdate2(Pathpoint coordinate, float delta_f, float T, float *x_vehicle, float *y_vehicle, float *heading)
{
    // Pathpoint coordinate;
    //车辆运动学模型 后轴为中心点
    //函数输入前轮转角，车辆速度v，航向角phi
    //函数输出：x，y，headingangle

    *x_vehicle = *x_vehicle + (coordinate.v) * cos((*heading) / 180 * PI) * T;
    *y_vehicle = *y_vehicle + (coordinate.v) * sin((*heading) / 180 * PI) * T;
    *heading = *heading / 180 * PI + ((coordinate.v) / (lf + lr)) * tan(delta_f) * T;
    *heading = *heading * 180 / PI;
    if (*heading > 360)
    {
        *heading = *heading - 360;
    }

    // return coordinate;
}

void icvCarlaControl::pathdeal()
{
    /*******calculate follow error  just ********/
    //"pathdeal to calculte cte_d  cte_a "

    float cte_D = 0;
    float cte_A;
    int plo_i = 0; //曲线拟合的索引点
    float ctd = 100;
    float dis_a = 0;
    int m = _path_x.size(); //轨迹点长度
    float x_len = 0;
    float x_plo_start = 0;
    float d_ditance = 0; //x的离散补偿
    float sig_d = 0;
    int sgn = 0;
    /*曲线拟合*/
    VectorXd x_veh(5);
    VectorXd y_veh(5);
    for (int i = 0; i < m; i++)
    {
        dis_a = sqrt(pow((Current_Point.x - Waypoints[i].x), 2) + (pow((Current_Point.y - Waypoints[i].y), 2)));
        if (dis_a >= ctd)
        {
            ctd = ctd;
            plo_i = plo_i;
        }
        else
        {
            ctd = dis_a;
            plo_i = i;
        }
    }
    if (plo_i < 4)
    {

        float plox[5] = {Waypoints[0].x, Waypoints[1].x, Waypoints[2].x, Waypoints[3].x, Waypoints[4].x};
        float ploy[5] = {Waypoints[0].y, Waypoints[1].y, Waypoints[2].y, Waypoints[3].y, Waypoints[4].y};
        for (int j = 0; j < 5; j++)
        {
            x_veh[j] = plox[j];
            y_veh[j] = ploy[j];
        }
        // f1 = np.polyfit(plox, ploy, 3)
        // p1 = np.poly1d(f1)
        x_len = Waypoints[3].x - Waypoints[0].x;
        x_plo_start = Waypoints[0].x;
    }
    else
    {
        float plox[5] = {Waypoints[plo_i - 2].x, Waypoints[plo_i - 1].x, Waypoints[plo_i].x, Waypoints[plo_i + 1].x, Waypoints[plo_i + 2].x};
        float ploy[5] = {Waypoints[plo_i - 2].y, Waypoints[plo_i - 1].y, Waypoints[plo_i].y, Waypoints[plo_i + 1].y, Waypoints[plo_i + 2].y};
        for (int j = 0; j < 5; j++)
        {
            x_veh[j] = plox[j];
            y_veh[j] = ploy[j];
        }
        // f1 = np.polyfit(plox, ploy, 3)
        // p1 = np.poly1d(f1)
        x_len = Waypoints[plo_i + 1].x - Waypoints[plo_i - 1].y;
        x_plo_start = Waypoints[plo_i - 1].x;
    }
    auto coeffs = polyfit(x_veh, y_veh, 3); //

    for (int i = 0; i < coeffs.size(); i++)
    {
        // ROS_INFO("coeffs[%d]:%f",i,coeffs[i]);
    }
    if (x_len > 0)
        d_ditance = 0.1;
    else
        d_ditance = -0.1;
    float i_plo = 0;
    float x_plo = 0;
    float y_plo = 0;
    float d_plo = 0;
    while (abs(i_plo) < abs(x_len))
    {
        x_plo = x_plo_start + i_plo;
        y_plo = polyeval(coeffs, x_plo);
        d_plo = sqrt(pow((Current_Point.x - x_plo), 2) + (pow((Current_Point.y - y_plo), 2)));
        if (d_plo >= ctd)
            ctd = ctd;
        else
            ctd = d_plo;
        i_plo = i_plo + d_ditance;
    }
    sig_d = ((Waypoints[plo_i].x - Current_Point.x) * (Waypoints[plo_i + 1].y - Current_Point.y)) - ((Waypoints[plo_i].y - Current_Point.y) * (Waypoints[plo_i + 1].x - Current_Point.x));
    if (sig_d >= 0)
    {
        if (sig_d > 0)
            sgn = 1;
        else
            sgn = 0;
    }
    else
    {
        sgn = -1;
    }
    cte_D = ctd * sgn;
    cte_A = 0 - (fmod(((Waypoints[plo_i].theta - Current_Point.theta) / 180.0 * PI + PI), (2 * PI)) - PI) / PI * 180.0;
}

double icvCarlaControl::polyeval(Eigen::VectorXd coeffs, double x)
{
    //TODO::获取拟合的参数曲线在x时的y值

    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

Eigen::VectorXd icvCarlaControl::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
    //多项式拟合的一个函数,返回拟合的参数曲线系数
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
            A(j, i + 1) = A(j, i) * xvals(j);
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

double icvCarlaControl::CalculateCur(geometry_msgs::Point P1, geometry_msgs::Point P2, geometry_msgs::Point P3)
{
    //TODO::calculate cur
    //input::three points
    //output:: cur
    double curvity_pre = 0; //last time cur
    double curvity = 0;     //current cur
    double curvity_R = 0;   //current radius
    double speed_pre = 0;   //last speed
    double speed_cur = 0;   //current speed
    double acc = 0;
    double acc_pre = 0; //last time acc
    //judge three point not in same line
    if (P1.x == P2.x == P3.x || P1.y == P2.y == P3.y)
    {
        curvity = 0;
    }
    else
    {
        if (P1.x == P2.x && P1.y == P2.y || P1.x == P3.x && P1.y == P3.y || P2.x == P3.x && P2.y == P3.y)
        {
            // ROS_ERROR(" Path have same points!!!!  ");
        }
        else
        {
            double dis1, dis2, dis3;
            double cosA, sinA, dis;
            dis1 = sqrt((P1.x - P2.x) * (P1.x - P2.x) + (P1.y - P2.y) * (P1.y - P2.y));
            dis2 = sqrt((P1.x - P3.x) * (P1.x - P3.x) + (P1.y - P3.y) * (P1.y - P3.y));
            dis3 = sqrt((P2.x - P3.x) * (P2.x - P3.x) + (P2.y - P3.y) * (P2.y - P3.y));
            dis = dis1 * dis1 + dis3 * dis3 - dis2 * dis2;
            cosA = dis / (2 * dis1 * dis3) + 0.0000000000001;
            if (cosA > 1)
            {
                cosA = 1;
            }
            sinA = sqrt(1 - cosA * cosA);
            curvity_R = 0.5 * dis2 / sinA;
            curvity = 1 / curvity_R;
        }
    }
    return curvity;
}
//平滑度判断标准1.方向盘：最小半径,曲率变化率
//速度，点点之间的加速度是否超过最大值，加速度变化率。
bool icvCarlaControl::PathSmoothCheck()
{
    //TODO::check Path continuity
    //input::
    //output::Path status true or false
    geometry_msgs::Point P1;
    geometry_msgs::Point P2;
    geometry_msgs::Point P3;
    double curvity_pre = 0; //last time curvity
    double curvity = 0;     //current curvity
    double curvity_R = 0;   //current turn radius
    double speed_pre = 0;   //last time speed
    double speed_cur = 0;   //current speed
    double acc = 0;
    double acc_pre = 0; //last time acc

    for (int i = 0; i < (Waypoints_size - 2); i++)
    {

        try
        {
            P1.x = _path_x[i];
            P2.x = _path_x[i + 1];
            P3.x = _path_x[i + 2];
            P1.y = _path_y[i];
            P2.y = _path_y[i + 1];
            P3.y = _path_y[i + 2];
            curvity = CalculateCur(P1, P2, P3);
            curvity_R = 1 / curvity;
            //TODO::judge curvity_R is  good
            if (curvity_R < R_min)
            {
                ROS_DEBUG("error:Path Curvity is too big!!!");
                ROS_DEBUG("curvity_R=%lf", curvity_R);
                return false;
            }
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            cout << "三点取点失败\n";
        }
        double cur_dt = curvity - curvity_pre;
        if (cur_dt * L * K > 30)
        {
            ROS_DEBUG("error:Path Curvity change too much!!!");
            ROS_DEBUG("cur_dt*L*K =%lf", cur_dt * L * K);
            return false;
        }
        curvity_pre = curvity;
        //TODO::calculate speed continuity
        speed_cur = Waypoints[i].v;
        acc = (speed_cur - speed_pre) / dt;
        if (abs(acc) > Acc_max)
        {
            ROS_DEBUG("error:Acc is too MAX!!!!");
            return false;
        }
        if (abs(acc - acc_pre) > jerk_max)
            speed_pre = speed_cur;
    }
    return true;
}
//小鹏速度y预测模块，输入当前车速为m/s,预瞄距离   预瞄速度,返回目标车速m/s
double icvCarlaControl::Velocitytopredict(float velocity, float distance_pre, float velocity_pre)
{
    //TODO:: speed prediction module
    //input:: speed (m/s) preview distance(m) preview speed(m/s)
    //output::target speed (m/s)

    float V[20] = {0}; //存储速度
    float S[20] = {0}; //累积位移
    V[0] = velocity;
    float Acc_G1, out_speed;
    int Np = 20;      //预测周期20
    float T_G1 = 0.1; //步长0.1s
    int i_pre = 0;
    //TODO::calculate max speed limit,to limit the maxspeed
    for (int i = 0; i < (Np - 1); i++)
    {
        if (V[i] < 30)
        {
            Acc_G1 = 0.0006 * pow(V[i], 3) - 0.0329 * V[i] * V[i] + 0.4905 * V[i] + 0.8657; //G1加速度变化拟合方程
            V[i + 1] = V[i] + Acc_G1 * T_G1;
            S[i + 1] = S[i] + V[i + 1] * T_G1 - 0.5 * Acc_G1 * T_G1 * T_G1;
        }
        else
        {
            V[i + 1] = V[i];
            S[i + 1] = S[i] + V[i + 1] * T_G1;
        }
        // cout << "!!!!!!!!  == " << Acc_G1 << "  " << V[i] << "  " << S[i] << endl;
    }
    for (int i = 0; i < Np; i++)
    {
        if (distance_pre < S[i]) //preview distance
        {
            i_pre = i;
            break;
        }
        if (i == (Np - 1))
            i_pre = i;
    }
    if (velocity_pre < V[i_pre]) //acc
    {
        out_speed = velocity_pre;
    }
    else
    {
        ROS_DEBUG("target speed is too max!!! ");
        out_speed = V[i_pre];
    }
    return out_speed;
}
void icvCarlaControl::run_speedcontrol(double *out_throttle, double *out_brake, double tar_v)
{
    //TODO::speed control,use in carla
    //input::target speed(km/h)
    //output:: throttle,out_brake.
    target_speed = tar_v / 3.6; //get target speed
    tar_v = tar_v / 3.6;
    float tar_v1 = Velocitytopredict(Current_Point.v, 5, tar_v) * 3.6;
    target_speed_limit = tar_v1 / 3.6;
    double v_error;
    if (!follower_flag)
        tar_v1 = 0;
    v_error = tar_v1 - Current_Point.v * 3.6;
    if (v_error < -2)
    {
        *out_throttle = 0;
        v_error = limitParamChangeUnit(v_error, 0, -4);
        *out_brake = abs(PIDcontrolbrake(v_error, _v_pid));
        *out_brake = limitParamChangeUnit(*out_brake, 0.4, 0);
    }
    else
    {
        v_error = limitParamChangeUnit(v_error, 8, 0);
        *out_throttle = PIDcontrolacc(v_error, _v_pid);
        if (!setVehiclestauts_flag)
        {
            *out_throttle = 0;
        }
        *out_throttle = limitParamChangeUnit(*out_throttle, 0.8, 0);
        *out_brake = 0;
    }
}
double icvCarlaControl::limitParamChangeUnit(double param, double max_param, double min_param)
{
    //TODO::limit parameter boundary
    //input::limit param,max and min param
    //output:: param after limit
    if (param > max_param)
    {
        param = max_param;
    }
    else if (param < min_param)
    {
        param = min_param;
    }
    else
    {
        param = param;
    }

    return param;
}
double icvCarlaControl::PIDcontrolacc(double error, PID pid)
{
    //TODO::PID control acc
    //input:: error and pid param(kp,ki,kd)
    //output:: insturct throttle
    int k = 1;
    error_p = error;
    if (abs(error_p) < 3)
    {
        error_i += error_p;
    }
    else
    {
        error_i = 0;
    }
    error_d = error_p - error_dp;
    error_dp = error_p;
    out_insturct = pid.Kp * error_p + pid.Ki * error_i + pid.Kd * error_d;
    return out_insturct;
}
double icvCarlaControl::PIDcontrolbrake(double error, PID pid)
{
    //TODO::PID control acc
    //input:: error and pid param(kp,ki,kd)
    //output:: insturct brake
    error_p = error;
    if (abs(error_p) < 2)
    {
        error_i += error_p;
    }
    else
    {
        error_i = 0;
    }
    error_d = error_p - error_dp;
    error_dp = error_p;
    out_insturct = pid.Kp * error_p + pid.Ki * error_i + pid.Kd * error_d;
    return out_insturct;
}
/*四元素转航向角*/
double icvCarlaControl::qua_to_rpy(geometry_msgs::Quaternion posedata)
{
    //TODO:: oritation transform to RPY
    //input::oritation
    //output::Yaw angle(deg)
    float w = posedata.w;
    float x = posedata.x;
    float y = posedata.y;
    float z = posedata.z;

    float R = atan2((2 * (w * x + y * z)), (1 - 2 * (x * x + y * y)));
    float P = asin(2 * (w * y - z * x));
    float Y = atan2((2 * (w * z + x * y)), (1 - 2 * (z * z + y * y)));
    Y = Y * 180 / 3.141592654;

    return Y;
}
//RPY转四元素//
geometry_msgs::Quaternion icvCarlaControl::rpy_to_qua(double Yaw, double Pitch, double Roll)
{
    //TODO:: RPY transform to oritation
    //input::RPY(deg)
    //output::oritation

    geometry_msgs::Quaternion qua;
    Yaw = Yaw * PI / 180;
    Pitch = 0 * PI / 180;
    Roll = 0 * PI / 180;

    double cy = cos(Yaw * 0.5);
    double sy = sin(Yaw * 0.5);
    double cp = cos(Pitch * 0.5);
    double sp = sin(Pitch * 0.5);
    double cr = cos(Roll * 0.5);
    double sr = sin(Roll * 0.5);

    qua.w = cy * cp * cr + sy * sp * sr;
    qua.x = cy * cp * sr - sy * sp * cr;
    qua.y = sy * cp * sr + cy * sp * cr;
    qua.z = sy * cp * cr - cy * sp * sr;

    return qua;
}

} // namespace icv
