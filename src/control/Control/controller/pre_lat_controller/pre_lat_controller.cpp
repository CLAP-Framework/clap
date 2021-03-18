/*
 * @Descripttion:
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-27 19:07:33
 * @LastEditors: xiangzhang
 * @LastEditTime: 2020-10-26 14:50:37
 */

#include "pre_lat_controller.h"

using namespace std;
using namespace Eigen;

namespace Control
{
  Pre_Lat_Controller::Pre_Lat_Controller()

  {
    /**init param of control and use yaml  config*/
    controlInit();
  }

  Pre_Lat_Controller::~Pre_Lat_Controller() {}

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
  void Pre_Lat_Controller::controlInit()
  {
    //加载参数文件
    char config_path[MAX_PATH] = {0};

    strcpy(config_path, getenv("ZZZ_ROOT"));

    strcat(config_path,
           "/zzz/src/control/Control/conf/pre_lat_controller.yaml");

    // YAML::Node dset_config = YAML::LoadFile("/home/fangniuwa/g3bug_fix/control_fix/src/Control/conf/pre_lat_controller.yaml");
    // src/driver/icvfollower/config/Controlconfig.yaml
    // YAML::Node dset_config = YAML::LoadFile(
    //     "/home/zx/2.control_gitlab/src/Control/conf/pre_lat_controller.yaml");
    YAML::Node dset_config = YAML::LoadFile(config_path);
    // Vehiclemodel,Controlparam two key_value
    YAML::Node Vehiclemodel = dset_config["Vehiclemodel"];
    YAML::Node Controlparam = dset_config["Controlparam"];

    /****VehicleModle*****/
    lf = getParam<double>(Vehiclemodel, "_lf", 0);
    lr = getParam<double>(Vehiclemodel, "_lr", 0);
    L = lr + lf;
    m = getParam<double>(Vehiclemodel, "_m", 0); // mass of Vehicle
    Ccf = getParam<double>(Vehiclemodel, "_Ccf", 0);
    Ccr = getParam<double>(Vehiclemodel, "_Ccr", 0);
    Clf = getParam<double>(Vehiclemodel, "_Clf", 0);
    Clr = getParam<double>(Vehiclemodel, "_Clr", 0);
    delta_f = getParam<double>(Vehiclemodel, "_delta_f", 0);
    Sf = getParam<double>(Vehiclemodel, "_Sf", 0);
    Sr = getParam<double>(Vehiclemodel, "_Sr", 0);
    I = getParam<double>(Vehiclemodel, "_I", 0);
    wheel_base = getParam<double>(Vehiclemodel, "_L", 0);
    g = getParam<double>(Vehiclemodel, "_g", 0);
    R_min = getParam<double>(Vehiclemodel, "_Rmin", 0);
    K = getParam<double>(Vehiclemodel, "_K", 0);
    /********Controlparam***************/
    preview_time = getParam<double>(Controlparam, "preview_time", 0); // mass of Vehicle

    _p_post = getParam<double>(Controlparam, "_P_post", 0);
    // test cte_d  ang angle
    _lat_gain = getParam<double>(Controlparam, "_lat_gain", 0);
    _lat_angle_gain = getParam<double>(Controlparam, "_lat_angle_gain", 0);
    _dis_pre_max = getParam<double>(Controlparam, "_dis_pre_max", 0);
    _dis_pre_min = getParam<double>(Controlparam, "_dis_pre_min", 0);
    _wheel_angle_degree_max = getParam<double>(Controlparam, "_wheel_angle_degree_max", 0);
    _wheel_angle_degree_sec = getParam<double>(Controlparam, "_wheel_angle_degree_sec", 0);
    _t_preview = getParam<double>(Controlparam, "_t_preview", 2.8);
    _t_preview_turn = getParam<double>(Controlparam, "_t_preview_turn", 2.8);
    _delta_T = getParam<double>(Controlparam, "_delta_T", 0.02);
    _kappa = getParam<double>(Controlparam, "_kappa", 0);
    _smoothsteernum = getParam<double>(Controlparam, "_smoothsteernum", 0);
    path_size_min = getParam<double>(Controlparam, "path_size_min", 0);
    kappa_path_change_flag = getParam<double>(Controlparam, "kappa_path_change_flag", 0);
    yaw_error_max = getParam<double>(Controlparam, "yaw_error_max", 30);
    lat_error_max = getParam<double>(Controlparam, "lat_error_max", 1.5);
    PathLength_min = getParam<double>(Controlparam, "PathLength_min", 6);
    curvature_gain = getParam<double>(Controlparam, "curvature_gain", 0);

    //meanfilter
    lat_error_filter = MeanFilter(_smoothsteernum);
    heading_error_filter = MeanFilter(_smoothsteernum);
    kappa_filter = MeanFilter(_smoothsteernum);
    kappa_pure_filter = MeanFilter(_smoothsteernum);
    heading_ref_filter = MeanFilter(_smoothsteernum);

    //Interpolation
    LoadLatGainScheduler();
  }
  bool Pre_Lat_Controller::LoadLatGainScheduler()
  {
    Interpolation1D::DataType xy1, xy2;
    xy1 = {{4.0, 1.0},
           {8.0, 0.6},
           {12.0, 0.2},
           {20.0, 0.1}};
    xy2 = {{4.0, 1.0},
           {8.0, 0.6},
           {12.0, 0.4},
           {20.0, 0.1}};

    lat_err_interpolation_.reset(new Interpolation1D);
    if (!lat_err_interpolation_->Init(xy1))
    {
      cout << "LQR INIT lat_err_interpolation_ failed!!! " << endl;
      return false;
    }
    heading_err_interpolation_.reset(new Interpolation1D);
    if (!heading_err_interpolation_->Init(xy2))
    {
      return false;
    }
    cout << "PRELAT INIT heading_err_interpolation_ failed!!! " << endl;
    return true;
  }

  void Pre_Lat_Controller::run_follower(float *out_steerangle)
  {
    //
    if (Waypoints_size > 2)
      follower_flag = true;
    if (follower_flag)
    {
      _path_x = _path_x_rec;
      _path_y = _path_y_rec;
      _path_theta = _path_theta_rec;
      _path_curvature = _path_curvature_rec;
      _current_pose = _current_pose_rec;
      _vehicle_chassis_fd = _vehicle_chassis_fd_rec;

      Pathlength(_path_x, _path_y, &PathLength, &_path_s);
      delta_f = ControlStr(_current_pose.x,
                           _current_pose.y,
                           _current_pose.yaw,
                           _vehicle_chassis_fd.velocity_fd); //

      delta_f = meanfilter(filterdata, delta_f, _smoothsteernum);
      double car_finalpoints_dis = sqrt(pow((_current_pose.x - _path_x[Waypoints_size - 1]), 2) +
                                        pow((_current_pose.y - _path_y[Waypoints_size - 1]), 2));
      if (car_finalpoints_dis >= path_size_min)
      {
        delta_f_old = delta_f;
        *out_steerangle = delta_f;
        _dis_pre_max = 40;
        feedforward_K = 1;
      }
      else
      {
        *out_steerangle = delta_f;
        _dis_pre_max = car_finalpoints_dis;
        feedforward_K = 0;
      }
    }
    else
    {
      printf("Pathpoints lost!!! Waypoints_size = %d /n", Waypoints_size);
      *out_steerangle = 0;
    }
  }

  bool Pre_Lat_Controller::Pathlength(const std::vector<double> &path_x,
                                      const std::vector<double> &path_y,
                                      double *pathlength,
                                      std::vector<double> *path_s)
  {
    //
    if (path_x.size() < 2 && path_y.size() < 2)
      return false;
    if (path_s->size() > 0)
      path_s->clear();

    double path_length = 0;
    double dis_points = 0;
    path_s->push_back(0);

    dis_points = sqrt(pow((_current_pose.x - path_x[1]), 2) +
                      pow((_current_pose.y - path_y[1]), 2));

    for (int i = 0; i < (path_x.size() - 1); ++i)
    {
      dis_points = sqrt(pow((path_x[i + 1] - path_x[i]), 2) +

                        pow((path_y[i + 1] - path_y[i]), 2));

      path_s->push_back(dis_points);

      path_length += dis_points;
    }
    *pathlength = path_length;
    return true;
  }

  double Pre_Lat_Controller::ControlStr(double _vehicle_x, double _vehicle_y,
                                        double _vehicle_heading_theta,
                                        double _vehicle_v)
  {

    double _d_dis = 0;
    double _d_dis_k = 0;
    double _x_preview = 0;
    double _y_preview = 0;
    double _Oss = 0;
    double _Anglss = 0;
    double _transfer_val = 0;
    double _wheel_angle_degree = 0;

    // when enable follow
    if (_path_x.size() > 2 && _path_y.size() > 2 & _path_theta.size() > 2 && _path_curvature.size() > 2)
    {

      _vehicle_x = _vehicle_x + preview_time * _vehicle_v * cos(_vehicle_heading_theta);
      _vehicle_y = _vehicle_y + preview_time * _vehicle_v * sin(_vehicle_heading_theta);

      _d_dis_k = _t_preview * _vehicle_v + _d_0;
      _d_dis_k = limitParamChangeUnit(_d_dis_k, _dis_pre_max, _d_dis_min_k);
      double _x_preview_cur = _vehicle_x + _d_dis_k * cos(_vehicle_heading_theta);
      double _y_preview_cur = _vehicle_y + _d_dis_k * sin(_vehicle_heading_theta);

      // preview distance
      int nextpoint_num_1 = 0;
      int nextpoint_num_2 = 0;
      int nextpoint_num_3 = 0;
      double dis_next = 0;
      double kappa_path = 0;
      find_cloest_point_index(_path_x, _path_y, _vehicle_x, _vehicle_y, &nextpoint_num_1);
      if (PathLength > PathLength_min)
      {
        for (int i = 0; dis_next < 2 && nextpoint_num_1 + i < _path_s.size(); ++i)
        {
          dis_next += _path_s[nextpoint_num_1 + i];
          nextpoint_num_2 = nextpoint_num_1 + i;
        }
        dis_next = 0;
        for (int i = 0; dis_next < 2 && nextpoint_num_2 + i < _path_s.size(); ++i)
        {
          dis_next += _path_s[nextpoint_num_2 + i];
          nextpoint_num_3 = nextpoint_num_2 + i;
        }

        control_msgs::Point p1, p2, p3;
        p1.x = _path_x[nextpoint_num_1];
        p2.x = _path_x[nextpoint_num_2];
        p3.x = _path_x[nextpoint_num_3];
        p1.y = _path_y[nextpoint_num_1];
        p2.y = _path_y[nextpoint_num_2];
        p3.y = _path_y[nextpoint_num_3];
        kappa_path = CalculateCur(p1, p2, p3) + 1e-6;
      }
      else
      {
        kappa_path = 1e-6;
        cout << "kappa_path cann't get ~~~" << endl;
      }

      if (kappa_path > kappa_path_change_flag)
      {
        // fix927
        feedforward_K = 1;
        _d_dis = _t_preview * _vehicle_v + _d_0;
      }
      else
      {
        feedforward_K = 0.01;
        _d_dis = _t_preview_turn * _vehicle_v + _d_0;
      }
      double sig_k = 0;
      if (feedforward_K - feedforward_K_last > 0)
        sig_k = 1;
      if (feedforward_K - feedforward_K_last < 0)
        sig_k = -1;
      feedforward_K = feedforward_K_last + sig_k * 0.02;
      feedforward_K = limitParamChangeUnit(feedforward_K, 1, 0.3);
      feedforward_K_last = feedforward_K;

      //debug fix 1125 zx
      double curvature_angle = 0;
      static double curvature_angle_last = 0;
      double kappa_trajectory = _path_curvature[nextpoint_num_2];
      if (abs(kappa_trajectory) < 0.005)
      {
        kappa_trajectory = 0.0;
      }
      // kappa_trajectory = meanfilter(filterdata_kappa_trajectory, kappa_trajectory, _smoothsteernum);
      kappa_trajectory = kappa_filter.Update(kappa_trajectory);
      if (nextpoint_num_2 < _path_curvature.size())
      {
        curvature_angle = curvature_gain * atan(L * kappa_trajectory) * K / M_PI * 180;
        curvature_angle_last = curvature_angle;
      }
      else
      {
        curvature_angle = curvature_angle_last;
      }
      // cout << "  nextpoint_num_3  " << nextpoint_num_3 << "  nextpoint_num_2  " << nextpoint_num_2 << "  nextpoint_num_1  " << nextpoint_num_1 << endl;
      // cout << "  curvature_angle  " << curvature_angle << "  nextpoint_num_1  " << nextpoint_num_1 << endl;
      // cout << " _path_curvature[nextpoint_num_2]  " << _path_curvature[nextpoint_num_2] << " " << kappa_trajectory << endl;

      _d_dis = limitParamChangeUnit(_d_dis, _dis_pre_max, _d_dis_min);
      _x_preview = _vehicle_x + _d_dis * cos(_vehicle_heading_theta);
      _y_preview = _vehicle_y + _d_dis * sin(_vehicle_heading_theta);

      find_cloest_point_dis(_path_x, _path_y, _x_preview, _y_preview,
                            _vehicle_heading_theta, &_Oss);
      limitParamChangeUnit(_Oss, 3, -3);

      // coordinate system kappa puresuit method (l*l)/(2*x)
      control_msgs::Point pre_point;
      control_msgs::Point pre_point_test;
      int pre_number = 0;
      find_cloest_point_index(_path_x, _path_y, _x_preview_cur, _y_preview_cur,
                              &pre_number);
      //
      control_msgs::Point vehicle_pose;
      vehicle_pose.x = _x_preview_cur;
      vehicle_pose.y = _y_preview_cur;
//
#if 0
      find_cloest_point_coordinate(_path_x, _path_y, pre_number, vehicle_pose, &pre_point_test);
#endif
      pre_point.x = _path_x[pre_number];
      pre_point.y = _path_y[pre_number];
      pre_point.z = 0;
      // curvature_angle
      double kappa_pure = 0;
      double denominator = pow((pre_point.x - _current_pose.x), 2) +
                           pow((pre_point.y - _current_pose.y), 2);
      std::pair<double, double> numerator = pointtransformcoordinate(
          _current_pose.x, _current_pose.y, _current_pose.yaw,
          pre_point.x, pre_point.y);

      if (denominator != 0)
        kappa_pure = 2 * numerator.second / denominator;

      else
      {
        if (numerator.second > 0)
          kappa_pure = kappa_min;
        else
          kappa_pure = -kappa_min;
      }
      if (abs(kappa_pure) > 0.2)
      {
        if (kappa_pure > 0)
          kappa_pure = 1 / R_min;
        if (kappa_pure < 0)
          kappa_pure = -1 / R_min;
      }
      //
      kappa_pure = meanfilter(filterdata_kappa_pure, kappa_pure, _smoothsteernum);
      double _angle = 0;
      if (nextpoint_num_1 == _path_y.size() - 1)
      {
        _angle = atan2(_path_y[nextpoint_num_1] - _path_y[nextpoint_num_1 - 1],
                       _path_x[nextpoint_num_1] - _path_x[nextpoint_num_1 - 1]);
      }
      else
      {
        _angle = atan2(_path_y[nextpoint_num_1 + 1] - _path_y[nextpoint_num_1],
                       _path_x[nextpoint_num_1 + 1] - _path_x[nextpoint_num_1]);
      }
      double ys = fmod((_angle - _vehicle_heading_theta + M_PI), (2 * M_PI));
      if (ys < 0)
      {
        ys = ys + 2 * M_PI;
      }
      _Anglss = 0 - (ys - M_PI) / M_PI * 180.0;
      limitParamChangeUnit(_Anglss, yaw_error_max, -yaw_error_max);
      //filter
      //_Anglss = meanfilter(filterdata_angle_error, _Anglss, _smoothsteernum);

      _Anglss = heading_error_filter.Update(_Anglss);
      double _Anglss_ratio = heading_err_interpolation_->Interpolate(_vehicle_v);
      double angle_yaw_error = _Anglss * _lat_angle_gain;

      // TODO:: 2020.9.22 zhangxiang add current_pose lat_dis_error
      double lat_error = 0;
      find_cloest_point_dis(_path_x, _path_y, _vehicle_x, _vehicle_y, _vehicle_heading_theta, &lat_error);
      limitParamChangeUnit(lat_error, lat_error_max, -lat_error_max);
      //lat_error = meanfilter(filterdata_lat_error, lat_error, _smoothsteernum);
      lat_error = lat_error_filter.Update(lat_error);

      double lat_error_ratio = lat_err_interpolation_->Interpolate(_vehicle_v);
      double lat_angle_add = lat_error * _lat_gain;
      _transfer_val = cal_transfer_val(_d_dis, _vehicle_chassis_fd.velocity_fd);
      double _theta_ss = _transfer_val * _Oss;

      double feedforward_kappa_angle = feedforward_K * atan(L * kappa_pure) * 180 / M_PI * K * _kappa;
      double pre_dis_angle = _theta_ss * _p_post;
      // feedforward_kappa_angle =0;
      curvature_angle = 0;
      _wheel_angle_degree = curvature_angle + pre_dis_angle + feedforward_kappa_angle + angle_yaw_error + lat_angle_add;

      // DEBUG
      cout << "DEBUG  pre_dis_angle =  " << pre_dis_angle << " _theta_ss   " << _theta_ss << endl;
      cout << "DEBUG  feedforward_kappa_angle =  " << feedforward_kappa_angle << "  1/kappa_pure   " << 1 / kappa_pure << endl;
      cout << "DEBUG  angle_yaw_error =  " << angle_yaw_error << " _angle " << _angle << "  -  ys " << ys << "  = _Anglss　 " << _Anglss << endl;
      cout << "DEBUG  lat_angle_add =  " << lat_angle_add << " lat_error " << lat_error << endl;

      //
      ofstream out;
      out.open("/home/icv/follow_scuold.txt", std::ios::out | std::ios::app);
      out << setiosflags(ios::fixed) << setprecision(3) << curvature_angle << " " << angle_yaw_error << "  " << lat_angle_add << " " << feedforward_kappa_angle << "  " << pre_dis_angle << "  " << _wheel_angle_degree << endl;
      out.close();

      if (_flag < 30)
      {
        _wheel_angle_degree = limit_wheel_val(_vehicle_wheel_angle, _wheel_angle_degree);
      }
      else
      {
        _wheel_angle_degree = limit_wheel_val(_wheel_angle_degree_last, _wheel_angle_degree);
      }
      _wheel_angle_degree_last = _wheel_angle_degree;
      _cal_wheel_angle = _wheel_angle_degree;
      _record_cnt = 0;
    }
    _flag = _flag + 1;
    return _wheel_angle_degree;
  }

  bool Pre_Lat_Controller::find_cloest_point_index(
      const std::vector<double> &path_x, const std::vector<double> &path_y,
      double position_x, double position_y, int *count_min)
  {
    //

    if (path_x.size() < 2 && path_y.size() < 2)
      return false;

    double distance = 0.0;
    int _count_min = 0;
    double distance_min =
        pow((position_x - path_x[0]), 2) + pow((position_y - path_y[0]), 2);

    for (int i = 1; i < _path_x.size(); ++i)
    {
      distance =
          pow((position_x - path_x[i]), 2) + pow((position_y - path_y[i]), 2);
      if (distance <= distance_min)
      {
        distance_min = distance;
        _count_min = i;
      }
    }
    *count_min = _count_min;
    return true;
  }

  //011.30 add return cloest point's coordinate
  bool Pre_Lat_Controller::find_cloest_point_coordinate(const std::vector<double> &path_x,
                                                        const std::vector<double> &path_y,
                                                        const int cloest_index,
                                                        const control_msgs::Point &now_pose,
                                                        control_msgs::Point *cloest_pathpoint)
  {
    control_msgs::Point end;
    control_msgs::Point start;
    control_msgs::Point cloest_pose;

    if (cloest_index == 0)
    {
      start.x = path_x[0];
      start.y = path_y[0];
      end.x = path_x[1];
      end.y = path_y[1];
    }
    else
    {
      start.x = path_x[cloest_index - 1];
      start.y = path_y[cloest_index - 1];
      end.x = path_x[cloest_index];
      end.y = path_y[cloest_index];
    }
    double a = 0;
    double b = 0;
    double c = 0;
    double get_linear_flag = getLinearEquation(start, end, &a, &b, &c);
    getcloestPointCoorindate(now_pose, &cloest_pose, a, b, c);
    *cloest_pathpoint = cloest_pose;
    return true;
  }

  bool Pre_Lat_Controller::find_cloest_point_dis(const std::vector<double> &path_x,
                                                 const std::vector<double> &path_y,
                                                 double positon_x, double positon_y,
                                                 double positon_theta,
                                                 double *dis_min)
  {
    //
    if (path_x.size() < 2 && path_y.size() < 2)
      return false;
    double distance_min =
        pow((positon_x - path_x[0]), 2) + pow((positon_y - path_y[0]), 2);
    double distance = 0.0;
    double _x_min = path_x[0];
    double _y_min = path_y[0];
    int _count_min = 0;

    for (int i = 1; i < _path_x.size(); ++i)
    {
      distance =
          pow((positon_x - path_x[i]), 2) + pow((positon_y - path_y[i]), 2);
      if (distance <= distance_min)
      {
        distance_min = distance;
        _x_min = path_x[i];
        _y_min = path_y[i];
        _count_min = i;
      }
    }
    // distance_min = sqrt(pow((positon_x - path_x[_count_min]), 2) +
    //                     pow((positon_y - path_y[_count_min]), 2));
    // linear equation

    control_msgs::Point end;
    control_msgs::Point start;
    control_msgs::Point current_pose;

    current_pose.x = positon_x;
    current_pose.y = positon_y;

    if (_count_min == 0)
    {
      start.x = path_x[0];
      start.y = path_y[0];
      end.x = path_x[1];
      end.y = path_y[1];
    }
    else
    {
      start.x = path_x[_count_min - 1];
      start.y = path_y[_count_min - 1];
      end.x = path_x[_count_min];
      end.y = path_y[_count_min];
    }

    double a = 0;
    double b = 0;
    double c = 0;
    double get_linear_flag = getLinearEquation(start, end, &a, &b, &c);
    if (!get_linear_flag)
    {
      return false;
    }
    double d = getDistanceBetweenLineAndPoint(current_pose, a, b, c);

    // judge distance sign
    //                     |      |
    //  car here is +      | road |   car here is -
    //                     |      |
    double _vehicle_x_ref = positon_x - 0.01 * cos(positon_theta);
    double _vehicle_y_ref = positon_y - 0.01 * sin(positon_theta);
    if ((positon_x - _vehicle_x_ref) * (_y_min - _vehicle_y_ref) -
            (positon_y - _vehicle_y_ref) * (_x_min - _vehicle_x_ref) >=
        0)
    {
      *dis_min = d;
    }
    else
    {
      *dis_min = -1.0 * d;
    }
    return true;
  }

  double Pre_Lat_Controller::cal_transfer_val(double _d_dis_tmp, double _vehicle_v)
  {
    double _molecular =
        2 * (lf + lr - (m * (lf * Ccf - lr * Ccr)) / ((lf + lr) * Ccf * Ccr));
    double _T = lr - ((lf * m * (_vehicle_v * _vehicle_v)) / (Ccr * (lf + lr)));
    double _denominator = _d_dis_tmp * (_d_dis_tmp + 2 * _T);
    double _trans_val = _molecular / _denominator;
    return _trans_val;
  }

  double Pre_Lat_Controller::limit_wheel_val(double _wheel_last_ref_rec,
                                             double _wheel_angle_degree_rec)
  {
    double _wheel_now;
    double _wheel_last;

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
  double Pre_Lat_Controller::CalculateCur(control_msgs::Point P1,
                                          control_msgs::Point P2,
                                          control_msgs::Point P3)
  {
    double curvity_pre = 0; // last time cur
    double curvity = 0;     // current cur
    double curvity_R = 0;   // current radius
    double speed_pre = 0;   // last speed
    double speed_cur = 0;   // current speed
    double acc = 0;
    double acc_pre = 0; // last time acc
    // judge three point not in same line
    if (P1.x == P2.x == P3.x || P1.y == P2.y == P3.y)
    {
      curvity = 0;
    }
    else
    {
      if (P1.x == P2.x && P1.y == P2.y || P1.x == P3.x && P1.y == P3.y ||
          P2.x == P3.x && P2.y == P3.y)
      {
        // ROS_ERROR(" Path have same points!!!!  ");
      }
      else
      {
        double dis1, dis2, dis3;
        double cosA, sinA, dis;
        dis1 =
            sqrt((P1.x - P2.x) * (P1.x - P2.x) + (P1.y - P2.y) * (P1.y - P2.y));
        dis2 =
            sqrt((P1.x - P3.x) * (P1.x - P3.x) + (P1.y - P3.y) * (P1.y - P3.y));
        dis3 =
            sqrt((P2.x - P3.x) * (P2.x - P3.x) + (P2.y - P3.y) * (P2.y - P3.y));
        dis = dis1 * dis1 + dis3 * dis3 - dis2 * dis2;
        cosA = dis / (2 * dis1 * dis3) + 1e-6;
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

  double Pre_Lat_Controller::limitParamChangeUnit(double param, double max_param,
                                                  double min_param)
  {
    // TODO::limit parameter boundary
    // input::limit param,max and min param
    // output:: param after limit
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
  // 10.09add linear equation

  bool Pre_Lat_Controller::getLinearEquation(control_msgs::Point start,
                                             control_msgs::Point end, double *a,
                                             double *b, double *c)
  {
    //(x1, y1) = (start.x, star.y), (x2, y2) = (end.x, end.y)
    double sub_x = fabs(start.x - end.x);
    double sub_y = fabs(start.y - end.y);
    double error = pow(10, -5); // 0.00001

    if (sub_x < error && sub_y < error)
    {
      printf("two points are the same point!!");
      return false;
    }

    *a = end.y - start.y;
    *b = (-1) * (end.x - start.x);
    *c = (-1) * (end.y - start.y) * start.x + (end.x - start.x) * start.y;

    return true;
  }
  // let the center of circle be "(x0,y0)", in my code , the center of circle is
  // vehicle position the distance  "d" between the foot of a perpendicular line
  // and the center of circle is ...
  //    | a * x0 + b * y0 + c |
  // d = -------------------------------
  //          √( a~2 + b~2)
  double Pre_Lat_Controller::getDistanceBetweenLineAndPoint(
      control_msgs::Point point, double a, double b, double c)
  {
    double d = fabs(a * point.x + b * point.y + c) / sqrt(pow(a, 2) + pow(b, 2));

    return d;
  }

  bool Pre_Lat_Controller::getcloestPointCoorindate(control_msgs::Point Point_P, control_msgs::Point *PointonLine, double a,
                                                    double b, double c1)
  {
    // K1 * K2 = -1
    //      - a * c1 + b * c2
    //x1 = ------------------------   y1 = (b/a) * x1 - c2 / a;
    //        a * a  + b * b
    control_msgs::Point point;
    if (Point_P.y == (a / b) * Point_P.x + c1 / b)
    {
      *PointonLine = Point_P;
      return true;
    }
    if (a == 0)
    {
      point.x = Point_P.x;
      if (c1 != 0)
      {
        point.y = b / c1;
      }
      else
      {
        point.y = 0;
      }
      *PointonLine = point;
      return true;
    }
    if (b == 0)
    {
      point.y = Point_P.y;
      if (c1 != 0)
      {
        point.x = a / c1;
      }
      else
      {
        point.x = 0;
      }
      *PointonLine = point;
      return true;
    }

    float c2 = -a * Point_P.y + b * Point_P.x;
    float x1 = (-a * c1 + b * c2) / (a * a + b * b);
    float y1 = (b / a) * x1 - (c2 / a);
    point.x = x1;
    point.y = y1;
    *PointonLine = point;
    return true;
  }
} // namespace Control
