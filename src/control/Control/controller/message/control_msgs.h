
#pragma once
/*
 * @Descripttion:
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-04 21:51:03
 * @LastEditors: xiangzhang
 * @LastEditTime: 2020-10-26 13:55:55
 */
namespace Control
{
  namespace control_msgs
  {
    // struct //need in  comment struct headfile
    /**unit
 * m
 * m
 * rad
 * m/s
 * **/
    struct ego_pose
    {
      float x = 0;
      float y = 0;
      float yaw = 0;
      float velocity = 0;
    };
    /**unit
 * m
 * m
 * m
 */
    struct Point
    {
      float x = 0;
      float y = 0;
      float z = 0;
    };
    /**unit
 * m
 * m
 * m
 * rad
 * rad
 * rad
 * 1/m
 * **/
    struct Pathpoint
    {
      float x = 0;
      float y = 0;
      float z = 0;
      float yaw = 0;
      float pitch = 0;
      float roll = 0;
      float curvature = 0;
      float s = 0;
    };
    /**unit
 * m/s
 * m/s2
 * m/s3
 * **/
    struct Twistpoint
    {
      float velocity = 0;
      float acceleration = 0;
      float jerk = 0;
    };
    /**unit
 * Pathpoint
 * Twisitpoint
 * **/
    struct Trajectorypoint
    {
      Pathpoint pose;
      Twistpoint twist;
    };

    /**unit
 * deg
 * deg/s
 * m/s
 * m/s2
 * m/s3
 * **/
    struct Vehicle_status
    {
      float steerangle_fd = 0;
      float steeranglespeed_fd = 0;
      float velocity_fd = 0;
      float acceleration_fd = 0;
      float jerk_fd = 0;
      int gear_fd = 0;
      float yaw = 0;
      float pitch = 0;
      float roll = 0;
      int automode = 0;
    };
    /**unit
 * deg
 * deg/s
 * m/s
 * %
 * %
 * m/s2
 *
 */
    struct Vehicle_control
    {
      float steerangle_req = 0;
      float steeranglespeed_req = 0;
      float velocity_req = 0;
      float throttle = 0;
      float brake_req = 0;
      float acceleration = 0;
      int gear_req = 0;
    };

  } // namespace control_msgs
} // namespace Control
