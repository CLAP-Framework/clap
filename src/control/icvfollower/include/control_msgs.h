/*
 * @Descripttion:
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-04 21:51:03
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2020-09-15 14:57:18
 */
namespace Control {
namespace control_msgs {
// struct //need in  comment struct headfile
/**unit
 * m
 * m
 * rad
 * m/s
 * **/
struct ego_pose {
  float x;
  float y;
  float yaw;
  float velocity;
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
struct Pathpoint {
  float x;
  float y;
  float z;
  float yaw;
  float pitch;
  float roll;
  float curvature;
};
/**unit
 * m/s
 * m/s2
 * m/s3
 * **/
struct Twistpoint {
  float velocity;
  float acceleration;
  float jerk;
};
/**unit
 * Pathpoint
 * Twisitpoint
 * **/
struct Trajectorypoint {
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
struct Vehicle_status {
  float steerangle_fd;
  float steeranglespeed_fd;
  float velocity_fd;
  float acceleration_fd;
  float jerk_fd;
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
struct Vehicle_control {
  float steerangle_req;
  float steeranglespeed_req;
  float velocity_req;
  float throttle;
  float brake_req;
  float acceleration;
};

}  // namespace control_msgs
}  // namespace Control