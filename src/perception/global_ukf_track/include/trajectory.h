
#pragma once

// #include <iostream>
// #include <string>


namespace ele {

struct Vector2D {
  Vector2D() : x(0.), y(0.) {}
  double x;
  double y;
};
struct Vector3D {
  Vector3D() : x(0.), y(0.), z(0.) {}
  double x;
  double y;
  double z;
};
struct Vector4D {
  Vector4D() : x(0.), y(0.), z(0.), w(0.) {}
  double x;
  double y;
  double z;
  double w;
};

typedef struct Vector2D Point2D;
typedef struct Vector3D Point3D;
typedef struct Vector2D Pose2D;
typedef struct Vector3D Pose3D;
typedef struct Vector2D Dimension2D;
typedef struct Vector3D Dimension3D;
typedef struct Vector2D Velocity2D;
typedef struct Vector3D Velocity3D;
typedef struct Vector2D Accel2D;
typedef struct Vector3D Accel3D;
typedef struct Vector4D Orientation3D;

struct ColorRGBA {
  ColorRGBA() : r(0.), g(0.), b(0.), a(0.) {}
  float r;
  float g;
  float b;
  float a;
};

struct LidarFeatures {
  LidarFeatures() : yaw(0.) {}
  /** center point (x,y) */
  Point2D c; 
  /** front left point (x,y) */
  Point2D fl;
  /** rear right point (x,y) */
  Point2D rr;
  /** attitude angle yaw */
  float yaw;
}; // struct LidarFeatures

struct LidarDetectObject {
  LidarDetectObject() : timestamp(0), label(0), score(0.) {}
  // unsigned int    seq;
  // unsigned int    id;
  long long       timestamp;
  int             label;
  float           score;
  Pose3D          pose;
  Dimension3D     dimension;
  Orientation3D   orientation;
  LidarFeatures   features;
}; // struct LidarObject 

struct LidarTrackObject {
  unsigned int    id;
  long long       timestamp;
  int             label;
  float           score;
  Pose3D          pose;
  Dimension3D     dimension;
  Orientation3D   orientation;
  LidarFeatures   features;
  Velocity3D      velocity;

  ColorRGBA       color;
  bool            valid;
  bool            pose_reliable;
  bool            velocity_reliable;
  Pose2D          pose2d;
  Dimension3D     dimension2d;
}; // struct LidarTrackObject 

// struct LidarTrackObject {
//     // unsigned int    seq;
//     unsigned int    id;
//     long long       timestamp;
//     int             label;
//     float           score;
//     Pose3D          pose;
//     Dimension3D     dimension;
//     Orientation3D   orientation;
//     Velocity3D      velocity;
// }; // struct LidarTrackObject 



enum TrackingState : int {
  Die = 0,     // No longer tracking
  Init = 1,    // Start tracking
  Stable = 4,  // Stable tracking
  Occlusion = 5, // Lost 1 frame possibly by occlusion
  Lost = 10,   // About to lose target
}; // enum TrackingState

struct Trajectory {
  Trajectory() : id(0), 
      matched(false), 
      state(Die), 
      miss_time(0), 
      life_time(0), 
      is_stabled(false), 
      initialized(false) {}
  Trajectory(LidarTrackObject& in_obj, unsigned int in_id) : 
      obj(in_obj), 
      id(in_id), 
      matched(false), 
      state(Init), 
      miss_time(0), 
      life_time(1), 
      is_stabled(false), 
      initialized(false) {}
  LidarTrackObject    obj;
  unsigned int        id;    
  bool                matched;
  TrackingState       state;
  int                 miss_time;
  int                 life_time;
  bool                is_stabled;
  bool                initialized;
}; // struct Trajectory

} // namespace ele 