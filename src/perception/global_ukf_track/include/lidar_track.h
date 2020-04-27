// #pragma once 
#ifndef _LIDAR_TRACK_H_
#define _LIDAR_TRACK_H_

#include "ukf.h"
#include "trajectory.h"
// #include "Eigen/Dense"
#include <vector>
// #include <iostream>

namespace ele {

struct Pair {
  int a;
  int b;
};

class TrajectoryUKF : public UKF{
public:
  void ProcessPrediction(long long timestamp);
  void ProcessUpdate(const MeasurementPackage& meas_package);
public:
  Trajectory traj_;
};

class LidarTrack {
public:
  LidarTrack();
  ~LidarTrack();

  bool Track () ;
  bool AddNewTrajectory(const LidarTrackObject& obj, 
      unsigned int id);
  // protected:

private:
  double PointDistanceSqrt(const Point2D& p1, 
      const Point2D& p2) const;
  bool InDistanceThreshold (const LidarTrackObject& obj_a, 
      const LidarTrackObject& obj_b) const;
// private:
public:
  /** trajectories of tracked objects */
  std::vector<TrajectoryUKF> trajs_;
  /** input detected objects */
  std::vector<LidarTrackObject> in_objs_;
  /** output tracked objects */
  std::vector<LidarTrackObject> out_objs_;
  /** tracker id */
  unsigned int track_id_;
  /** timestamp of input objects */
  long long timestamp_;
  /** time step */
  double dt_;
  /** initialized flag, default false */
  bool initialized_;
  /** match distance threshold sqrt */
  double dis_sqrt_thres_;
  /** birth threshold */
  int birth_thres_;
  /** die threshold */
  int die_thres_;
}; // class LidarTrack

} // namespace ele

#endif // _LIDAR_TRACK_H_