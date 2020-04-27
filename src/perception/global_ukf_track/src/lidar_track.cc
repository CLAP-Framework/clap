#include "assignment.h"
#include "lidar_track.h"
#include "ukf.h"
#include "trajectory.h"
#include "Eigen/Dense"
#include <vector>
#include <iostream>

namespace ele {

void TrajectoryUKF::ProcessPrediction(long long timestamp) {
    /** Prediction */
    double delta_t = (timestamp - time_us_) / 1000000.0;  // convert unit from us to s.
    // when we apply Euler method for large steps, we are assuming that the derivative of the states is constant over
    // that whole interval, this leads to error in state propagation, so we divide the one big step to several small ones.
    // *See detail discussion* @ https://discussions.udacity.com/t/numerical-instability-of-the-implementation/230449/55
    while (delta_t > 0.1){
        const double dt = 0.05;
        Prediction(dt);
        delta_t -= dt;
    }
    Prediction(delta_t);
    traj_.obj.features.c.x = x_(0);
    traj_.obj.features.c.y = x_(1);
}

void TrajectoryUKF::ProcessUpdate(const MeasurementPackage& meas_package) {
    /** Initialization */
    if (!is_initialized_){
        Initialization(meas_package);
        return;
    }
  // convert unit from us to s.
    // double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;  
    time_us_ = meas_package.timestamp_;

    /** Update */
    if (meas_package.sensor_type_ == MeasurementPackage::LASER){
        UpdateLidar(meas_package);
    }
}

LidarTrack::LidarTrack() : 
    track_id_(1), 
    dt_(0.), 
    initialized_(false), 
    dis_sqrt_thres_(4.44 * 4.44) {

}

LidarTrack::~LidarTrack() {

}

bool LidarTrack::Track() {
    // TODO: if (in_objs_.size() == 0)
    int rows = trajs_.size();
    int cols = in_objs_.size();
    std::cout << "trajs size " << rows << "objs size " << cols  << std::endl;
    // if in_objs_.size() != 0
    std::vector<bool> obj_matched(in_objs_.size(), false);
    // init
    if (! initialized_) { 
        initialized_ = true;
    } else { // has initialized
#if 0
        /** Predict */
        for(auto i=0; i<rows; ++i){
            trajs_[i].ProcessPrediction(timestamp_);
        }
        /** Match */
        std::cout << "create Assingment" << std::endl;
        ele::Assignment<ele::TrajectoryUKF, ele::LidarTrackObject> assign_engine(&trajs_, &in_objs_, 
                [](ele::TrajectoryUKF& left_class, ele::LidarTrackObject& right_class) -> double {
                    return ( (left_class.traj_.obj.features.c.x - right_class.features.c.x) 
                            * (left_class.traj_.obj.features.c.x - right_class.features.c.x) 
                            + (left_class.traj_.obj.features.c.y - right_class.features.c.y)
                            * (left_class.traj_.obj.features.c.y - right_class.features.c.y) );
                } ,
                dis_sqrt_thres_ );
        std::cout << "Assingment created and start Solve" << std::endl;
        assign_engine.Solve();
        std::cout << "Assingment Solved and get result" << std::endl;
        std::vector<int> result = assign_engine.Assign(); 
        std::cout << "get result and print" << std::endl;
        for (auto i = 0; i < result.size(); ++i){
            if ( -1 != result[i] ){
                trajs_[i].traj_.matched = true;
                trajs_[i].traj_.obj = in_objs_[result[i]];
                obj_matched[result[i]] = true;
                std::cout << "traj " << i << " matched obj " << result[i] << std::endl;
            }  
        }
#else
        /** step 1: distance select */
        Eigen::MatrixXd dis_matrix(rows, cols);
        dis_matrix.fill(0.0);
        std::vector<std::vector<int> > dis_matrix_index(rows);
        // dis_matrix_index.resize(rows);

        // get objects within distance threshold indices 
        for(auto i=0; i<rows; ++i){
            trajs_[i].ProcessPrediction(timestamp_);
            for(auto j=0; j<cols; ++j){
                dis_matrix(i, j) = PointDistanceSqrt(trajs_[i].traj_.obj.features.c, in_objs_[j].features.c);
                if (dis_matrix(i, j) < dis_sqrt_thres_) {
                    dis_matrix_index[i].emplace_back(j);
                }
            }
        }


        /** 1 vs 1 match */
        for (auto i = 0; i < rows; ++i){
            if ( 1 == dis_matrix_index[i].size() ){
                trajs_[i].traj_.matched = true;
                trajs_[i].traj_.obj = in_objs_[dis_matrix_index[i].at(0)];
                obj_matched[dis_matrix_index[i].at(0)] = true;
                // std::cout << "1v1 matched obj " << dis_matrix_index[i].at(0) << std::endl;
            }  
        }

        /** 1 vs multiple match */
        for (auto i = 0; i < rows; i++) { 
            if (trajs_[i].traj_.matched || dis_matrix_index[i].empty() ) {
                continue;
            }
            if (1 < dis_matrix_index[i].size()) {
                double min_dis = std::numeric_limits<double>::max();
                int min_indice = -1;
                // int min_dis_indice = dis_matrix_index[i].at(0);
                for (size_t j = 0; j < dis_matrix_index[i].size(); j++) {
                    if ( obj_matched[dis_matrix_index[i].at(j)] ) {
                        continue;
                    }
                    double dis = PointDistanceSqrt(trajs_[i].traj_.obj.features.c, 
                                    in_objs_[dis_matrix_index[i].at(j)].features.c);
                    if (dis < min_dis) {
                        min_indice = dis_matrix_index[i].at(j);
                        min_dis = dis; 
                    }
                } 
                if (min_indice != -1) {
                    trajs_[i].traj_.matched = true;
                    trajs_[i].traj_.obj = in_objs_[min_indice];
                    obj_matched[min_indice] = true;
                    // std::cout << "1vN matched obj " << min_indice << std::endl;
                }
            } 
        }
#endif
        /** life time manage */
        for (auto& traj : trajs_) {
            if (traj.traj_.matched ) {
                traj.traj_.life_time++;
                // if Stable, do nothing
                if (traj.traj_.state == Init) {
                    if (traj.traj_.life_time > birth_thres_) {
                        traj.traj_.life_time = 0;
                        traj.traj_.miss_time = 0;
                        traj.traj_.state = Stable;
                        traj.traj_.is_stabled = true;
                    }
                    continue;
                }
                if (traj.traj_.state == Occlusion) {
                    traj.traj_.life_time = 0;
                    traj.traj_.miss_time = 0;
                    traj.traj_.state = Stable;
                    traj.traj_.is_stabled = true;
                    continue;
                }
                if ( traj.traj_.state == Stable ) {
                    traj.traj_.life_time = 0;
                    // traj.traj_.miss_time = 0;
                    // traj.traj_.state = Stable;
                    traj.traj_.is_stabled = true;
                }
            } else {
                traj.traj_.miss_time++;
                traj.traj_.is_stabled = false;
                if (traj.traj_.miss_time > die_thres_) {
                    traj.traj_.state = Die;
                    continue;
                }
                // if Stable
                if (traj.traj_.state == Stable) {
                    traj.traj_.life_time = 0;
                    traj.traj_.state = Occlusion;
                    traj.traj_.is_stabled = false;
                    continue;
                }
                if (traj.traj_.state == Init) {
                    // if (traj.traj_.life_time < traj.traj_.miss_time )
                    traj.traj_.state = Die;
                }
            }
        }

        /** remove die objects */
        for (auto it = trajs_.begin(); it != trajs_.end(); ) {
            if (it->traj_.state == Die ) {
                // std::cout << "remove traj " << it->traj_.id << std::endl;
                it = trajs_.erase(it);
            } else {
                ++it;                
            }
        }
    }

    /** Add new objects */
    for (auto j=0; j<cols; j++ ) {
        if ( obj_matched[j] ) {
            continue;
        }
        
        // add new object
        AddNewTrajectory(in_objs_.at(j), track_id_);
        // std::cout << "add traj " << track_id_ << std::endl;
        track_id_++;
    }

    /** filter */
    out_objs_.clear();
    for (auto& traj : trajs_) {
        traj.traj_.matched = false;
        MeasurementPackage meas;
        meas.sensor_type_ = MeasurementPackage::LASER;
        meas.timestamp_ = timestamp_;
        meas.raw_measurements_ = Eigen::VectorXd(2);
        meas.raw_measurements_ << traj.traj_.obj.pose.x, traj.traj_.obj.pose.y;
        // traj.ProcessMeasurement(meas);
        traj.ProcessUpdate(meas);

        
        switch (traj.traj_.state) {
            case Stable:
                traj.traj_.obj.valid = true;
                traj.traj_.obj.pose_reliable = true;
                traj.traj_.obj.velocity_reliable = true;
                break;
            case Init:
                traj.traj_.obj.valid = true;
                traj.traj_.obj.pose_reliable = true;
                traj.traj_.obj.velocity_reliable = false;
                break;
            case Occlusion:
                traj.traj_.obj.valid = true;
                traj.traj_.obj.pose_reliable = true;
                traj.traj_.obj.velocity_reliable = true;
                break;
            default:

                break;
        }
        traj.traj_.obj.id = traj.traj_.id;
        traj.traj_.obj.pose.x = traj.x_(0);
        traj.traj_.obj.pose.y = traj.x_(1);
        traj.traj_.obj.velocity.x = traj.x_(2) * cos(traj.x_(3));
        traj.traj_.obj.velocity.y = traj.x_(2) * sin(traj.x_(3));
        out_objs_.push_back(traj.traj_.obj);
    }

    //
    return true;
}

double LidarTrack::PointDistanceSqrt(const Point2D& p1, const Point2D& p2) const {
    return ( (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) );
}

bool LidarTrack::InDistanceThreshold (const LidarTrackObject& obj_a, 
                          const LidarTrackObject& obj_b) const {
    if ( dis_sqrt_thres_ > PointDistanceSqrt(obj_a.features.c, obj_b.features.c) ||
         dis_sqrt_thres_ > PointDistanceSqrt(obj_a.features.fl, obj_b.features.fl) ||
         dis_sqrt_thres_ > PointDistanceSqrt(obj_a.features.rr, obj_b.features.rr) ) {
        return true;
    }
    return false;
}

bool LidarTrack::AddNewTrajectory(const LidarTrackObject& obj, unsigned int id) {
    TrajectoryUKF traj_ukf;
    traj_ukf.traj_.id = id;
    traj_ukf.traj_.obj = obj;
    traj_ukf.traj_.life_time = 1;
    traj_ukf.traj_.matched = false;
    traj_ukf.traj_.miss_time = 0;
    traj_ukf.traj_.state = Init;
    traj_ukf.traj_.is_stabled = false;
    trajs_.push_back(traj_ukf);
    return true;
}



} // namespace ele