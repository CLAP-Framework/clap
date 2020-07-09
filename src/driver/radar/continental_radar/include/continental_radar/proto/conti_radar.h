
#ifndef PROTO_CONTI_RADAR_H_
#define PROTO_CONTI_RADAR_H_

#include <cstring>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>
#include <array>

namespace drivers {
class ClusterListStatus_600;
class ContiRadar;
class ContiRadarObs;
class ObjectListStatus_60A;
class RadarState_201;

namespace conti_radar {
enum {
  Radar_State = 0x201,
  Cluster_List_Status = 0x600,
  Object_List_Status = 0x60A,  
  Object_General_Info = 0x60B,
  Object_Quality_Info = 0x60C,
  Object_Extended_Info = 0x60D,
  Cluster_General_Info = 0x701,
  Cluster_Quality_Info = 0x702,
};
} // namespace conti_radar
}  // namespace drivers

namespace drivers {

class ClusterListStatus_600 
{
public:
  ClusterListStatus_600();
  virtual ~ClusterListStatus_600();
  ClusterListStatus_600(const ClusterListStatus_600& from);
  inline ClusterListStatus_600& operator=(const ClusterListStatus_600& from)
  {
      has_bits_ = from.has_bits_;
      near_ = from.near_;
      far_ = from.far_;
      interface_version_ = from.interface_version_;
      meas_counter_ = from.meas_counter_;
      return *this;
  }
public:
  bool has_near() const;
  void clear_near();
  uint32_t near() const;
  void set_near(uint32_t value);

  bool has_far() const;
  void clear_far();
  uint32_t far() const;
  void set_far(uint32_t value);

  bool has_interface_version() const;
  void clear_interface_version();
  uint32_t interface_version() const;
  void set_interface_version(uint32_t value);

  bool has_meas_counter() const;
  void clear_meas_counter();
  uint32_t meas_counter() const;
  void set_meas_counter(uint32_t value);

 private:
  uint32_t has_bits_;
  uint32_t near_;
  uint32_t far_;
  uint32_t interface_version_;
  uint32_t meas_counter_;
};
// -------------------------------------------------------------------

class ObjectListStatus_60A 
{
 public:
  ObjectListStatus_60A();
  virtual ~ObjectListStatus_60A();
  ObjectListStatus_60A(const ObjectListStatus_60A& from);
  inline ObjectListStatus_60A& operator=(const ObjectListStatus_60A& from) {
    has_bits_ = from.has_bits_;
    nof_objects_ = from.nof_objects_;
    interface_version_ = from.interface_version_;
    meas_counter_ = from.meas_counter_;
    return *this;
  }

  public:
  bool has_nof_objects() const;
  void clear_nof_objects();
  uint32_t nof_objects() const;
  void set_nof_objects(uint32_t value);

  bool has_interface_version() const;
  void clear_interface_version();
  uint32_t interface_version() const;
  void set_interface_version(uint32_t value);

  bool has_meas_counter() const;
  void clear_meas_counter();
  uint32_t meas_counter() const;
  void set_meas_counter(uint32_t value);

 private:
  uint32_t has_bits_;
  uint32_t nof_objects_;
  uint32_t interface_version_;
  uint32_t meas_counter_;

};
// -------------------------------------------------------------------

class RadarState_201 
{
 public:
  RadarState_201();
  virtual ~RadarState_201();
  RadarState_201(const RadarState_201& from);
  inline RadarState_201& operator=(const RadarState_201& from) {
      has_bits_ = from.has_bits_;
      max_distance_ = from.max_distance_;
      radar_power_ = from.radar_power_;
      output_type_ = from.output_type_;
      rcs_threshold_ = from.rcs_threshold_;
      send_quality_ = from.send_quality_;
      send_ext_info_ = from.send_ext_info_;
      return *this;
  }

  public:
  bool has_max_distance() const;
  void clear_max_distance();
  uint32_t max_distance() const;
  void set_max_distance(uint32_t value);

  bool has_radar_power() const;
  void clear_radar_power();
  uint32_t radar_power() const;
  void set_radar_power(uint32_t value);

  bool has_output_type() const;
  void clear_output_type();
  uint32_t output_type() const;
  void set_output_type(uint32_t value);

  bool has_rcs_threshold() const;
  void clear_rcs_threshold();
  uint32_t rcs_threshold() const;
  void set_rcs_threshold(uint32_t value);

  bool has_send_quality() const;
  void clear_send_quality();
  bool send_quality() const;
  void set_send_quality(bool value);

  bool has_send_ext_info() const;
  void clear_send_ext_info();
  bool send_ext_info() const;
  void set_send_ext_info(bool value);

 private:
  uint32_t has_bits_;
  uint32_t max_distance_;
  uint32_t radar_power_;
  uint32_t output_type_;
  uint32_t rcs_threshold_;
  bool send_quality_;
  bool send_ext_info_;
};
// -------------------------------------------------------------------

class ContiRadarObs 
{
 public:
  ContiRadarObs();
  virtual ~ContiRadarObs();
  ContiRadarObs(const ContiRadarObs& from);
  inline ContiRadarObs& operator=(const ContiRadarObs& from) {
    has_bits_=(from.has_bits_);
    clusterortrack_=(from.clusterortrack_);
    obstacle_id_=(from.obstacle_id_);
    longitude_dist_=(from.longitude_dist_);
    lateral_dist_=(from.lateral_dist_);
    longitude_vel_=(from.longitude_vel_);
    lateral_vel_=(from.lateral_vel_);
    rcs_=(from.rcs_);
    longitude_dist_rms_=(from.longitude_dist_rms_);
    lateral_dist_rms_=(from.lateral_dist_rms_);
    longitude_vel_rms_=(from.longitude_vel_rms_);
    dynprop_=(from.dynprop_);
    meas_state_=(from.meas_state_);
    lateral_vel_rms_=(from.lateral_vel_rms_);
    probexist_=(from.probexist_);
    longitude_accel_=(from.longitude_accel_);
    lateral_accel_=(from.lateral_accel_);
    oritation_angle_=(from.oritation_angle_);
    longitude_accel_rms_=(from.longitude_accel_rms_);
    lateral_accel_rms_=(from.lateral_accel_rms_);
    oritation_angle_rms_=(from.oritation_angle_rms_);
    length_=(from.length_);
    width_=(from.width_);
    obstacle_class_=(from.obstacle_class_);
    return *this;
  }

  public:
  // optional uint64 header = 1;
  // bool has_header() const;
  // void clear_header();
  // static const int kHeaderFieldNumber = 1;
  // // ::PROTOBUF_NAMESPACE_ID::uint64 header() const;
  // void set_header(::PROTOBUF_NAMESPACE_ID::uint64 value);

  bool has_clusterortrack() const;
  void clear_clusterortrack();
  bool clusterortrack() const;
  void set_clusterortrack(bool value);

  bool has_obstacle_id() const;
  void clear_obstacle_id();
  uint32_t obstacle_id() const;
  void set_obstacle_id(uint32_t value);

  bool has_longitude_dist() const;
  void clear_longitude_dist();
  double longitude_dist() const;
  void set_longitude_dist(double value);

  bool has_lateral_dist() const;
  void clear_lateral_dist();
  double lateral_dist() const;
  void set_lateral_dist(double value);

  bool has_longitude_vel() const;
  void clear_longitude_vel();
  double longitude_vel() const;
  void set_longitude_vel(double value);

  bool has_lateral_vel() const;
  void clear_lateral_vel();
  double lateral_vel() const;
  void set_lateral_vel(double value);

  bool has_rcs() const;
  void clear_rcs();
  double rcs() const;
  void set_rcs(double value);

  bool has_longitude_dist_rms() const;
  void clear_longitude_dist_rms();
  double longitude_dist_rms() const;
  void set_longitude_dist_rms(double value);

  bool has_lateral_dist_rms() const;
  void clear_lateral_dist_rms();
  double lateral_dist_rms() const;
  void set_lateral_dist_rms(double value);

  bool has_longitude_vel_rms() const;
  void clear_longitude_vel_rms();
  double longitude_vel_rms() const;
  void set_longitude_vel_rms(double value);

  bool has_dynprop() const;
  void clear_dynprop();
  uint32_t dynprop() const;
  void set_dynprop(uint32_t value);

  bool has_meas_state() const;
  void clear_meas_state();
  uint32_t meas_state() const;
  void set_meas_state(uint32_t value);

  bool has_lateral_vel_rms() const;
  void clear_lateral_vel_rms();
  double lateral_vel_rms() const;
  void set_lateral_vel_rms(double value);

  bool has_probexist() const;
  void clear_probexist();
  double probexist() const;
  void set_probexist(double value);

  bool has_longitude_accel() const;
  void clear_longitude_accel();
  double longitude_accel() const;
  void set_longitude_accel(double value);

  bool has_lateral_accel() const;
  void clear_lateral_accel();
  double lateral_accel() const;
  void set_lateral_accel(double value);

  bool has_oritation_angle() const;
  void clear_oritation_angle();
  double oritation_angle() const;
  void set_oritation_angle(double value);

  bool has_longitude_accel_rms() const;
  void clear_longitude_accel_rms();
  double longitude_accel_rms() const;
  void set_longitude_accel_rms(double value);

  bool has_lateral_accel_rms() const;
  void clear_lateral_accel_rms();
  double lateral_accel_rms() const;
  void set_lateral_accel_rms(double value);

  bool has_oritation_angle_rms() const;
  void clear_oritation_angle_rms();
  double oritation_angle_rms() const;
  void set_oritation_angle_rms(double value);

  bool has_length() const;
  void clear_length();
  double length() const;
  void set_length(double value);

  bool has_width() const;
  void clear_width();
  double width() const;
  void set_width(double value);

  bool has_obstacle_class() const;
  void clear_obstacle_class();
  uint32_t obstacle_class() const;
  void set_obstacle_class(uint32_t value);

 private:
  uint32_t has_bits_;
  bool clusterortrack_;
  uint32_t obstacle_id_;
  double longitude_dist_;
  double lateral_dist_;
  double longitude_vel_;
  double lateral_vel_;
  double rcs_;
  double longitude_dist_rms_;
  double lateral_dist_rms_;
  double longitude_vel_rms_;
  uint32_t dynprop_;
  uint32_t meas_state_;
  double lateral_vel_rms_;
  double probexist_;
  double longitude_accel_;
  double lateral_accel_;
  double oritation_angle_;
  double longitude_accel_rms_;
  double lateral_accel_rms_;
  double oritation_angle_rms_;
  double length_;
  double width_;
  uint32_t obstacle_class_;
};
// -------------------------------------------------------------------

class ContiRadar 
{
 public:
  ContiRadar();
  virtual ~ContiRadar();
  ContiRadar(const ContiRadar& from);
  inline ContiRadar& operator=(const ContiRadar& from) {
      has_bits_ = from.has_bits_;
      contiobs_.clear();
      contiobs_.insert(contiobs_.begin(), from.contiobs_.cbegin(), from.contiobs_.cend());
      if(from.radar_state_ == nullptr ) {
        radar_state_ = nullptr;
      } else {
        radar_state_ = new drivers::RadarState_201(*from.radar_state_);
      }
      if(from.cluster_list_status_ == nullptr ) {
        cluster_list_status_ = nullptr;
      } else {
        cluster_list_status_ = new drivers::ClusterListStatus_600(*from.cluster_list_status_);
      }
      if(from.object_list_status_ == nullptr ) {
        object_list_status_ = nullptr;
      } else {
        object_list_status_ = new drivers::ObjectListStatus_60A(*from.object_list_status_);
      }
      return *this;
  }
  public:

  int contiobs_size() const;
  void clear_contiobs();
  ::drivers::ContiRadarObs* mutable_contiobs(int index);
  ::std::vector< ::drivers::ContiRadarObs >* mutable_contiobs();
  const ::drivers::ContiRadarObs& contiobs(int index) const;
  ::drivers::ContiRadarObs* add_contiobs();
  const ::std::vector< ::drivers::ContiRadarObs >& contiobs() const;

  bool has_radar_state() const;
  void clear_radar_state();
  const ::drivers::RadarState_201& radar_state() const;
  ::drivers::RadarState_201* release_radar_state();
  ::drivers::RadarState_201* mutable_radar_state();
  void set_allocated_radar_state(::drivers::RadarState_201* radar_state);

  bool has_cluster_list_status() const;
  void clear_cluster_list_status();
  const ::drivers::ClusterListStatus_600& cluster_list_status() const;
  ::drivers::ClusterListStatus_600* release_cluster_list_status();
  ::drivers::ClusterListStatus_600* mutable_cluster_list_status();
  void set_allocated_cluster_list_status(::drivers::ClusterListStatus_600* cluster_list_status);

  bool has_object_list_status() const;
  void clear_object_list_status();
  const ::drivers::ObjectListStatus_60A& object_list_status() const;
  ::drivers::ObjectListStatus_60A* release_object_list_status();
  ::drivers::ObjectListStatus_60A* mutable_object_list_status();
  void set_allocated_object_list_status(::drivers::ObjectListStatus_60A* object_list_status);

  // bool has_header() const;
  // void clear_header();
  // ::PROTOBUF_NAMESPACE_ID::uint64 header() const;
  // void set_header(::PROTOBUF_NAMESPACE_ID::uint64 value);

 private:
  uint32_t has_bits_;
  std::vector<drivers::ContiRadarObs> contiobs_;
  drivers::RadarState_201* radar_state_;
  drivers::ClusterListStatus_600* cluster_list_status_;
  drivers::ObjectListStatus_60A* object_list_status_;
};

}  // namespace drivers

#endif  // v
