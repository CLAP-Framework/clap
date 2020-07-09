
#include <cstring>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>
#include <array>
#include "continental_radar/proto/conti_radar.h"


namespace drivers {
#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ClusterListStatus_600
ClusterListStatus_600::ClusterListStatus_600() :
      has_bits_(0),
      near_(0),
      far_(0),
      interface_version_(0),
      meas_counter_(0)
{}
ClusterListStatus_600::~ClusterListStatus_600()
{}
ClusterListStatus_600::ClusterListStatus_600(const ClusterListStatus_600& from) :
      has_bits_(from.has_bits_),
      near_(from.near_),
      far_(from.far_),
      interface_version_(from.interface_version_),
      meas_counter_(from.meas_counter_)
{}

bool ClusterListStatus_600::has_near() const {
  return (has_bits_ & 0x00000001u) != 0;
}
void ClusterListStatus_600::clear_near() {
  near_ = 0;
  has_bits_ &= ~0x00000001u;
}
uint32_t ClusterListStatus_600::near() const {
  return near_;
}
void ClusterListStatus_600::set_near(uint32_t value) {
  has_bits_ |= 0x00000001u;
  near_ = value;
}

bool ClusterListStatus_600::has_far() const {
  return (has_bits_ & 0x00000002u) != 0;
}
void ClusterListStatus_600::clear_far() {
  far_ = 0;
  has_bits_ &= ~0x00000002u;
}
uint32_t ClusterListStatus_600::far() const {
  return far_;
}
void ClusterListStatus_600::set_far(uint32_t value) {
  has_bits_ |= 0x00000002u;
  far_ = value;
}

bool ClusterListStatus_600::has_meas_counter() const {
  return (has_bits_ & 0x00000008u) != 0;
}
void ClusterListStatus_600::clear_meas_counter() {
  meas_counter_ = -1;
  has_bits_ &= ~0x00000008u;
}
uint32_t ClusterListStatus_600::meas_counter() const {
  return meas_counter_;
}
void ClusterListStatus_600::set_meas_counter(uint32_t value) {
  has_bits_ |= 0x00000008u;
  meas_counter_ = value;
}

bool ClusterListStatus_600::has_interface_version() const {
  return (has_bits_ & 0x00000004u) != 0;
}
void ClusterListStatus_600::clear_interface_version() {
  interface_version_ = 0;
  has_bits_ &= ~0x00000004u;
}
uint32_t ClusterListStatus_600::interface_version() const {
  return interface_version_;
}
void ClusterListStatus_600::set_interface_version(uint32_t value) {
  has_bits_ |= 0x00000004u;
  interface_version_ = value;
}

// ObjectListStatus_60A
ObjectListStatus_60A::ObjectListStatus_60A() :
  has_bits_(0),
  nof_objects_(0),
  interface_version_(0),
  meas_counter_(0)
{}
ObjectListStatus_60A::~ObjectListStatus_60A()
{}
ObjectListStatus_60A::ObjectListStatus_60A(const ObjectListStatus_60A& from) :
  has_bits_(from.has_bits_),
  nof_objects_(from.nof_objects_),
  interface_version_(from.interface_version_),
  meas_counter_(from.meas_counter_)
{}
bool ObjectListStatus_60A::has_nof_objects() const {
  return (has_bits_ & 0x00000001u) != 0;
}
void ObjectListStatus_60A::clear_nof_objects() {
  nof_objects_ = 0;
  has_bits_ &= ~0x00000001u;
}
uint32_t ObjectListStatus_60A::nof_objects() const {
  return nof_objects_;
}
void ObjectListStatus_60A::set_nof_objects(uint32_t value) {
  has_bits_ |= 0x00000001u;
  nof_objects_ = value;
}

bool ObjectListStatus_60A::has_meas_counter() const {
  return (has_bits_ & 0x00000004u) != 0;
}
void ObjectListStatus_60A::clear_meas_counter() {
  meas_counter_ = -1;
  has_bits_ &= ~0x00000004u;
}
uint32_t ObjectListStatus_60A::meas_counter() const {
  return meas_counter_;
}
void ObjectListStatus_60A::set_meas_counter(uint32_t value) {
  has_bits_ |= 0x00000004u;
  meas_counter_ = value;
}

bool ObjectListStatus_60A::has_interface_version() const {
  return (has_bits_ & 0x00000002u) != 0;
}
void ObjectListStatus_60A::clear_interface_version() {
  interface_version_ = 0;
  has_bits_ &= ~0x00000002u;
}
uint32_t ObjectListStatus_60A::interface_version() const {
  return interface_version_;
}
void ObjectListStatus_60A::set_interface_version(uint32_t value) {
  has_bits_ |= 0x00000002u;
  interface_version_ = value;
}

// RadarState_201
RadarState_201::RadarState_201() :
    has_bits_(0),
    max_distance_(0),
    radar_power_(0),
    output_type_(0),
    rcs_threshold_(0),
    send_quality_(0),
    send_ext_info_(0)
{}
RadarState_201::~RadarState_201()
{}
RadarState_201::RadarState_201(const RadarState_201& from) :
    has_bits_(from.has_bits_),
    max_distance_(from.max_distance_),
    radar_power_(from.radar_power_),
    output_type_(from.output_type_),
    rcs_threshold_(from.rcs_threshold_),
    send_quality_(from.send_quality_),
    send_ext_info_(from.send_ext_info_)
{}

bool RadarState_201::has_max_distance() const {
  return (has_bits_ & 0x00000001u) != 0;
}
void RadarState_201::clear_max_distance() {
  max_distance_ = 0u;
  has_bits_ &= ~0x00000001u;
}
uint32_t RadarState_201::max_distance() const {
  return max_distance_;
}
void RadarState_201::set_max_distance(uint32_t value) {
  has_bits_ |= 0x00000001u;
  max_distance_ = value;
}

bool RadarState_201::has_radar_power() const {
  return (has_bits_ & 0x00000002u) != 0;
}
void RadarState_201::clear_radar_power() {
  radar_power_ = 0u;
  has_bits_ &= ~0x00000002u;
}
uint32_t RadarState_201::radar_power() const {
  return radar_power_;
}
void RadarState_201::set_radar_power(uint32_t value) {
  has_bits_ |= 0x00000002u;
  radar_power_ = value;
}

bool RadarState_201::has_output_type() const {
  return (has_bits_ & 0x00000004u) != 0;
}
void RadarState_201::clear_output_type() {
  output_type_ = 0u;
  has_bits_ &= ~0x00000004u;
}
uint32_t RadarState_201::output_type() const {
  return output_type_;
}
void RadarState_201::set_output_type(uint32_t value) {
  has_bits_ |= 0x00000004u;
  output_type_ = value;
}

bool RadarState_201::has_rcs_threshold() const {
  return (has_bits_ & 0x00000008u) != 0;
}
void RadarState_201::clear_rcs_threshold() {
  rcs_threshold_ = 0u;
  has_bits_ &= ~0x00000008u;
}
uint32_t RadarState_201::rcs_threshold() const {
  return rcs_threshold_;
}
void RadarState_201::set_rcs_threshold(uint32_t value) {
  has_bits_ |= 0x00000008u;
  rcs_threshold_ = value;
}

bool RadarState_201::has_send_quality() const {
  return (has_bits_ & 0x00000010u) != 0;
}
void RadarState_201::clear_send_quality() {
  send_quality_ = false;
  has_bits_ &= ~0x00000010u;
}
bool RadarState_201::send_quality() const {
  return send_quality_;
}
void RadarState_201::set_send_quality(bool value) {
  has_bits_ |= 0x00000010u;
  send_quality_ = value;
}

bool RadarState_201::has_send_ext_info() const {
  return (has_bits_ & 0x00000020u) != 0;
}
void RadarState_201::clear_send_ext_info() {
  send_ext_info_ = false;
  has_bits_ &= ~0x00000020u;
}
bool RadarState_201::send_ext_info() const {
  return send_ext_info_;
}
void RadarState_201::set_send_ext_info(bool value) {
  has_bits_ |= 0x00000020u;
  send_ext_info_ = value;
}


// // optional uint64 header = 1;
// bool ContiRadarObs::has_header() const {
//   return (has_bits_ & 0x00000001u) != 0;
// }
// void ContiRadarObs::clear_header() {
//   header_ = PROTOBUF_ULONGLONG(0);
//   has_bits_ &= ~0x00000001u;
// }
// ::PROTOBUF_NAMESPACE_ID::uint64 ContiRadarObs::header() const {
//   // @@protoc_insertion_point(field_get:drivers.ContiRadarObs.header)
//   return header_;
// }
// void ContiRadarObs::set_header(::PROTOBUF_NAMESPACE_ID::uint64 value) {
//   has_bits_ |= 0x00000001u;
//   header_ = value;
//   // @@protoc_insertion_point(field_set:drivers.ContiRadarObs.header)
// }
ContiRadarObs::ContiRadarObs() :
    has_bits_(0),
    clusterortrack_(0),
    obstacle_id_(0),
    longitude_dist_(0),
    lateral_dist_(0),
    longitude_vel_(0),
    lateral_vel_(0),
    rcs_(0),
    longitude_dist_rms_(0),
    lateral_dist_rms_(0),
    longitude_vel_rms_(0),
    dynprop_(0),
    meas_state_(0),
    lateral_vel_rms_(0),
    probexist_(0),
    longitude_accel_(0),
    lateral_accel_(0),
    oritation_angle_(0),
    longitude_accel_rms_(0),
    lateral_accel_rms_(0),
    oritation_angle_rms_(0),
    length_(0),
    width_(0),
    obstacle_class_(0)
{}
ContiRadarObs::~ContiRadarObs()
{}
ContiRadarObs::ContiRadarObs(const ContiRadarObs& from) :
    has_bits_(from.has_bits_),
    clusterortrack_(from.clusterortrack_),
    obstacle_id_(from.obstacle_id_),
    longitude_dist_(from.longitude_dist_),
    lateral_dist_(from.lateral_dist_),
    longitude_vel_(from.longitude_vel_),
    lateral_vel_(from.lateral_vel_),
    rcs_(from.rcs_),
    longitude_dist_rms_(from.longitude_dist_rms_),
    lateral_dist_rms_(from.lateral_dist_rms_),
    longitude_vel_rms_(from.longitude_vel_rms_),
    dynprop_(from.dynprop_),
    meas_state_(from.meas_state_),
    lateral_vel_rms_(from.lateral_vel_rms_),
    probexist_(from.probexist_),
    longitude_accel_(from.longitude_accel_),
    lateral_accel_(from.lateral_accel_),
    oritation_angle_(from.oritation_angle_),
    longitude_accel_rms_(from.longitude_accel_rms_),
    lateral_accel_rms_(from.lateral_accel_rms_),
    oritation_angle_rms_(from.oritation_angle_rms_),
    length_(from.length_),
    width_(from.width_),
    obstacle_class_(from.obstacle_class_)
{}

bool ContiRadarObs::has_clusterortrack() const {
  return (has_bits_ & 0x00000002u) != 0;
}
void ContiRadarObs::clear_clusterortrack() {
  clusterortrack_ = false;
  has_bits_ &= ~0x00000002u;
}
bool ContiRadarObs::clusterortrack() const {
  return clusterortrack_;
}
void ContiRadarObs::set_clusterortrack(bool value) {
  has_bits_ |= 0x00000002u;
  clusterortrack_ = value;
}

bool ContiRadarObs::has_obstacle_id() const {
  return (has_bits_ & 0x00000004u) != 0;
}
void ContiRadarObs::clear_obstacle_id() {
  obstacle_id_ = 0;
  has_bits_ &= ~0x00000004u;
}
uint32_t ContiRadarObs::obstacle_id() const {
  return obstacle_id_;
}
void ContiRadarObs::set_obstacle_id(uint32_t value) {
  has_bits_ |= 0x00000004u;
  obstacle_id_ = value;
}

bool ContiRadarObs::has_longitude_dist() const {
  return (has_bits_ & 0x00000008u) != 0;
}
void ContiRadarObs::clear_longitude_dist() {
  longitude_dist_ = 0;
  has_bits_ &= ~0x00000008u;
}
double ContiRadarObs::longitude_dist() const {
  return longitude_dist_;
}
void ContiRadarObs::set_longitude_dist(double value) {
  has_bits_ |= 0x00000008u;
  longitude_dist_ = value;
}

bool ContiRadarObs::has_lateral_dist() const {
  return (has_bits_ & 0x00000010u) != 0;
}
void ContiRadarObs::clear_lateral_dist() {
  lateral_dist_ = 0;
  has_bits_ &= ~0x00000010u;
}
double ContiRadarObs::lateral_dist() const {
  return lateral_dist_;
}
void ContiRadarObs::set_lateral_dist(double value) {
  has_bits_ |= 0x00000010u;
  lateral_dist_ = value;
}

bool ContiRadarObs::has_longitude_vel() const {
  return (has_bits_ & 0x00000020u) != 0;
}
void ContiRadarObs::clear_longitude_vel() {
  longitude_vel_ = 0;
  has_bits_ &= ~0x00000020u;
}
double ContiRadarObs::longitude_vel() const {
  return longitude_vel_;
}
void ContiRadarObs::set_longitude_vel(double value) {
  has_bits_ |= 0x00000020u;
  longitude_vel_ = value;
}

bool ContiRadarObs::has_lateral_vel() const {
  return (has_bits_ & 0x00000040u) != 0;
}
void ContiRadarObs::clear_lateral_vel() {
  lateral_vel_ = 0;
  has_bits_ &= ~0x00000040u;
}
double ContiRadarObs::lateral_vel() const {
  return lateral_vel_;
}
void ContiRadarObs::set_lateral_vel(double value) {
  has_bits_ |= 0x00000040u;
  lateral_vel_ = value;
}

bool ContiRadarObs::has_rcs() const {
  return (has_bits_ & 0x00000080u) != 0;
}
void ContiRadarObs::clear_rcs() {
  rcs_ = 0;
  has_bits_ &= ~0x00000080u;
}
double ContiRadarObs::rcs() const {
  return rcs_;
}
void ContiRadarObs::set_rcs(double value) {
  has_bits_ |= 0x00000080u;
  rcs_ = value;
}

bool ContiRadarObs::has_dynprop() const {
  return (has_bits_ & 0x00000800u) != 0;
}
void ContiRadarObs::clear_dynprop() {
  dynprop_ = 0;
  has_bits_ &= ~0x00000800u;
}
uint32_t ContiRadarObs::dynprop() const {
  return dynprop_;
}
void ContiRadarObs::set_dynprop(uint32_t value) {
  has_bits_ |= 0x00000800u;
  dynprop_ = value;
}

bool ContiRadarObs::has_longitude_dist_rms() const {
  return (has_bits_ & 0x00000100u) != 0;
}
void ContiRadarObs::clear_longitude_dist_rms() {
  longitude_dist_rms_ = 0;
  has_bits_ &= ~0x00000100u;
}
double ContiRadarObs::longitude_dist_rms() const {
  return longitude_dist_rms_;
}
void ContiRadarObs::set_longitude_dist_rms(double value) {
  has_bits_ |= 0x00000100u;
  longitude_dist_rms_ = value;
}

bool ContiRadarObs::has_lateral_dist_rms() const {
  return (has_bits_ & 0x00000200u) != 0;
}
void ContiRadarObs::clear_lateral_dist_rms() {
  lateral_dist_rms_ = 0;
  has_bits_ &= ~0x00000200u;
}
double ContiRadarObs::lateral_dist_rms() const {
  return lateral_dist_rms_;
}
void ContiRadarObs::set_lateral_dist_rms(double value) {
  has_bits_ |= 0x00000200u;
  lateral_dist_rms_ = value;
}

bool ContiRadarObs::has_longitude_vel_rms() const {
  return (has_bits_ & 0x00000400u) != 0;
}
void ContiRadarObs::clear_longitude_vel_rms() {
  longitude_vel_rms_ = 0;
  has_bits_ &= ~0x00000400u;
}
double ContiRadarObs::longitude_vel_rms() const {
  return longitude_vel_rms_;
}
void ContiRadarObs::set_longitude_vel_rms(double value) {
  has_bits_ |= 0x00000400u;
  longitude_vel_rms_ = value;
}

bool ContiRadarObs::has_lateral_vel_rms() const {
  return (has_bits_ & 0x00002000u) != 0;
}
void ContiRadarObs::clear_lateral_vel_rms() {
  lateral_vel_rms_ = 0;
  has_bits_ &= ~0x00002000u;
}
double ContiRadarObs::lateral_vel_rms() const {
  return lateral_vel_rms_;
}
void ContiRadarObs::set_lateral_vel_rms(double value) {
  has_bits_ |= 0x00002000u;
  lateral_vel_rms_ = value;
}

bool ContiRadarObs::has_probexist() const {
  return (has_bits_ & 0x00004000u) != 0;
}
void ContiRadarObs::clear_probexist() {
  probexist_ = 0;
  has_bits_ &= ~0x00004000u;
}
double ContiRadarObs::probexist() const {
  return probexist_;
}
void ContiRadarObs::set_probexist(double value) {
  has_bits_ |= 0x00004000u;
  probexist_ = value;
}

bool ContiRadarObs::has_meas_state() const {
  return (has_bits_ & 0x00001000u) != 0;
}
void ContiRadarObs::clear_meas_state() {
  meas_state_ = 0;
  has_bits_ &= ~0x00001000u;
}
uint32_t ContiRadarObs::meas_state() const {
  return meas_state_;
}
void ContiRadarObs::set_meas_state(uint32_t value) {
  has_bits_ |= 0x00001000u;
  meas_state_ = value;
}

bool ContiRadarObs::has_longitude_accel() const {
  return (has_bits_ & 0x00008000u) != 0;
}
void ContiRadarObs::clear_longitude_accel() {
  longitude_accel_ = 0;
  has_bits_ &= ~0x00008000u;
}
double ContiRadarObs::longitude_accel() const {
  return longitude_accel_;
}
void ContiRadarObs::set_longitude_accel(double value) {
  has_bits_ |= 0x00008000u;
  longitude_accel_ = value;
}

bool ContiRadarObs::has_lateral_accel() const {
  return (has_bits_ & 0x00010000u) != 0;
}
void ContiRadarObs::clear_lateral_accel() {
  lateral_accel_ = 0;
  has_bits_ &= ~0x00010000u;
}
double ContiRadarObs::lateral_accel() const {
  return lateral_accel_;
}
void ContiRadarObs::set_lateral_accel(double value) {
  has_bits_ |= 0x00010000u;
  lateral_accel_ = value;
}

bool ContiRadarObs::has_oritation_angle() const {
  return (has_bits_ & 0x00020000u) != 0;
}
void ContiRadarObs::clear_oritation_angle() {
  oritation_angle_ = 0;
  has_bits_ &= ~0x00020000u;
}
double ContiRadarObs::oritation_angle() const {
  return oritation_angle_;
}
void ContiRadarObs::set_oritation_angle(double value) {
  has_bits_ |= 0x00020000u;
  oritation_angle_ = value;
}

bool ContiRadarObs::has_longitude_accel_rms() const {
  return (has_bits_ & 0x00040000u) != 0;
}
void ContiRadarObs::clear_longitude_accel_rms() {
  longitude_accel_rms_ = 0;
  has_bits_ &= ~0x00040000u;
}
double ContiRadarObs::longitude_accel_rms() const {
  return longitude_accel_rms_;
}
void ContiRadarObs::set_longitude_accel_rms(double value) {
  has_bits_ |= 0x00040000u;
  longitude_accel_rms_ = value;
}

bool ContiRadarObs::has_lateral_accel_rms() const {
  return (has_bits_ & 0x00080000u) != 0;
}
void ContiRadarObs::clear_lateral_accel_rms() {
  lateral_accel_rms_ = 0;
  has_bits_ &= ~0x00080000u;
}
double ContiRadarObs::lateral_accel_rms() const {
  return lateral_accel_rms_;
}
void ContiRadarObs::set_lateral_accel_rms(double value) {
  has_bits_ |= 0x00080000u;
  lateral_accel_rms_ = value;
}

bool ContiRadarObs::has_oritation_angle_rms() const {
  return (has_bits_ & 0x00100000u) != 0;
}
void ContiRadarObs::clear_oritation_angle_rms() {
  oritation_angle_rms_ = 0;
  has_bits_ &= ~0x00100000u;
}
double ContiRadarObs::oritation_angle_rms() const {
  return oritation_angle_rms_;
}
void ContiRadarObs::set_oritation_angle_rms(double value) {
  has_bits_ |= 0x00100000u;
  oritation_angle_rms_ = value;
}

bool ContiRadarObs::has_length() const {
  return (has_bits_ & 0x00200000u) != 0;
}
void ContiRadarObs::clear_length() {
  length_ = 0;
  has_bits_ &= ~0x00200000u;
}
double ContiRadarObs::length() const {
  return length_;
}
void ContiRadarObs::set_length(double value) {
  has_bits_ |= 0x00200000u;
  length_ = value;
}

bool ContiRadarObs::has_width() const {
  return (has_bits_ & 0x00400000u) != 0;
}
void ContiRadarObs::clear_width() {
  width_ = 0;
  has_bits_ &= ~0x00400000u;
}
double ContiRadarObs::width() const {
  return width_;
}
void ContiRadarObs::set_width(double value) {
  has_bits_ |= 0x00400000u;
  width_ = value;
}

bool ContiRadarObs::has_obstacle_class() const {
  return (has_bits_ & 0x00800000u) != 0;
}
void ContiRadarObs::clear_obstacle_class() {
  obstacle_class_ = 0;
  has_bits_ &= ~0x00800000u;
}
uint32_t ContiRadarObs::obstacle_class() const {
  return obstacle_class_;
}
void ContiRadarObs::set_obstacle_class(uint32_t value) {
  has_bits_ |= 0x00800000u;
  obstacle_class_ = value;
}

// ContiRadar

// // optional uint64 header = 1;
// bool ContiRadar::has_header() const {
//   return (has_bits_ & 0x00000008u) != 0;
// }
// void ContiRadar::clear_header() {
//   header_ = PROTOBUF_ULONGLONG(0);
//   has_bits_ &= ~0x00000008u;
// }
// ::PROTOBUF_NAMESPACE_ID::uint64 ContiRadar::header() const {
//   // @@protoc_insertion_point(field_get:drivers.ContiRadar.header)
//   return header_;
// }
// void ContiRadar::set_header(::PROTOBUF_NAMESPACE_ID::uint64 value) {
//   has_bits_ |= 0x00000008u;
//   header_ = value;
//   // @@protoc_insertion_point(field_set:drivers.ContiRadar.header)
// }
ContiRadar::ContiRadar() :
  has_bits_(0),
  radar_state_(nullptr),
  cluster_list_status_(nullptr),
  object_list_status_(nullptr)
{}
ContiRadar::ContiRadar(const ContiRadar& from)
{
    has_bits_ = from.has_bits_;
    contiobs_.clear();
    contiobs_.insert(contiobs_.begin(), from.contiobs_.cbegin(),from.contiobs_.cend());
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
}
ContiRadar::~ContiRadar()
{}
int ContiRadar::contiobs_size() const {
  return contiobs_.size();
}
void ContiRadar::clear_contiobs() {
  contiobs_.clear();
}
drivers::ContiRadarObs* ContiRadar::mutable_contiobs(int index) {
  return &contiobs_.at(index);
}
::std::vector< drivers::ContiRadarObs >*
ContiRadar::mutable_contiobs() {
  return &contiobs_;
}
const drivers::ContiRadarObs& ContiRadar::contiobs(int index) const {
  return contiobs_.at(index);
}
drivers::ContiRadarObs* ContiRadar::add_contiobs() {
  contiobs_.push_back(drivers::ContiRadarObs());
  return &contiobs_.at(contiobs_.size()-1);
}
const ::std::vector< drivers::ContiRadarObs >&
ContiRadar::contiobs() const {
  return contiobs_;
}

bool ContiRadar::has_radar_state() const {
  return (has_bits_ & 0x00000001u) != 0;
}
void ContiRadar::clear_radar_state() {
  if (radar_state_ != nullptr) delete radar_state_;
  has_bits_ &= ~0x00000001u;
}
const drivers::RadarState_201& ContiRadar::radar_state() const {
  const drivers::RadarState_201* p = radar_state_;
  return *p;
}
drivers::RadarState_201* ContiRadar::release_radar_state() {
  has_bits_ &= ~0x00000001u;
  drivers::RadarState_201* temp = radar_state_;
  radar_state_ = nullptr;
  return temp;
}
drivers::RadarState_201* ContiRadar::mutable_radar_state() {
  has_bits_ |= 0x00000001u;
  if (radar_state_ == nullptr) {
    auto* p = new drivers::RadarState_201;
    radar_state_ = p;
  }
  return radar_state_;
}
void ContiRadar::set_allocated_radar_state(drivers::RadarState_201* radar_state) {
  if (radar_state_!= nullptr) {
    delete radar_state_;
  }
  if (radar_state) {
    has_bits_ |= 0x00000001u;
  } else {
    has_bits_ &= ~0x00000001u;
  }
  radar_state_ = radar_state;
}

bool ContiRadar::has_cluster_list_status() const {
  return (has_bits_ & 0x00000002u) != 0;
}
void ContiRadar::clear_cluster_list_status() {
  if (cluster_list_status_ != nullptr) delete cluster_list_status_;
  has_bits_ &= ~0x00000002u;
}
const drivers::ClusterListStatus_600& ContiRadar::cluster_list_status() const {
  const drivers::ClusterListStatus_600* p = cluster_list_status_;
    return *p;
}
drivers::ClusterListStatus_600* ContiRadar::release_cluster_list_status() {
  has_bits_ &= ~0x00000002u;
  drivers::ClusterListStatus_600* temp = cluster_list_status_;
  cluster_list_status_ = nullptr;
  return temp;
}
drivers::ClusterListStatus_600* ContiRadar::mutable_cluster_list_status() {
  has_bits_ |= 0x00000002u;
  if (cluster_list_status_ == nullptr) {
    auto* p = new drivers::ClusterListStatus_600;
    cluster_list_status_ = p;
  }
  return cluster_list_status_;
}
void ContiRadar::set_allocated_cluster_list_status(drivers::ClusterListStatus_600* cluster_list_status) {
  if (cluster_list_status_ != nullptr) {
    delete cluster_list_status_;
  }
  if (cluster_list_status) {
    has_bits_ |= 0x00000002u;
  } else {
    has_bits_ &= ~0x00000002u;
  }
  cluster_list_status_ = cluster_list_status;
}

bool ContiRadar::has_object_list_status() const {
  return (has_bits_ & 0x00000004u) != 0;
}
void ContiRadar::clear_object_list_status() {
  if (object_list_status_ != nullptr) delete object_list_status_;
  has_bits_ &= ~0x00000004u;
}
const drivers::ObjectListStatus_60A& ContiRadar::object_list_status() const {
  const drivers::ObjectListStatus_60A* p = object_list_status_;
    return *p;
}
drivers::ObjectListStatus_60A* ContiRadar::release_object_list_status() {
  has_bits_ &= ~0x00000004u;
  drivers::ObjectListStatus_60A* temp = object_list_status_;
  object_list_status_ = nullptr;
  return temp;
}
drivers::ObjectListStatus_60A* ContiRadar::mutable_object_list_status() {
  has_bits_ |= 0x00000004u;
  if (object_list_status_ == nullptr) {
    auto* p = new drivers::ObjectListStatus_60A;
    object_list_status_ = p;
  }
  return object_list_status_;
}
void ContiRadar::set_allocated_object_list_status(drivers::ObjectListStatus_60A* object_list_status) {

  if (object_list_status_ != nullptr) {
    delete object_list_status_;
  }
  if (object_list_status) {
    has_bits_ |= 0x00000004u;
  } else {
    has_bits_ &= ~0x00000004u;
  }
  object_list_status_ = object_list_status;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

}  // namespace drivers

