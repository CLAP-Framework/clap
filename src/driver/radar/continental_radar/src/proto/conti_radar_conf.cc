#include <assert.h>
#include <cstring>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>
#include <array>
#include "continental_radar/proto/conti_radar_conf.h"

// namespace drivers {
// namespace conti_radar {

// // class ContiRadarConfDefaultTypeInternal;
// // extern ContiRadarConfDefaultTypeInternal _ContiRadarConf_default_instance_;
// // class RadarConfDefaultTypeInternal;
// // extern RadarConfDefaultTypeInternal _RadarConf_default_instance_;

// }  // namespace conti_radar
// }  // namespace drivers

namespace drivers {
namespace conti_radar {

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// RadarConf
RadarConf::RadarConf() : 
  has_bits_(0),
  max_distance_valid_(0),
  sensor_id_valid_(0),
  radar_power_valid_(0),
  sort_index_valid_(0),
  ctrl_relay_valid_(0),
  sensor_id_(0),
  radar_power_(0),
  ctrl_relay_(0),
  sort_index_(0),
  rcs_threshold_(0),
  store_in_nvm_(0),
  output_type_valid_(0),
  send_quality_valid_(0),
  send_ext_info_valid_(0),
  store_in_nvm_valid_(0),
  max_distance_(0),
  output_type_(0),
  rcs_threshold_valid_(0),
  send_ext_info_(0),
  send_quality_(0)
{}
RadarConf::~RadarConf()
{}
RadarConf::RadarConf(const RadarConf& from) :
  has_bits_(from.has_bits_),
  max_distance_valid_(from.max_distance_valid_),
  sensor_id_valid_(from.sensor_id_valid_),
  radar_power_valid_(from.radar_power_valid_),
  sort_index_valid_(from.sort_index_valid_),
  ctrl_relay_valid_(from.ctrl_relay_valid_),
  sensor_id_(from.sensor_id_),
  radar_power_(from.radar_power_),
  ctrl_relay_(from.ctrl_relay_),
  sort_index_(from.sort_index_),
  rcs_threshold_(from.rcs_threshold_),
  store_in_nvm_(from.store_in_nvm_),
  output_type_valid_(from.output_type_valid_),
  send_quality_valid_(from.send_quality_valid_),
  send_ext_info_valid_(from.send_ext_info_valid_),
  store_in_nvm_valid_(from.store_in_nvm_valid_),
  max_distance_(from.max_distance_),
  output_type_(from.output_type_),
  rcs_threshold_valid_(from.rcs_threshold_valid_),
  send_ext_info_(from.send_ext_info_),
  send_quality_(from.send_quality_)
{}

bool RadarConf::has_max_distance_valid() const {
  return (has_bits_ & 0x00000001u) != 0;
}
void RadarConf::clear_max_distance_valid() {
  max_distance_valid_ = false;
  has_bits_ &= ~0x00000001u;
}
bool RadarConf::max_distance_valid() const {
  return max_distance_valid_;
}
void RadarConf::set_max_distance_valid(bool value) {
  has_bits_ |= 0x00000001u;
  max_distance_valid_ = value;
}

bool RadarConf::has_sensor_id_valid() const {
  return (has_bits_ & 0x00000002u) != 0;
}
void RadarConf::clear_sensor_id_valid() {
  sensor_id_valid_ = false;
  has_bits_ &= ~0x00000002u;
}
bool RadarConf::sensor_id_valid() const {
  return sensor_id_valid_;
}
void RadarConf::set_sensor_id_valid(bool value) {
  has_bits_ |= 0x00000002u;
  sensor_id_valid_ = value;
}

bool RadarConf::has_radar_power_valid() const {
  return (has_bits_ & 0x00000004u) != 0;
}
void RadarConf::clear_radar_power_valid() {
  radar_power_valid_ = false;
  has_bits_ &= ~0x00000004u;
}
bool RadarConf::radar_power_valid() const {
  return radar_power_valid_;
}
void RadarConf::set_radar_power_valid(bool value) {
  has_bits_ |= 0x00000004u;
  radar_power_valid_ = value;
}

bool RadarConf::has_output_type_valid() const {
  return (has_bits_ & 0x00001000u) != 0;
}
void RadarConf::clear_output_type_valid() {
  output_type_valid_ = true;
  has_bits_ &= ~0x00001000u;
}
bool RadarConf::output_type_valid() const {
  return output_type_valid_;
}
void RadarConf::set_output_type_valid(bool value) {
  has_bits_ |= 0x00001000u;
  output_type_valid_ = value;
}

bool RadarConf::has_send_quality_valid() const {
  return (has_bits_ & 0x00002000u) != 0;
}
void RadarConf::clear_send_quality_valid() {
  send_quality_valid_ = true;
  has_bits_ &= ~0x00002000u;
}
bool RadarConf::send_quality_valid() const {
  return send_quality_valid_;
}
void RadarConf::set_send_quality_valid(bool value) {
  has_bits_ |= 0x00002000u;
  send_quality_valid_ = value;
}

bool RadarConf::has_send_ext_info_valid() const {
  return (has_bits_ & 0x00004000u) != 0;
}
void RadarConf::clear_send_ext_info_valid() {
  send_ext_info_valid_ = true;
  has_bits_ &= ~0x00004000u;
}
bool RadarConf::send_ext_info_valid() const {
  return send_ext_info_valid_;
}
void RadarConf::set_send_ext_info_valid(bool value) {
  has_bits_ |= 0x00004000u;
  send_ext_info_valid_ = value;
}

bool RadarConf::has_sort_index_valid() const {
  return (has_bits_ & 0x00000008u) != 0;
}
void RadarConf::clear_sort_index_valid() {
  sort_index_valid_ = false;
  has_bits_ &= ~0x00000008u;
}
bool RadarConf::sort_index_valid() const {
  return sort_index_valid_;
}
void RadarConf::set_sort_index_valid(bool value) {
  has_bits_ |= 0x00000008u;
  sort_index_valid_ = value;
}

bool RadarConf::has_store_in_nvm_valid() const {
  return (has_bits_ & 0x00008000u) != 0;
}
void RadarConf::clear_store_in_nvm_valid() {
  store_in_nvm_valid_ = true;
  has_bits_ &= ~0x00008000u;
}
bool RadarConf::store_in_nvm_valid() const {
  return store_in_nvm_valid_;
}
void RadarConf::set_store_in_nvm_valid(bool value) {
  has_bits_ |= 0x00008000u;
  store_in_nvm_valid_ = value;
}

bool RadarConf::has_ctrl_relay_valid() const {
  return (has_bits_ & 0x00000010u) != 0;
}
void RadarConf::clear_ctrl_relay_valid() {
  ctrl_relay_valid_ = false;
  has_bits_ &= ~0x00000010u;
}
bool RadarConf::ctrl_relay_valid() const {
  return ctrl_relay_valid_;
}
void RadarConf::set_ctrl_relay_valid(bool value) {
  has_bits_ |= 0x00000010u;
  ctrl_relay_valid_ = value;
}

bool RadarConf::has_rcs_threshold_valid() const {
  return (has_bits_ & 0x00040000u) != 0;
}
void RadarConf::clear_rcs_threshold_valid() {
  rcs_threshold_valid_ = true;
  has_bits_ &= ~0x00040000u;
}
bool RadarConf::rcs_threshold_valid() const {
  return rcs_threshold_valid_;
}
void RadarConf::set_rcs_threshold_valid(bool value) {
  has_bits_ |= 0x00040000u;
  rcs_threshold_valid_ = value;
}

bool RadarConf::has_max_distance() const {
  return (has_bits_ & 0x00010000u) != 0;
}
void RadarConf::clear_max_distance() {
  max_distance_ = 248u;
  has_bits_ &= ~0x00010000u;
}
uint32_t RadarConf::max_distance() const {
  return max_distance_;
}
void RadarConf::set_max_distance(uint32_t value) {
  has_bits_ |= 0x00010000u;
  max_distance_ = value;
}

bool RadarConf::has_sensor_id() const {
  return (has_bits_ & 0x00000020u) != 0;
}
void RadarConf::clear_sensor_id() {
  sensor_id_ = 0u;
  has_bits_ &= ~0x00000020u;
}
uint32_t RadarConf::sensor_id() const {
  return sensor_id_;
}
void RadarConf::set_sensor_id(uint32_t value) {
  has_bits_ |= 0x00000020u;
  sensor_id_ = value;
}

bool RadarConf::has_output_type() const {
  return (has_bits_ & 0x00020000u) != 0;
}
void RadarConf::clear_output_type() {
  output_type_ = 1;
  has_bits_ &= ~0x00020000u;
}
::drivers::conti_radar::OutputType RadarConf::output_type() const {
  return static_cast< ::drivers::conti_radar::OutputType >(output_type_);
}
void RadarConf::set_output_type(::drivers::conti_radar::OutputType value) {
  has_bits_ |= 0x00020000u;
  output_type_ = value;
}

bool RadarConf::has_radar_power() const {
  return (has_bits_ & 0x00000040u) != 0;
}
void RadarConf::clear_radar_power() {
  radar_power_ = 0u;
  has_bits_ &= ~0x00000040u;
}
uint32_t RadarConf::radar_power() const {
  return radar_power_;
}
void RadarConf::set_radar_power(uint32_t value) {
  has_bits_ |= 0x00000040u;
  radar_power_ = value;
}

bool RadarConf::has_ctrl_relay() const {
  return (has_bits_ & 0x00000080u) != 0;
}
void RadarConf::clear_ctrl_relay() {
  ctrl_relay_ = 0u;
  has_bits_ &= ~0x00000080u;
}
uint32_t RadarConf::ctrl_relay() const {
  return ctrl_relay_;
}
void RadarConf::set_ctrl_relay(uint32_t value) {
  has_bits_ |= 0x00000080u;
  ctrl_relay_ = value;
}

bool RadarConf::has_send_ext_info() const {
  return (has_bits_ & 0x00080000u) != 0;
}
void RadarConf::clear_send_ext_info() {
  send_ext_info_ = true;
  has_bits_ &= ~0x00080000u;
}
bool RadarConf::send_ext_info() const {
  return send_ext_info_;
}
void RadarConf::set_send_ext_info(bool value) {
  has_bits_ |= 0x00080000u;
  send_ext_info_ = value;
}

bool RadarConf::has_send_quality() const {
  return (has_bits_ & 0x00100000u) != 0;
}
void RadarConf::clear_send_quality() {
  send_quality_ = true;
  has_bits_ &= ~0x00100000u;
}
bool RadarConf::send_quality() const {
  return send_quality_;
}
void RadarConf::set_send_quality(bool value) {
  has_bits_ |= 0x00100000u;
  send_quality_ = value;
}

bool RadarConf::has_sort_index() const {
  return (has_bits_ & 0x00000100u) != 0;
}
void RadarConf::clear_sort_index() {
  sort_index_ = 0u;
  has_bits_ &= ~0x00000100u;
}
uint32_t RadarConf::sort_index() const {
  return sort_index_;
}
void RadarConf::set_sort_index(uint32_t value) {
  has_bits_ |= 0x00000100u;
  sort_index_ = value;
}

bool RadarConf::has_store_in_nvm() const {
  return (has_bits_ & 0x00000800u) != 0;
}
void RadarConf::clear_store_in_nvm() {
  store_in_nvm_ = 1u;
  has_bits_ &= ~0x00000800u;
}
uint32_t RadarConf::store_in_nvm() const {
  return store_in_nvm_;
}
void RadarConf::set_store_in_nvm(uint32_t value) {
  has_bits_ |= 0x00000800u;
  store_in_nvm_ = value;
}

bool RadarConf::has_rcs_threshold() const {
  return (has_bits_ & 0x00000400u) != 0;
}
void RadarConf::clear_rcs_threshold() {
  rcs_threshold_ = 0;
  has_bits_ &= ~0x00000400u;
}
::drivers::conti_radar::RcsThreshold RadarConf::rcs_threshold() const {
  return static_cast< ::drivers::conti_radar::RcsThreshold >(rcs_threshold_);
}
void RadarConf::set_rcs_threshold(::drivers::conti_radar::RcsThreshold value) {
  has_bits_ |= 0x00000400u;
  rcs_threshold_ = value;
}

//
ContiRadarConf::ContiRadarConf() :
  has_bits_(0),
  radar_conf_(nullptr)
{}
ContiRadarConf::~ContiRadarConf()
{}
ContiRadarConf::ContiRadarConf(const ContiRadarConf& from)
{
  has_bits_ = from.has_bits_;
  radar_conf_ = new drivers::conti_radar::RadarConf(from.radar_conf());
}

bool ContiRadarConf::has_radar_conf() const {
  return (has_bits_ & 0x00000004u) != 0;
}
void ContiRadarConf::clear_radar_conf() {
  if (radar_conf_ != nullptr) 
  {
    delete radar_conf_;
  }
  has_bits_ &= ~0x00000004u;
}
const drivers::conti_radar::RadarConf& ContiRadarConf::radar_conf() const {
  const drivers::conti_radar::RadarConf* p = radar_conf_;
  return *p;
}
drivers::conti_radar::RadarConf* ContiRadarConf::release_radar_conf() {
  has_bits_ &= ~0x00000004u;
  drivers::conti_radar::RadarConf* temp = radar_conf_;
  radar_conf_ = nullptr;
  return temp;
}
drivers::conti_radar::RadarConf* ContiRadarConf::mutable_radar_conf() {
  has_bits_ |= 0x00000004u;
  if (radar_conf_ == nullptr) {
    radar_conf_ = new drivers::conti_radar::RadarConf();
  }
  return radar_conf_;
}
void ContiRadarConf::set_allocated_radar_conf(drivers::conti_radar::RadarConf* radar_conf) {
  if (radar_conf_ != nullptr) {
    delete radar_conf_;
    radar_conf_ = nullptr;
  }
  if (radar_conf) {
    has_bits_ |= 0x00000004u;
  } else {
    has_bits_ &= ~0x00000004u;
  }
  radar_conf_ = radar_conf;
}


#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

}  // namespace conti_radar
}  // namespace drivers

