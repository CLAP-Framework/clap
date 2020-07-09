/******************************************************************************
 * @file radar_config_200.cc
 * @brief short range radar - Sensor Configuration
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-01
 *****************************************************************************/

#include "srr_radar/protocol/radar_config_200.h"
#include "srr_radar/common/byte.h"


namespace drivers {
namespace srr_radar {

using drivers::SrrRadar;
using drivers::canbus::Byte;

const uint32_t RadarConfig200::ID = 0x200;

RadarConfig200::RadarConfig200() {}
RadarConfig200::~RadarConfig200() {}

uint32_t RadarConfig200::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;       //Not used yet
  return PERIOD;
}

/**
 * @brief update the data
 * @param data a pointer to the data to be updated
 */
void RadarConfig200::UpdateData(uint8_t* data) {
  set_output_type_valid_p(data, radar_conf_.output_type_valid());
  set_output_type_p(data, radar_conf_.output_type());

  set_sensor_id_valid_p(data, radar_conf_.sensor_id_valid());
  set_sensor_id_p(data, static_cast<uint8_t>(radar_conf_.sensor_id()));
}

/**
 * @brief reset the private variables
 */
void RadarConfig200::Reset() {
  radar_conf_.set_output_type_valid(false);
  radar_conf_.set_output_type(OUTPUT_TYPE_CLUSTERS);

  radar_conf_.set_sensor_id_valid(false);
  radar_conf_.set_sensor_id(0);
}

RadarConfig200* RadarConfig200::set_output_type_valid(bool valid) {
  radar_conf_.set_output_type_valid(valid);
  return this;
}
 
RadarConfig200* RadarConfig200::set_output_type(OutputType type) {
  radar_conf_.set_output_type(type);
  return this;
} 

RadarConfig200* RadarConfig200::set_sensor_id_valid(bool valid) {
  radar_conf_.set_sensor_id_valid(valid);
  return this;
}

RadarConfig200* RadarConfig200::set_sensor_id(uint8_t data) {
  radar_conf_.set_sensor_id(data);
  return this;
}

RadarConfig200* RadarConfig200::set_radar_conf(RadarConf radar_conf) {
  radar_conf_ = radar_conf;
  return this;
}

RadarConf RadarConfig200::radar_conf() { return radar_conf_; }

void RadarConfig200::set_output_type_valid_p(uint8_t* data, bool valid) {
  Byte frame(data+7);
  if (valid) {
    frame.set_bit_1(0);
  } else {
    frame.set_bit_0(0);
  }
}

void RadarConfig200::set_output_type_p(uint8_t* data, OutputType type) {
  Byte frame(data);
  uint8_t value = static_cast<uint8_t>(type);
  frame.set_value(value, 4, 2);
}

void RadarConfig200::set_sensor_id_valid_p(uint8_t* data, bool valid) {
  Byte frame(data+7);
  if (valid) {
    frame.set_bit_1(1);
  } else {
    frame.set_bit_0(1);
  }
}

void RadarConfig200::set_sensor_id_p(uint8_t* data, uint8_t value) {
  Byte frame(data);
  frame.set_value(value, 0, 4);
}
}  // namespace srr_radar
}  // namespace drivers

