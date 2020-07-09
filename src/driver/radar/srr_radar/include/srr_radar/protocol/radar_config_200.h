/******************************************************************************
 * @file radar_config_200.h
 * @brief short range radar - Sensor Configuration
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-01
 *****************************************************************************/

#ifndef RADAR_CONFIG_200_H_
#define RADAR_CONFIG_200_H_

#include "srr_radar/proto/srr_radar_conf.h"
#include "srr_radar/common/protocol_data.h"
#include "srr_radar/proto/srr_radar.h"


namespace drivers {
namespace srr_radar {

using drivers::SrrRadar;
using drivers::srr_radar::OutputType;
using drivers::srr_radar::RadarConf;

class RadarConfig200
    : public drivers::canbus::ProtocolData<SrrRadar> {
 public:
  static const uint32_t ID;
  RadarConfig200();
  ~RadarConfig200();
  /**
   * @brief get the data period
   * @return the value of data period
   */
  uint32_t GetPeriod() const override;

  /**
   * @brief update the data
   * @param data a pointer to the data to be updated
   */
  void UpdateData(uint8_t* data) override;

  /**
   * @brief reset the private variables
   */
  void Reset() override;

  RadarConfig200* set_output_type_valid(bool valid);
  RadarConfig200* set_output_type(OutputType type);
  RadarConfig200* set_sensor_id_valid(bool valid);
  RadarConfig200* set_sensor_id(uint8_t data);
  RadarConfig200* set_radar_conf(RadarConf radar_conf);
  RadarConf radar_conf();

  void set_output_type_valid_p(uint8_t* data, bool valid);    //0～1
  void set_output_type_p(uint8_t* data, OutputType type);     //0～1
  void set_sensor_id_valid_p(uint8_t* data, bool valid);      //0～1
  void set_sensor_id_p(uint8_t* data, uint8_t value);     //0～7

 private:
  RadarConf radar_conf_;
};

}  // namespace srr_radar
}  // namespace drivers

#endif // SRR_RADAR_DRIVER_ROS_H_