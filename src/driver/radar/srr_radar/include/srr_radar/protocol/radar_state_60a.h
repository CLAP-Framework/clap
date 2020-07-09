/******************************************************************************
 * @file radar_state_60a.h
 * @brief short range radar - Radar Status
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-03
 *****************************************************************************/

#pragma once

#include "srr_radar/common/protocol_data.h"
#include "srr_radar/proto/srr_radar.h"
#include "srr_radar/proto/srr_radar_conf.h"

namespace drivers {
namespace srr_radar {

using drivers::SrrRadar;
using ::drivers::srr_radar::OutputType;

class RadarState60A : public drivers::canbus::ProtocolData<SrrRadar> {
 public:
  static const uint32_t ID;
  RadarState60A();
  void Parse(const std::uint8_t* bytes, int32_t length,
             SrrRadar* srr_radar) const override;

 private: 
  int actl_mode(const std::uint8_t* bytes, int32_t length) const;     //0～5

  int radar_cfg_status(const std::uint8_t* bytes, int32_t length) const;    //0～4

  int radarst_rollcount(const std::uint8_t* bytes, int32_t length) const;     //0～3
};

}  // namespace srr_radar
}  // namespace drivers

