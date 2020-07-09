/******************************************************************************
 * @file track_2_60d.h
 * @brief short range radar - Track 2
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-04
 *****************************************************************************/

#pragma once

#include "srr_radar/common/protocol_data.h"
#include "srr_radar/proto/srr_radar.h"


namespace drivers {
namespace srr_radar {

using drivers::SrrRadar;

class Track2_60D
    : public drivers::canbus::ProtocolData<SrrRadar> {
 public:
  static const uint32_t ID;
  Track2_60D();
  void Parse(const std::uint8_t* bytes, int32_t length,
             SrrRadar* srr_radar) const override;

 private: 
double track_rcsvalue(const std::uint8_t* bytes, int32_t length) const;   //-50~30

double track_lifetime(const std::uint8_t* bytes, int32_t length) const;   //0~6553.5

int track2_index(const std::uint8_t* bytes, int32_t length) const;    //0~24

int track2_rollcount(const std::uint8_t* bytes, int32_t length) const;    //0~3
};

}  // namespace srr_radar
}  // namespace drivers

