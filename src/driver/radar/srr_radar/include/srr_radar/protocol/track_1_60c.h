/******************************************************************************
 * @file track_1_60c.h
 * @brief short range radar - Track 1
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

class Track1_60C
    : public drivers::canbus::ProtocolData<SrrRadar> {
 public:
  static const uint32_t ID;
  Track1_60C();
  void Parse(const std::uint8_t* bytes, int32_t length,
             SrrRadar* srr_radar) const override;

 private: 
  int track_id(const std::uint8_t* bytes, int32_t length) const;    //0~65535

  double track_longitude_displ(const std::uint8_t* bytes, int32_t length) const;  //0~51.1

  double track_lateral_displ(const std::uint8_t* bytes, int32_t length) const;  //-51.1~51.2

  int track_index(const std::uint8_t* bytes, int32_t length) const;    //0~24

  double track_longitude_vel(const std::uint8_t* bytes, int32_t length) const;    //-35~35

  double track_lateral_vel(const std::uint8_t* bytes, int32_t length) const;    //-32~31.75

  int track1_rollcount(const std::uint8_t* bytes, int32_t length) const;    //0~3
};

}  // namespace srr_radar
}  // namespace drivers

