/******************************************************************************
 * @file cluster_1_70c.h
 * @brief short range radar - Cluster 1
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

class Cluster1_70C
    : public drivers::canbus::ProtocolData<SrrRadar> {
 public:
  static const uint32_t ID;
  Cluster1_70C();
  void Parse(const std::uint8_t* bytes, int32_t length,
             SrrRadar* srr_radar) const override;

 private:
  int cluster_index(const std::uint8_t* bytes, int32_t length) const;       //0~127

  int cluster1_rollcount(const std::uint8_t* bytes, int32_t length) const;       //0~3
  
  double rcs_value(const std::uint8_t* bytes, int32_t length) const;    //-50~30

  double range(const std::uint8_t* bytes, int32_t length) const;    //0~51

  int azimuth(const std::uint8_t* bytes, int32_t length) const;    //-90~90

  double vrel(const std::uint8_t* bytes, int32_t length) const;    //-35~35
};

}  // namespace srr_radar
}  // namespace drivers

