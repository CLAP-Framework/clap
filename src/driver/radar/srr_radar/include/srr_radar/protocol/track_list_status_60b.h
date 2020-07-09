/******************************************************************************
 * @file track_list_status_60b.h
 * @brief short range radar - Track List Status
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-03
 *****************************************************************************/

#pragma once

#include "srr_radar/common/protocol_data.h"
#include "srr_radar/proto/srr_radar.h"


namespace drivers {
namespace srr_radar {

using drivers::SrrRadar;

class TrackListStatus60B
    : public drivers::canbus::ProtocolData<SrrRadar> {
 public:
  static const uint32_t ID;
  TrackListStatus60B();
  void Parse(const std::uint8_t* bytes, int32_t length,
             SrrRadar* srr_radar) const override;

 private:
  int num_of_tracks(const std::uint8_t* bytes, int32_t length) const;   //0~25

  int trackst_rollcount(const std::uint8_t* bytes, int32_t length) const;   //0~3
};
}  // namespace srr_radar
}  // namespace drivers

