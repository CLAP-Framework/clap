/******************************************************************************
 * @file track_list_status_60b.cc
 * @brief short range radar - Track List Status
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-03
 *****************************************************************************/

#include "srr_radar/protocol/track_list_status_60b.h"

#include "glog/logging.h"

#include "srr_radar/common/byte.h"
#include "srr_radar/common/canbus_consts.h"

namespace drivers {
namespace srr_radar {

using drivers::SrrRadarObs;
using drivers::canbus::Byte;

TrackListStatus60B::TrackListStatus60B() {}
const uint32_t TrackListStatus60B::ID = 0x60B;

void TrackListStatus60B::Parse(const std::uint8_t* bytes, int32_t length,
                                SrrRadar* srr_radar) const {
  auto status = srr_radar->mutable_track_list_status();
  auto set_num_of_tracks = num_of_tracks(bytes, length);
  status->set_num_of_tracks(set_num_of_tracks);
  status->set_trackst_rollcount(trackst_rollcount(bytes, length));
  srr_radar->mutable_srrobs()->reserve(set_num_of_tracks);
}

int TrackListStatus60B::num_of_tracks(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int TrackListStatus60B::trackst_rollcount(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 1);
  uint32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}
}  // namespace srr_radar
}  // namespace drivers

