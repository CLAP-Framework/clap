/******************************************************************************
 * @file track_2_60d.cc
 * @brief short range radar - Track 2
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-04
 *****************************************************************************/

#include "srr_radar/protocol/track_2_60d.h"
#include "srr_radar/protocol/const_vars.h"

#include "glog/logging.h"

#include "srr_radar/common/byte.h"
#include "srr_radar/common/canbus_consts.h"


namespace drivers {
namespace srr_radar {

using drivers::canbus::Byte;

Track2_60D::Track2_60D() {}
const uint32_t Track2_60D::ID = 0x60D;

void Track2_60D::Parse(const std::uint8_t* bytes, int32_t length,
                                 SrrRadar* srr_radar) const {
  auto track_index = track2_index(bytes, length);
  for (int i = 0; i < srr_radar->srrobs_size(); ++i) {
    if (srr_radar->srrobs(i).track_index() == track_index) {
      auto obs = srr_radar->mutable_srrobs(i);
      obs->set_rcs_value(track_rcsvalue(bytes, length));
      obs->set_track_lifetime(track_lifetime(bytes, length));
      obs->set_track2_rollcount(track2_rollcount(bytes, length));      
      break;
    }
  }
  double timestamp = 0.0; //common::time::Clock::NowInSeconds();
  // auto header = obs->mutable_header();
  // header->CopyFrom(srr_radar->header());
  // header->set_timestamp_sec(timestamp);  
}     //end of Parse

double Track2_60D::track_rcsvalue(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes);
  uint32_t x = t0.get_byte(0, 8);

  double ret = x * TRACK_RCSVALUE_RES + TRACK_RCSVALUE_OFFSET;
  return ret;
}

double Track2_60D::track_lifetime(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 1);
  uint32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  uint32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;
  double ret = x * TRACK_LIFETIME_RES + TRACK_LIFETIME_OFFSET;
  return ret;
}

int Track2_60D::track2_index(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 5);

  int ret = x;
  return ret;
}

int Track2_60D::track2_rollcount(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}
}  // namespace srr_radar
}  // namespace drivers

