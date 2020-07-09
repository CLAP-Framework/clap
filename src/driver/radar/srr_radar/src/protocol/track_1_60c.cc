/******************************************************************************
 * @file track_1_60c.cc
 * @brief short range radar - Track 1
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-04
 *****************************************************************************/

#include "srr_radar/protocol/track_1_60c.h"
#include "srr_radar/protocol/const_vars.h"

#include "glog/logging.h"

#include "srr_radar/common/byte.h"
#include "srr_radar/common/canbus_consts.h"


namespace drivers {
namespace srr_radar {

using drivers::canbus::Byte;

Track1_60C::Track1_60C() {}
const uint32_t Track1_60C::ID = 0x60C;

void Track1_60C::Parse(const std::uint8_t* bytes, int32_t length,
                                 SrrRadar* srr_radar) const {
  auto obs = srr_radar->add_srrobs();  
  //auto obs = srr_radar->mutable_srrobs(i);
  obs->set_clusterortrack(false);    //track
  obs->set_track_id(track_id(bytes, length));
  obs->set_track_longitude_displ(track_longitude_displ(bytes, length));
  obs->set_track_lateral_displ(track_lateral_displ(bytes, length));
  obs->set_track_index(track_index(bytes, length));
  obs->set_track_longitude_vel(track_longitude_vel(bytes, length));
  obs->set_track_lateral_vel(track_lateral_vel(bytes, length));
  obs->set_track1_rollcount(track1_rollcount(bytes, length));  
  double timestamp = 0.0; //common::time::Clock::NowInSeconds();
  // auto header = obs->mutable_header();
  // header->CopyFrom(srr_radar->header());
  // header->set_timestamp_sec(timestamp);  
}     //end of Parse

int Track1_60C::track_id(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);

  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

double Track1_60C::track_longitude_displ(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 2);
  uint32_t x = t0.get_byte(0, 6);

  Byte t1(bytes + 3);
  uint32_t t = t1.get_byte(5, 3);
  x <<= 3;
  x |= t;
  double ret = x * TRACK_LONGDISPL_RES + TRACK_LONGDISPL_OFFSET;
  return ret;
}

double Track1_60C::track_lateral_displ(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 4);
  uint32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  uint32_t t = t1.get_byte(6, 2);
  x <<= 2;
  x |= t;
  double ret = x * TRACK_LATDISPL_RES + TRACK_LATDISPL_OFFSET;
  return ret;
}

int Track1_60C::track_index(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 5);

  int ret = x;
  return ret;
}

double Track1_60C::track_longitude_vel(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 5);
  uint32_t x = t0.get_byte(0, 6);

  Byte t1(bytes + 6);
  uint32_t t = t1.get_byte(2, 6);
  x <<= 6;
  x |= t;
  double ret = x * TRACK_LONGVREL_RES + TRACK_LONGVREL_OFFSET;
  return ret;
}

double Track1_60C::track_lateral_vel(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 7);
  uint32_t x = t0.get_byte(0, 8);

  double ret = x * TRACK_LATVREL_RES + TRACK_LATVREL_OFFSET;
  return ret;
}

int Track1_60C::track1_rollcount(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}
}  // namespace srr_radar
}  // namespace drivers

