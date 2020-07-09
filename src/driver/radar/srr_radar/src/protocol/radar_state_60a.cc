/******************************************************************************
 * @file radar_state_60a.cc
 * @brief short range radar - Radar Status
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-03
 *****************************************************************************/

#include "srr_radar/protocol/radar_state_60a.h"

#include "glog/logging.h"

#include "srr_radar/common/byte.h"
#include "srr_radar/common/canbus_consts.h"

namespace drivers {
namespace srr_radar {

using drivers::canbus::Byte;

RadarState60A::RadarState60A() {}
const uint32_t RadarState60A::ID = 0x60A;

void RadarState60A::Parse(const std::uint8_t* bytes, int32_t length,
                          SrrRadar* srr_radar) const {
  auto state = srr_radar->mutable_radar_state();
  //drivers::RadarState_60A* state = srr_radar->mutable_radar_state();
  state->set_actl_mode(actl_mode(bytes, length));
  state->set_radar_cfg_status(radar_cfg_status(bytes, length));
  state->set_radarst_rollcount(radarst_rollcount(bytes, length));
}

int RadarState60A::actl_mode(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes);
  uint32_t x = t0.get_byte(0, 6);

  int ret = x;
  return ret;
}

int RadarState60A::radar_cfg_status(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  uint32_t x = t0.get_byte(4, 4);
  
  int ret = x;
  return ret;
}

int RadarState60A::radarst_rollcount(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  uint32_t x = t0.get_byte(0, 2);
  
  int ret = x;
  return ret;
}
}  // namespace srr_radar
}  // namespace drivers

