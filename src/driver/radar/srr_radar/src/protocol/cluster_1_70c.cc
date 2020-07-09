/******************************************************************************
 * @file cluster_1_70c.cc
 * @brief short range radar - Cluster 1
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-04
 *****************************************************************************/
#include "glog/logging.h"
#include "ros/time.h"
#include "srr_radar/protocol/cluster_1_70c.h"
#include "srr_radar/common/byte.h"
#include "srr_radar/common/canbus_consts.h"
#include "srr_radar/protocol/const_vars.h"



namespace drivers {
namespace srr_radar {

using drivers::canbus::Byte;

Cluster1_70C::Cluster1_70C() {}
const uint32_t Cluster1_70C::ID = 0x70C;

void Cluster1_70C::Parse(const std::uint8_t* bytes, int32_t length,
                                  SrrRadar* srr_radar) const {
  auto obs = srr_radar->add_srrobs();  
  //auto obs = srr_radar->mutable_srrobs(i);
  obs->set_clusterortrack(true);    //cluster
  obs->set_cluster_index(cluster_index(bytes, length));
  obs->set_cluster1_rollcount(cluster1_rollcount(bytes, length));
  obs->set_rcs_value(rcs_value(bytes, length));
  obs->set_range(range(bytes, length));
  obs->set_azimuth(azimuth(bytes, length));
  obs->set_vrel(vrel(bytes, length));
  
  double timestamp = 0.0; //common::time::Clock::NowInSeconds();
  // auto header = obs->mutable_header();
  // header->CopyFrom(srr_radar->header());
  // header->set_timestamp_sec(timestamp);
}

int Cluster1_70C::cluster_index(const std::uint8_t* bytes,
                                       int32_t length) const {  
  Byte t0(bytes);
  uint32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int Cluster1_70C::cluster1_rollcount(const std::uint8_t* bytes,
                                       int32_t length) const {  
  Byte t0(bytes + 4);
  uint32_t x = t0.get_byte(6, 2);

  int ret = x;
  return ret;
}

double Cluster1_70C::rcs_value(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 1);
  uint32_t x = t0.get_byte(0, 8);

  double ret = x * CLUSTER_RCSVALUE_RES + CLUSTER_RCSVALUE_OFFSET;
  return ret;
}

double Cluster1_70C::range(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 2);
  uint32_t x = t0.get_byte(0, 8);

  double ret = x * CLUSTER_RANGE_RES + CLUSTER_RANGE_OFFSET;
  return ret;
}

int Cluster1_70C::azimuth(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 3);
  uint32_t x = t0.get_byte(0, 7);

  int ret = x * CLUSTER_AZIMUTH_RES + CLUSTER_AZIMUTH_OFFSET;
  return ret;
}

double Cluster1_70C::vrel(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 4);
  uint32_t x = t0.get_byte(0, 3);

  Byte t1(bytes + 5);
  uint32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;
  double ret = x * CLUSTER_VREL_RES + CLUSTER_VREL_OFFSET;
  return ret;
}
}  // namespace srr_radar
}  // namespace drivers

