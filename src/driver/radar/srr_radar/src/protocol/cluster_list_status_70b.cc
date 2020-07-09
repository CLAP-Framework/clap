/******************************************************************************
 * @file cluster_list_status_70b.cc
 * @brief short range radar - Cluster List Status
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-02-03
 *****************************************************************************/

#include "srr_radar/protocol/cluster_list_status_70b.h"

#include "glog/logging.h"

#include "srr_radar/common/byte.h"
#include "srr_radar/common/canbus_consts.h"

namespace drivers {
namespace srr_radar {

using drivers::SrrRadarObs;
using drivers::canbus::Byte;

ClusterListStatus70B::ClusterListStatus70B() {}
const uint32_t ClusterListStatus70B::ID = 0x70B;

void ClusterListStatus70B::Parse(const std::uint8_t* bytes, int32_t length,
                                 SrrRadar* srr_radar) const {
  auto status = srr_radar->mutable_cluster_list_status();
  auto set_num_of_cluster = num_of_cluster(bytes, length);
  status->set_num_of_cluster(set_num_of_cluster);    //
  status->set_clusterst_rollcount(clusterst_rollcount(bytes, length));
  srr_radar->mutable_srrobs()->reserve(set_num_of_cluster);    
  //auto counter = status->near() + status->far();
  // srr_radar->mutable_contiobs()->Reserve(counter);
  //srr_radar->mutable_contiobs()->reserve(counter);    //::std::vector< ::drivers::ContiRadarObs >* mutable_contiobs();
}

int ClusterListStatus70B::num_of_cluster(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int ClusterListStatus70B::clusterst_rollcount(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}
}  // namespace srr_radar
}  // namespace drivers

