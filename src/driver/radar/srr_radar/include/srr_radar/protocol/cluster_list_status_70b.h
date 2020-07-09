/******************************************************************************
 * @file cluster_list_status_70b.h
 * @brief short range radar - Cluster List Status
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

class ClusterListStatus70B
    : public drivers::canbus::ProtocolData<SrrRadar> {
 public:
  static const uint32_t ID;
  ClusterListStatus70B();
  void Parse(const std::uint8_t* bytes, int32_t length,
             SrrRadar* srr_radar) const override;

 private:
  int num_of_cluster(const std::uint8_t* bytes, int32_t length) const;    //1～128

  int clusterst_rollcount(const std::uint8_t* bytes, int32_t length) const;   //0～3    2bit
};

}  // namespace srr_radar
}  // namespace drivers

