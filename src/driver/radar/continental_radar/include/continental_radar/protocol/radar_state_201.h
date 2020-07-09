/******************************************************************************
 * Copyright 2017 The AutoVehicle Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include "continental_radar/common/protocol_data.h"
#include "continental_radar/proto/conti_radar.h"
#include "continental_radar/proto/conti_radar_conf.h"

namespace drivers {
namespace conti_radar {

using drivers::ContiRadar;
using ::drivers::conti_radar::OutputType;
using ::drivers::conti_radar::RcsThreshold;

class RadarState201 : public drivers::canbus::ProtocolData<ContiRadar> {
 public:
  static const uint32_t ID;
  RadarState201();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ContiRadar* conti_radar) const override;

 private:
  int max_dist(const std::uint8_t* bytes, int32_t length) const;

  int radar_power(const std::uint8_t* bytes, int32_t length) const;

  OutputType output_type(const std::uint8_t* bytes, int32_t length) const;

  RcsThreshold rcs_threshold(const std::uint8_t* bytes, int32_t length) const;

  bool send_quality(const std::uint8_t* bytes, int32_t length) const;

  bool send_ext_info(const std::uint8_t* bytes, int32_t length) const;
};

}  // namespace conti_radar
}  // namespace drivers

