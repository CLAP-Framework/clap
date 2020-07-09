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

namespace drivers {
namespace srr_radar {

const int CONTIID_START = 0x600;
const int CONTIID_END = 0x702;
const int WAIT_TIME = 4000;

// Try this many times when receiving using bcan, by default.
const int BCAN_RECV_TRIES = 4;

const int RADAR_CONFIG = 0x200;
const int RADAR_STATE = 0x60A;

// cluster
const double CLUSTER_RCSVALUE_RES = 0.5;        //unit:dBm2
const double CLUSTER_RCSVALUE_OFFSET = -50.0;
const double CLUSTER_RANGE_RES = 0.2;       //unit:m
const double CLUSTER_RANGE_OFFSET = 0.0;
const int CLUSTER_AZIMUTH_RES = 2;       //unit:deg
const int CLUSTER_AZIMUTH_OFFSET = -90;
const double CLUSTER_VREL_RES = 0.05;       //unit:m/s
const double CLUSTER_VREL_OFFSET = -35.0;

// Track1 
const double TRACK_LONGDISPL_RES = 0.1;     //unit:m
const double TRACK_LONGDISPL_OFFSET = 0.0;
const double TRACK_LATDISPL_RES = 0.1;     //unit:m
const double TRACK_LATDISPL_OFFSET = -51.1;
const double TRACK_LONGVREL_RES = 0.02;     //unit:m/s
const double TRACK_LONGVREL_OFFSET = -35.0;
const double TRACK_LATVREL_RES = 0.25;     //unit:m/s
const double TRACK_LATVREL_OFFSET = -32.0;

// Track2
const double TRACK_RCSVALUE_RES = 0.5;     //unit:dBm2
const double TRACK_RCSVALUE_OFFSET = -50.0;
const double TRACK_LIFETIME_RES = 0.1;     //unit:s
const double TRACK_LIFETIME_OFFSET = 0.0;
}  // namespace srr_radar
}  // namespace drivers

