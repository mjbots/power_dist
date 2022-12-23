// Copyright 2018-2021 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

// r2 and r3 silk is hw rev 0
// r4.0, r4.1 silk is hw rev 1
// r4.2 is hw rev 2, but is undistributed
// r4.3, r4.3b, r4.4 is hw rev 2
// r4.5 is hw rev 3

namespace fw {

extern volatile uint8_t g_measured_hw_rev;

}

// The most recent version of the HW.
#ifndef POWER_DIST_HW_REV
// r4.3b silk
#define POWER_DIST_HW_REV 2
#endif

constexpr int kHardwareInterlock[] = {
  0,   // r2, r3
  1,   // r4.0, r4.1
  2,   // r4.3, r4.3b, r4.4
  3,   // r4.5
};

// This firmware is compatible with the following hardware revisions.
constexpr int kCompatibleHwRev[] = {
  2, 3
};

#define HWREV_PIN0 PC_6
#define HWREV_PIN1 PA_15
#define HWREV_PIN2 PC_13

#define DEBUG_LED1 PA_0
#define DEBUG_LED2 PA_1

#define PWR_LED PB_12
#define PWR_SW PA_2

#define VSAMP_IN PB_14
#define VSAMP_OUT PA_7

#define CAN_TX PA_12
#define CAN_RX PA_11
#define CAN_SHDN PA_10


#define GPIO1 PC_10
#define GPIO2 PC_11
#define GPIO3 PB_3

#define ISAMP_BIAS PA_4
#define ISAMP PB_0
#define ISAMP_BUF_OUT PB_1
#define ISAMP_BUF_IN PB_15
#define ISAMP_AMP PA_8
#define OVERRIDE_PWR PC_14
#define OVERRIDE_3V3 PB_13
#define TPS2490_FLT PA_5
#define TSP2490_TIMER PB_11
#define FET_TEMP PC_5
