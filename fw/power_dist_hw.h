// Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
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
// r4 silk is hw rev 1

// The most recent version of the HW.
#ifndef POWER_DIST_HW_REV
// r4 silk
#define POWER_DIST_HW_REV 1
#endif

constexpr int kHardwareInterlock[] = {
  0,   // r2, r3
  1,   // r4
};

// This firmware is compatible with the following hardware revisions.
constexpr int kCompatibleHwRev[] = {
  1
};

#define HWREV_PIN0 PC_6
#define HWREV_PIN1 PA_15
#define HWREV_PIN2 PC_13

#if POWER_DIST_HW_REV < 1
#define DEBUG_LED1 PA_4
#define DEBUG_LED2 PA_5
#else
#define DEBUG_LED1 PA_0
#define DEBUG_LED2 PA_1
#endif

#if POWER_DIST_HW_REV < 1
#define FET_PRECHARGE PA_0
#define FET_MAIN PA_1
#endif

#if POWER_DIST_HW_REV < 1
#define PWR_LED PA_2
#define PWR_SW PA_3
#else
#define PWR_LED PB_9
#define PWR_SW PB_7
#endif

#if POWER_DIST_HW_REV < 1
#define VSAMP_IN PA_9
#define VSAMP_OUT PA_8
#endif

#define CAN_TX PA_12
#define CAN_RX PA_11
#if POWER_DIST_HW_REV < 1
#define CAN_SHDN PA_10
#else
#define CAN_SHDN PA_15
#endif

#if POWER_DIST_HW_REV < 1
// Only reworked boards r2/r3/r3.1 boards have this divider, but
// non-reworked versions have an older firmware installed.
#define VSAMP_DIVIDE (4.7f / (10.0f + 100.0f))
#endif

#if POWER_DIST_HW_REV >= 1
#define PSHDN PB_13

#define I2C_SCL PA_9
#define I2C_SDA PA_8
#define I2C_SMBA PA_10

#define GPIO1 PC_10
#define GPIO2 PC_11
#define GPIO3 PB_3
#endif
