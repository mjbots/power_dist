// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

#include "PinNames.h"

#include "mjlib/base/visitor.h"

#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/telemetry_manager.h"

#include "fw/millisecond_timer.h"

namespace fw {

class Lm5066 {
 public:
  struct Options {
    PinName sda = NC;
    PinName scl = NC;
    PinName smba = NC;
  };

  Lm5066(mjlib::micro::Pool*,
         mjlib::micro::CommandManager*,
         mjlib::micro::PersistentConfig*,
         mjlib::micro::TelemetryManager*,
         MillisecondTimer* timer,
         const Options&);
  ~Lm5066();

  void PollMillisecond();

  enum class Fault {
    kNone = 0,
    kOverCurrent = 1,
    kUnderVoltage = 2,
    kOverTemperature = 3,
    kOverVoltage = 4,
    kCircuitBreaker = 5,
    kMosfetShorted = 6,
    kCommunications = 7,
    kSize,
  };

  struct Status {
    bool vout_uv_warn = false;
    bool i_op_warn = false;
    bool vin_uv_warn = false;
    bool vin_ov_warn = false;
    bool power_good = false;
    bool otemp_warn = false;
    bool timer_latched_off = false;
    bool ext_mosfet_shorted = false;
    bool config_preset = false;
    bool device_off = false;
    bool vin_uv_fault = false;
    bool vin_ov_fault = false;
    bool i_oc_fault = false;
    bool otemp_fault = false;
    bool cml_fault = false;
    bool cb_fault = false;

    int16_t iin_raw = 0;
    int16_t vout_raw = 0;
    int16_t vin_raw = 0;
    int16_t pin_raw = 0;
    int16_t temperature_raw = 0;

    int16_t iin_10mA = 0;
    int16_t vout_10mv = 0;
    int16_t vin_10mv = 0;
    int16_t pin_100mW = 0;
    int16_t temperature_C = 0;

    Fault fault = Fault::kNone;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(vout_uv_warn));
      a->Visit(MJ_NVP(i_op_warn));
      a->Visit(MJ_NVP(vin_uv_warn));
      a->Visit(MJ_NVP(vin_ov_warn));
      a->Visit(MJ_NVP(power_good));
      a->Visit(MJ_NVP(otemp_warn));
      a->Visit(MJ_NVP(timer_latched_off));
      a->Visit(MJ_NVP(ext_mosfet_shorted));
      a->Visit(MJ_NVP(config_preset));
      a->Visit(MJ_NVP(device_off));
      a->Visit(MJ_NVP(vin_uv_fault));
      a->Visit(MJ_NVP(vin_ov_fault));
      a->Visit(MJ_NVP(i_oc_fault));
      a->Visit(MJ_NVP(otemp_fault));
      a->Visit(MJ_NVP(cml_fault));
      a->Visit(MJ_NVP(cb_fault));

      a->Visit(MJ_NVP(iin_raw));
      a->Visit(MJ_NVP(vout_raw));
      a->Visit(MJ_NVP(vin_raw));
      a->Visit(MJ_NVP(pin_raw));
      a->Visit(MJ_NVP(temperature_raw));

      a->Visit(MJ_NVP(iin_10mA));
      a->Visit(MJ_NVP(vout_10mv));
      a->Visit(MJ_NVP(vin_10mv));
      a->Visit(MJ_NVP(pin_100mW));
      a->Visit(MJ_NVP(temperature_C));
      a->Visit(MJ_NVP(fault));
    }
  };

  const Status& status() const;

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}

namespace mjlib {
namespace base {

template <>
struct IsEnum<fw::Lm5066::Fault> {
  static constexpr bool value = true;

  using F = fw::Lm5066::Fault;
  static std::array<std::pair<F, const char*>, static_cast<size_t>(F::kSize)> map() {
    return { {
        { F::kNone, "none" },
        { F::kOverCurrent, "over_current" },
        { F::kUnderVoltage, "under_voltage" },
        { F::kOverTemperature, "over_temp" },
        { F::kOverVoltage, "over_voltage" },
        { F::kCircuitBreaker, "circuit_breaker" },
        { F::kMosfetShorted, "mosfet_shorted" },
        { F::kCommunications, "communications" },
      }};
  }
};

}
}
