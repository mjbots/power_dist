// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#include <numeric>

#include "mbed.h"

#include "mjlib/base/assert.h"
#include "mjlib/base/tokenizer.h"
#include "mjlib/base/limit.h"
#include "mjlib/micro/async_exclusive.h"
#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/callback_table.h"
#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/telemetry_manager.h"
#include "mjlib/multiplex/micro_server.h"
#include "mjlib/multiplex/micro_stream_datagram.h"

#include "fw/fdcan.h"
#include "fw/fdcan_micro_server.h"
#include "fw/firmware_info.h"
#include "fw/git_info.h"
#include "fw/lm5066.h"
#include "fw/millisecond_timer.h"
#include "fw/power_dist_hw.h"
#include "fw/stm32g4_flash.h"
#include "fw/uuid.h"

namespace base = mjlib::base;
namespace micro = mjlib::micro;
namespace multiplex = mjlib::multiplex;
using Value = multiplex::MicroServer::Value;
using mjlib::base::Limit;
using FDCan = fw::FDCan;

namespace {

template <typename T>
void Store(T& out, T value) {
  std::memcpy(&out, &value, sizeof(value));
}

template <typename T>
Value IntMapping(T value, size_t type) {
  switch (type) {
    case 0: return static_cast<int8_t>(value);
    case 1: return static_cast<int16_t>(value);
    case 2: return static_cast<int32_t>(value);
    case 3: return static_cast<float>(value);
  }
  MJ_ASSERT(false);
  return static_cast<int8_t>(0);
}

template <typename T>
Value ScaleSaturate(float value, float scale) {
  if (!std::isfinite(value)) {
    return std::numeric_limits<T>::min();
  }

  const float scaled = value / scale;
  const auto max = std::numeric_limits<T>::max();
  // We purposefully limit to +- max, rather than to min.  The minimum
  // value for our two's complement types is reserved for NaN.
  return Limit<T>(static_cast<T>(scaled), -max, max);
}

Value ScaleMapping(float value,
                   float int8_scale, float int16_scale, float int32_scale,
                   size_t type) {
  switch (type) {
    case 0: return ScaleSaturate<int8_t>(value, int8_scale);
    case 1: return ScaleSaturate<int16_t>(value, int16_scale);
    case 2: return ScaleSaturate<int32_t>(value, int32_scale);
    case 3: return Value(value);
  }
  MJ_ASSERT(false);
  return Value(static_cast<int8_t>(0));
}

Value ScaleTemperature(float value, size_t type) {
  return ScaleMapping(value, 1.0f, 0.1f, 0.001f, type);
}

Value ScaleCurrent(float value, size_t type) {
  // For now, current and temperature have identical scaling.
  return ScaleTemperature(value, type);
}

Value ScaleVoltage(float value, size_t type) {
  return ScaleMapping(value, 0.5f, 0.1f, 0.001f, type);
}

int16_t ReadInt16Mapping(Value value) {
  return std::visit([](auto a) {
      return static_cast<int16_t>(a);
    }, value);
}

int32_t ReadInt32Mapping(Value value) {
  return std::visit([](auto a) {
    return static_cast<int32_t>(a);
  }, value);
}

struct ValueScaler {
  float int8_scale;
  float int16_scale;
  float int32_scale;

  float operator()(int8_t value) const {
    if (value == std::numeric_limits<int8_t>::min()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return value * int8_scale;
  }

  float operator()(int16_t value) const {
    if (value == std::numeric_limits<int16_t>::min()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return value * int16_scale;
  }

  float operator()(int32_t value) const {
    if (value == std::numeric_limits<int32_t>::min()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return value * int32_scale;
  }

  float operator()(float value) const {
    return value;
  }
};

enum class Register {
  kState = 0x000,
  kFaultCode = 0x001,
  kSwitchStatus = 0x002,
  kLockTime = 0x003,
  kBootTime = 0x004,
  kOutputVoltage = 0x010,
  kOutputCurrent = 0x011,
  kTemperature = 0x012,
  kEnergy = 0x013,

  kUuid1 = 0x150,
  kUuid2 = 0x151,
  kUuid3 = 0x152,
  kUuid4 = 0x153,

  kUuidMask1 = 0x154,
  kUuidMask2 = 0x155,
  kUuidMask3 = 0x156,
  kUuidMask4 = 0x157,

  kUuidMaskCapable = 0x158,
};

enum State {
  kPowerOff,
  kPrecharging,
  kPowerOn,
  kFault,

  kNumStates,
};
}

namespace mjlib {
namespace base {

template <>
struct IsEnum<State> {
  static constexpr bool value = true;

  using S = State;
  static std::array<std::pair<S, const char*>, S::kNumStates> map() {
    return { {
        { S::kPowerOff, "power_off" },
        { S::kPrecharging, "precharging" },
        { S::kPowerOn, "power_on" },
        { S::kFault, "fault" },
      }};
  }
};

}
}

namespace {

void SetClock2() {
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_ClkInitStruct.ClockType = (
      RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 |
      RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // 170 MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  // 85 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  // 85 MHz

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
    return;
  }

  {
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

    PeriphClkInit.PeriphClockSelection =
        RCC_PERIPHCLK_FDCAN |
        RCC_PERIPHCLK_ADC12 |
        RCC_PERIPHCLK_ADC345;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      mbed_die();
    }
  }

  SystemCoreClockUpdate();
}

class OpAmpInvertingAmplifier {
 public:
  OpAmpInvertingAmplifier(OPAMP_TypeDef* opamp) {
    ctx_.Instance = opamp;
    ctx_.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
    ctx_.Init.Mode = OPAMP_PGA_MODE;
    ctx_.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;  // N/A in OPAMP_PGA_MODE
    ctx_.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_DAC;
    ctx_.Init.InternalOutput = DISABLE;
    ctx_.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    ctx_.Init.InvertingInputSecondary = {};
    ctx_.Init.NonInvertingInputSecondary = {};
    ctx_.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
    ctx_.Init.PgaGain = OPAMP_PGA_GAIN_8_OR_MINUS_7;
    ctx_.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
    ctx_.Init.TrimmingValueP = {};
    ctx_.Init.TrimmingValueN = {};

    if (HAL_OPAMP_Init(&ctx_) != HAL_OK) {
      mbed_die();
    }

    HAL_OPAMP_Start(&ctx_);
  }

  ~OpAmpInvertingAmplifier() {
    HAL_OPAMP_Stop(&ctx_);
  }

  OPAMP_HandleTypeDef ctx_ = {};
};

class OpAmpBuffer {
 public:
  enum Output {
    kInternal,
    kExternal,
  };

  OpAmpBuffer(OPAMP_TypeDef* opamp, int input_channel, Output output = kInternal) {
    ctx_.Instance = opamp;
    ctx_.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
    ctx_.Init.Mode = OPAMP_FOLLOWER_MODE;
    ctx_.Init.InvertingInput = {};  // N/A for follower
    ctx_.Init.NonInvertingInput = MapInput(input_channel);
    ctx_.Init.InternalOutput = (output == kInternal) ? ENABLE : DISABLE;
    ctx_.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    ctx_.Init.InvertingInputSecondary = {};
    ctx_.Init.NonInvertingInputSecondary = {};
    ctx_.Init.PgaConnect = {};
    ctx_.Init.PgaGain = {};
    ctx_.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
    ctx_.Init.TrimmingValueP = {};
    ctx_.Init.TrimmingValueN = {};

    if (HAL_OPAMP_Init(&ctx_) != HAL_OK) {
      mbed_die();
    }

    HAL_OPAMP_Start(&ctx_);
  }

  ~OpAmpBuffer() {
    HAL_OPAMP_Stop(&ctx_);
  }

  static int MapInput(int input_channel) {
    switch (input_channel) {
      case 0: return OPAMP_NONINVERTINGINPUT_IO0;
      case 1: return OPAMP_NONINVERTINGINPUT_IO1;
      case 2: return OPAMP_NONINVERTINGINPUT_IO2;
      case 3: return OPAMP_NONINVERTINGINPUT_IO3;
    }
    return 0;
  }

  OPAMP_HandleTypeDef ctx_ = {};
};

void ConfigureDAC1(fw::MillisecondTimer* timer) {
  __HAL_RCC_DAC1_CLK_ENABLE();

  // Initialize our gpio:
  {
    GPIO_InitTypeDef init = {};
    init.Pin = GPIO_PIN_4;
    init.Mode = GPIO_MODE_ANALOG;
    init.Pull = {};
    init.Speed = {};
    init.Alternate = {};
    HAL_GPIO_Init(GPIOA, &init);
  }

  // ISAMP_BIAS is PA4 -> DAC1_OUT1
  DAC1->MCR = (
      (0 << DAC_MCR_HFSEL_Pos) | // High frequency mode disabled
      (0 << DAC_MCR_MODE1_Pos) | // external pin with buffer enabled
      0);

  DAC1->CR = (
      (DAC_CR_EN1) | // enable channel 1
      0);

  // tWAKEUP is defined as max 7.5us
  timer->wait_us(10);

  // Write a half voltage.
  DAC1->DHR12R1 = 2048;
}

void ConfigureDAC4(fw::MillisecondTimer* timer) {
  __HAL_RCC_DAC4_CLK_ENABLE();

  DAC4->MCR = (
      (0 << DAC_MCR_HFSEL_Pos) | // High frequency mode disabled
      (3 << DAC_MCR_MODE1_Pos) | // internal with no buffer
      (3 << DAC_MCR_MODE2_Pos) | // internal with no buffer
      0);

  DAC4->CR = (
      (DAC_CR_EN1) | // enable channel 1
      (DAC_CR_EN2) | // enable channel 2
      0);

  // tWAKEUP is defined as max 7.5us
  timer->wait_us(10);

  // Write a half voltage to channel 1 and 2.
  DAC4->DHR12R1 = 2048;
  DAC4->DHR12R2 = 2048;
}

void ConfigureDAC3(fw::MillisecondTimer* timer) {
  __HAL_RCC_DAC3_CLK_ENABLE();

  DAC3->MCR = (
      (0 << DAC_MCR_HFSEL_Pos) | // High frequency mode disabled
      (3 << DAC_MCR_MODE1_Pos) | // internal with no buffer
      (3 << DAC_MCR_MODE2_Pos) | // internal with no buffer
      0);

  DAC3->CR = (
      (DAC_CR_EN1) | // enable channel 1
      (DAC_CR_EN2) | // enable channel 2
      0);

  // tWAKEUP is defined as max 7.5us
  timer->wait_us(10);

  // Write a half voltage to channel 1 and 2.
  DAC3->DHR12R1 = 2048;
  DAC3->DHR12R2 = 2048;
}

void ConfigureADC(ADC_TypeDef* adc, int channel_sqr, fw::MillisecondTimer* timer) {
  // Disable it to ensure we are in a known state.
  if (adc->CR & ADC_CR_ADEN) {
    adc->CR |= ADC_CR_ADDIS;
    while (adc->CR & ADC_CR_ADEN);
  }

  // The prescaler must be at least 6x to be able to accurately read
  // across all channels.  If it is too low, you'll see errors that
  // look like quantization, but not in a particularly uniform way
  // and not consistently across each of the channels.
  auto map_adc_prescale = [](int prescale) {
    if (prescale == 1) { return 0; }
    if (prescale == 2) { return 1; }
    if (prescale == 4) { return 2; }
    if (prescale == 6) { return 3; }
    if (prescale == 8) { return 4; }
    if (prescale == 10) { return 5; }
    if (prescale == 12) { return 6; }
    if (prescale == 16) { return 7; }
    if (prescale == 32) { return 8; }
    if (prescale == 64) { return 9; }
    if (prescale == 128) { return 10; }
    if (prescale == 256) { return 11; }
    MJ_ASSERT(false);
    return 0;
  };

  constexpr int kAdcPrescale = 8;

  ADC12_COMMON->CCR =
      (map_adc_prescale(kAdcPrescale) << ADC_CCR_PRESC_Pos);
  ADC345_COMMON->CCR =
      (map_adc_prescale(kAdcPrescale) << ADC_CCR_PRESC_Pos);

  adc->CR &= ~ADC_CR_DEEPPWD;
  adc->CR |= ADC_CR_ADVREGEN;
  timer->wait_us(20);
  adc->CR |= ADC_CR_ADCAL;
  while (adc->CR & ADC_CR_ADCAL);
  timer->wait_us(1);

  adc->ISR |= ADC_ISR_ADRDY;
  adc->CR |= ADC_CR_ADEN;
  while (!(adc->ISR & ADC_ISR_ADRDY));

  adc->ISR |= ADC_ISR_ADRDY;
  adc->CFGR &= ~(ADC_CFGR_CONT);
  adc->CFGR2 = (
      (0 << ADC_CFGR2_SMPTRIG_Pos) |
      (0 << ADC_CFGR2_BULB_Pos) |
      (0 << ADC_CFGR2_SWTRIG_Pos) |
      (0 << ADC_CFGR2_GCOMP_Pos) |
      (0 << ADC_CFGR2_ROVSM_Pos) |
      (0 << ADC_CFGR2_TROVS_Pos) |
      (0 << ADC_CFGR2_JOVSE_Pos) |
      (5 << ADC_CFGR2_OVSS_Pos) |  // 5 bit shift right
      (4 << ADC_CFGR2_OVSR_Pos) |  // oversample 32x
      (1 << ADC_CFGR2_ROVSE_Pos) | // enable regular oversampling
      0);


  adc->SQR1 =
      (0 << ADC_SQR1_L_Pos) | // length 1
      (channel_sqr << ADC_SQR1_SQ1_Pos);
  auto make_cycles = [](auto v) {
                       return
                           (v << 0) |
                           (v << 3) |
                           (v << 6) |
                           (v << 9) |
                           (v << 12) |
                           (v << 15) |
                           (v << 18) |
                           (v << 21) |
                           (v << 24);
                     };
  adc->SMPR1 = make_cycles(2);  // 47 ADC cycles
  adc->SMPR2 = make_cycles(2);
}

struct CanConfig {
  uint32_t prefix = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(prefix));
  }

  bool operator==(const CanConfig& rhs) const {
    return prefix == rhs.prefix;
  }
};

const int kShutdownTimeoutMs = 5000;
const int kMinOffTimeMs = 500;

class PowerDist : public mjlib::multiplex::MicroServer::Server {
 public:
  struct Config {
    float current_sense_ohm = 0.0005f;
    bool disable_sleep = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(current_sense_ohm));
      a->Visit(MJ_NVP(disable_sleep));
    }
  };

  struct Status {
    State state = kPowerOff;
    int8_t fault_code = 0;
    int8_t tps2490_fault = 0;
    int8_t switch_status = 0;
    int16_t lock_time_100ms = 0;

    float input_voltage_V = 0.0f;
    float output_voltage_V = 0.0f;
    float output_current_A = 0.0f;
    float fet_temp_C = 0.0f;
    int32_t energy_uW_hr = 0;

    int16_t int_temp_raw = 0;
    float int_temp_C = 0.0f;


    int32_t precharge_timeout_ms = 0;
    int32_t shutdown_timeout_ms = 0;

    int32_t off_time_ms = 0;

    uint16_t isamp_offset = 0;
    uint16_t isamp_average = 0;

    int8_t force_output = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(state));
      a->Visit(MJ_NVP(fault_code));
      a->Visit(MJ_NVP(tps2490_fault));
      a->Visit(MJ_NVP(switch_status));
      a->Visit(MJ_NVP(lock_time_100ms));

      a->Visit(MJ_NVP(input_voltage_V));
      a->Visit(MJ_NVP(output_voltage_V));
      a->Visit(MJ_NVP(output_current_A));
      a->Visit(MJ_NVP(fet_temp_C));
      a->Visit(MJ_NVP(energy_uW_hr));

      a->Visit(MJ_NVP(int_temp_raw));
      a->Visit(MJ_NVP(int_temp_C));

      a->Visit(MJ_NVP(precharge_timeout_ms));
      a->Visit(MJ_NVP(shutdown_timeout_ms));

      a->Visit(MJ_NVP(off_time_ms));

      a->Visit(MJ_NVP(isamp_offset));
      a->Visit(MJ_NVP(isamp_average));

      a->Visit(MJ_NVP(force_output));
    }
  };

  PowerDist() :
      gpio1_(GPIO1, 1),
      gpio2_(GPIO2, 0),
      can_shdn_(CAN_SHDN, 0),
      can_([&]() {
             fw::FDCan::Options options;
             options.td = CAN_TX;
             options.rd = CAN_RX;
             options.slow_bitrate = 1000000;
             options.fast_bitrate = 5000000;
             options.fdcan_frame = true;
             options.bitrate_switch = true;

             return options;
           }()),
      fdcan_micro_server_(&can_),
      multiplex_protocol_(&pool_, &fdcan_micro_server_, {}) {
    multiplex_protocol_.config()->id = 32;
  }

  void MaybeUpdateFilters() {
    // We only update our config if it has actually changed.
    // Re-initializing the CAN-FD controller can cause packets to
    // be lost, so don't do it unless actually necessary.
    if (can_config_ == old_can_config_ &&
        multiplex_protocol_.config()->id == old_multiplex_id_) {
      return;
    }

    old_can_config_ = can_config_;
    old_multiplex_id_ = multiplex_protocol_.config()->id;

    FDCan::Filter filters[4] = {};
    filters[0].id1 = (can_config_.prefix << 16) | old_multiplex_id_;
    filters[0].id2 = 0x1fff00ffu;
    filters[0].mode = FDCan::FilterMode::kMask;
    filters[0].action = FDCan::FilterAction::kAccept;
    filters[0].type = FDCan::FilterType::kExtended;

    filters[1].id1 = (can_config_.prefix << 16) | 0x7f;
    filters[1].id2 = 0x1fff00ffu;
    filters[1].mode = FDCan::FilterMode::kMask;
    filters[1].action = FDCan::FilterAction::kAccept;
    filters[1].type = FDCan::FilterType::kExtended;

    filters[2].id1 = (can_config_.prefix << 16) | old_multiplex_id_;
    filters[2].id2 = 0x1fff00ffu;
    filters[2].mode = FDCan::FilterMode::kMask;
    filters[2].action = FDCan::FilterAction::kAccept;
    filters[2].type = FDCan::FilterType::kStandard;

    filters[3].id1 = (can_config_.prefix << 16) | 0x7f;
    filters[3].id2 = 0x1fff00ffu;
    filters[3].mode = FDCan::FilterMode::kMask;
    filters[3].action = FDCan::FilterAction::kAccept;
    filters[3].type = FDCan::FilterType::kStandard;

    FDCan::FilterConfig filter_config;
    filter_config.begin = std::begin(filters);
    filter_config.end = std::end(filters);
    filter_config.global_std_action = FDCan::FilterAction::kReject;
    filter_config.global_ext_action = FDCan::FilterAction::kReject;
    can_.ConfigureFilters(filter_config);

    fdcan_micro_server_.SetPrefix(can_config_.prefix);
  }

  /// multiplex::MicroServer

  void StartFrame() override {
    discard_all_ = false;
  }

  Action CompleteFrame() override {
    if (discard_all_) {
      return kDiscard;
    }
    return kAccept;
  }

  WriteAction Write(multiplex::MicroServer::Register reg,
                    const Value& value) override __attribute__((optimize("O3"))) {
    if (discard_all_) { return kDiscardRemaining; }

    switch (static_cast<Register>(reg)) {
      case Register::kState: {
        // TODO: For now, mark as not writeable.
        return kNotWriteable;
      }
      case Register::kLockTime: {
        status_.lock_time_100ms = ReadInt16Mapping(value);
        return kSuccess;
      }
      case Register::kFaultCode:
      case Register::kSwitchStatus:
      case Register::kBootTime:
      case Register::kOutputVoltage:
      case Register::kOutputCurrent:
      case Register::kTemperature:
      case Register::kEnergy:
      case Register::kUuid1:
      case Register::kUuid2:
      case Register::kUuid3:
      case Register::kUuid4:
      case Register::kUuidMaskCapable: {
        // Not writeable.
        return kNotWriteable;
      }

      case Register::kUuidMask1:
      case Register::kUuidMask2:
      case Register::kUuidMask3:
      case Register::kUuidMask4: {
        const auto uuid = uuid_.uuid();
        const auto index =
            (static_cast<int>(reg) -
             static_cast<int>(Register::kUuidMask1)) * 4;

        const auto expected = *(reinterpret_cast<const int32_t*>(&uuid[index]));
        const auto written = ReadInt32Mapping(value);
        if (expected != written) {
          discard_all_ = true;
          return kDiscardRemaining;
        }
        return kSuccess;
      }

    }

    // This is an unknown register.
    return kUnknownRegister;
  }

  multiplex::MicroServer::ReadResult Read(
      multiplex::MicroServer::Register reg,
      size_t type) const override {
    if (discard_all_) {
      return static_cast<uint32_t>(1);
    }

    switch (static_cast<Register>(reg)) {
      case Register::kState: {
        return IntMapping(static_cast<int8_t>(status_.state), type);
      }
      case Register::kFaultCode: {
        return IntMapping(static_cast<int8_t>(status_.fault_code), type);
      }
      case Register::kSwitchStatus: {
        return IntMapping(static_cast<int8_t>(status_.switch_status), type);
      }
      case Register::kLockTime: {
        return IntMapping(static_cast<int16_t>(status_.lock_time_100ms), type);
      }
      case Register::kBootTime: {
        return IntMapping(static_cast<int16_t>(0), type);
      }
      case Register::kOutputVoltage: {
        return ScaleVoltage(status_.output_voltage_V, type);
      }
      case Register::kOutputCurrent: {
        return ScaleCurrent(status_.output_current_A, type);
      }
      case Register::kTemperature: {
        return ScaleTemperature(status_.fet_temp_C, type);
      }
      case Register::kEnergy: {
        const auto e = status_.energy_uW_hr;
        switch (type) {
          case 0: return Value(static_cast<int8_t>(e / 1000000));
          case 1: return Value(static_cast<int16_t>(e / 10000));
          case 2: return Value(static_cast<int32_t>(e));
          case 3: return Value(static_cast<float>(e) / 1000000.0f);
        }
        MJ_ASSERT(false);
        break;
      }
      case Register::kUuid1:
      case Register::kUuid2:
      case Register::kUuid3:
      case Register::kUuid4: {
        if (type != 2) { break; }

        const auto uuid = uuid_.uuid();
        const auto index =
            (static_cast<int>(reg) -
             static_cast<int>(Register::kUuid1)) * 4;
        return Value(*(reinterpret_cast<const int32_t*>(&uuid[index])));
      }
      case Register::kUuidMaskCapable: {
        return IntMapping(1, type);
      }
      case Register::kUuidMask1:
      case Register::kUuidMask2:
      case Register::kUuidMask3:
      case Register::kUuidMask4: {
        break;
      }
    }

    // Unknown register.
    return static_cast<uint32_t>(1);
  }


  /// Non-overriden methods

  void SetupAnalogGpio() {
    GPIO_InitTypeDef init = {};
    init.Mode = GPIO_MODE_ANALOG;
    init.Pull = {};
    init.Speed = {};
    init.Alternate = {};

    init.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &init);

    init.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &init);

    init.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &init);

    init.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &init);

    init.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &init);

    init.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB, &init);

    init.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &init);

    init.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOB, &init);
  }

  void SetupAnalog() {
    // Analog Mappings:
    //  VSAMP_OUT -> PA7 -> OPAMP1_VINP -> ADC1/IN13
    //  VSAMP_IN -> PB14 -> OPAMP2_VINP -> ADC2/IN16
    //  FET_TEMP -> PC5 -> ADC2/IN5
    //  ISAMP -> PB0 -> OPAMP3 -> PB1 -> ADC3/IN1 -> PB15 -> OPAMP5 -> PA8 -> ADC5/IN1
    //  DAC1 -> PA4 -> ISAMP_BIAS -> PA1 -> ADC12_IN2
    //  DAC3 -> internal
    //  DAC4 -> internal -> OPAMP5/VINP
    //  Internal_TEMP -> ADC5/IN4

    ConfigureDAC1(&timer_);
    ConfigureDAC3(&timer_);
    ConfigureDAC4(&timer_);

    ConfigureADC(ADC1, 13, &timer_);
    ConfigureADC(ADC2, 16, &timer_);
    ConfigureADC(ADC3, 1, &timer_);
    ConfigureADC(ADC5, 1, &timer_);

    ADC345_COMMON->CCR |= ADC_CCR_VSENSESEL;
  }

  void Setup() {
    command_manager_.Register(
        "p", std::bind(&PowerDist::HandleCommand, this,
                       std::placeholders::_1, std::placeholders::_2));

    persistent_config_.Register("id", multiplex_protocol_.config(), [this]() { MaybeUpdateFilters(); });
    persistent_config_.Register("can", &can_config_, [this]() { MaybeUpdateFilters(); });
    persistent_config_.Register("power", &config_, [](){});
    telemetry_manager_.Register("git", &git_info_);
    telemetry_manager_.Register("power", &status_);
    persistent_config_.Load();

    SetupAnalogGpio();
    SetupAnalog();
  }

  void HandleCommand(const std::string_view& message,
                     const micro::CommandManager::Response& response) {
    base::Tokenizer tokenizer(message, " ");
    const auto cmd_text = tokenizer.next();
    if (cmd_text == "lock") {
      const auto time_100ms = tokenizer.next();
      if (time_100ms.empty()) {
        WriteMessage(response, "ERR invalid time\r\n");
        return;
      }

      status_.lock_time_100ms =
          std::strtol(time_100ms.data(), nullptr, 10);

      WriteOk(response);
      return;
    } else if (cmd_text == "force") {
      const auto force_str = tokenizer.next();
      if (force_str == "off") {
        status_.force_output = 1;
      } else if (force_str == "on") {
        status_.force_output = 2;
      } else if (force_str == "disable") {
        status_.force_output = 0;
      } else {
        WriteMessage(response, "ERR invalid force\r\n");
        return;
      }

      WriteOk(response);
      return;
    }

    WriteMessage(response, "ERR unknown command\r\n");
  }

  void WriteOk(const micro::CommandManager::Response& response) {
    WriteMessage(response, "OK\r\n");
  }

  void WriteMessage(const micro::CommandManager::Response& response,
                    const std::string_view& message) {
    AsyncWrite(*response.stream, message, response.callback);
  }

  void Run() {
    Setup();

    command_manager_.AsyncStart();
    multiplex_protocol_.Start(this);

    timer_.wait_ms(20);

    auto callback = mjlib::micro::CallbackTable::MakeFunction(
        [this]() {
          this->gpio2_.write(!this->gpio2_.read());
          if (this->status_.state == kPrecharging ||
              this->status_.state == kPowerOn) {
            this->status_.fault_code = 3;
            this->status_.state = kFault;
          }
        });

    tps2490_flt_.fall(callback.raw_function);

    gpio1_.write(0);

    while (true) {
      SingleLoop();
    }
  }

  void SingleLoop() {
    status_.switch_status =
        (power_switch_.read() == 0) ? 0 : 1;
    status_.tps2490_fault = tps2490_flt_.read();

    SetOutputsFromState();
    MaybeChangeState();
    const auto new_time = timer_.read_ms();
    if (new_time != old_time_) {
      old_time_ = new_time;

      PollMillisecond();
      MeasureEnergy();

      if (new_time % 100 == 0) {
        PollHundredMillisecond();
      }
    }

    fdcan_micro_server_.Poll();
    multiplex_protocol_.Poll();
  }

  void PollMillisecond() {
    telemetry_manager_.PollMillisecond();
    UpdateMillisecondTimers();
    if (status_.state == kPowerOff) {
      status_.off_time_ms = std::min<int32_t>(
          status_.off_time_ms + 1, kMinOffTimeMs);
    } else {
      status_.off_time_ms = 0;
    }
  }

  void UpdateMillisecondTimers() {
    if (status_.shutdown_timeout_ms) {
      status_.shutdown_timeout_ms--;
    }
    if (status_.precharge_timeout_ms) {
      status_.precharge_timeout_ms--;
    }
  }

  void PollHundredMillisecond() {
    const auto status = can_.status();
    if (status.BusOff) {
      can_.RecoverBusOff();
    }

    if (status_.lock_time_100ms > 0) {
      status_.lock_time_100ms--;
    }
  }

  void MeasureEnergy() {
    ADC2->SQR1 =
        (0 << ADC_SQR1_L_Pos) | // length 1
        (16 << ADC_SQR1_SQ1_Pos);
    ADC5->SQR1 =
        (0 << ADC_SQR1_L_Pos) | // length 1
        (1 << ADC_SQR1_SQ1_Pos);

    // Sample the ADCs.
    ADC1->CR |= ADC_CR_ADSTART;
    ADC2->CR |= ADC_CR_ADSTART;
    ADC3->CR |= ADC_CR_ADSTART;
    ADC5->CR |= ADC_CR_ADSTART;

    while (((ADC1->ISR & ADC_ISR_EOC) == 0) ||
           ((ADC2->ISR & ADC_ISR_EOC) == 0) ||
           ((ADC3->ISR & ADC_ISR_EOC) == 0) ||
           ((ADC5->ISR & ADC_ISR_EOC) == 0));

    const uint16_t vsamp_out_raw = ADC1->DR;
    const uint16_t vsamp_in_raw = ADC2->DR;
    const uint16_t isamp_in_raw = ADC3->DR;
    (void) isamp_in_raw;
    const uint16_t isamp_in = ADC5->DR;

    const float vsamp_out =
        static_cast<float>(vsamp_out_raw) / 4096.0f * 3.3f / vsamp_divide_;
    const float vsamp_in =
        static_cast<float>(vsamp_in_raw) / 4096.0f * 3.3f / vsamp_divide_;
    const float V_per_A = config_.current_sense_ohm * 8 * 7;
    const float isamp =
        -((static_cast<float>(isamp_in) -
           status_.isamp_offset) / 4096.0f * 3.3f) / V_per_A;

    ADC2->SQR1 =
        (0 << ADC_SQR1_L_Pos) | // length 1
        (5 << ADC_SQR1_SQ1_Pos);
    ADC5->SQR1 =
        (0 << ADC_SQR1_L_Pos) | // length 1
        (4 << ADC_SQR1_SQ1_Pos);

    ADC2->CR |= ADC_CR_ADSTART;
    ADC5->CR |= ADC_CR_ADSTART;

    while (((ADC2->ISR & ADC_ISR_EOC) == 0) ||
           ((ADC5->ISR & ADC_ISR_EOC) == 0));

    const auto fet_temp_raw = ADC2->DR;
    const auto int_temp_raw = ADC5->DR;

    const float fet_temp_C =
        ((static_cast<float>(fet_temp_raw) / 4096.0f * 3.3f) - 1.8663f) /
        -0.01169f;

    const float int_temp_C =
        (static_cast<float>(int_temp_raw) - ts_cal1_) / static_cast<float>(ts_cal2_ - ts_cal1_) * 100.0f + 30.0f;

    if (vsamp_out > 4.0f) {
      const float delta_energy_uW_hr = vsamp_in * isamp * 0.001f / 3600.0f * 1e6f;
      status_.energy_uW_hr += static_cast<int32_t>(delta_energy_uW_hr);
    }

    status_.input_voltage_V = vsamp_in;
    status_.output_voltage_V = vsamp_out;
    status_.output_current_A = isamp;
    status_.fet_temp_C = fet_temp_C;
    status_.int_temp_raw = int_temp_raw;
    status_.int_temp_C = int_temp_C;


    isamp_sample_window_[isamp_sample_offset_] = isamp_in;
    isamp_sample_offset_ = (isamp_sample_offset_ + 1) % isamp_sample_window_.size();
    status_.isamp_average =
        std::accumulate(isamp_sample_window_.begin(),
                        isamp_sample_window_.end(),
                        0) /
        static_cast<float>(isamp_sample_window_.size());
  }

  void SetOutputsFromState() {
    // First, do our things that depend upon our current state.
    switch (status_.state) {
      case kPowerOff: {
        switch_led_.write(0);
        override_pwr_.write(0);
        led1_.write(1);
        override_3v3_.write(config_.disable_sleep ||
                            status_.shutdown_timeout_ms > 0);
        break;
      }
      case kPrecharging: {
        override_pwr_.write(1);
        override_3v3_.write(1);
        switch_led_.write((timer_.read_ms() / 20) % 2);
        led1_.write(0);
        break;
      }
      case kPowerOn: {
        override_pwr_.write(1);
        override_3v3_.write(1);
        switch_led_.write(1);
        led1_.write(0);
        break;
      }
      case kFault: {
        override_pwr_.write(0);
        override_3v3_.write(1);
        const int cycle = (timer_.read_ms() / 200);
        const bool on =
            (cycle % 2) && (cycle % 8) < (status_.fault_code * 2);
        switch_led_.write(on ? 1 : 0);
        led1_.write(on ? 0 : 1);
        break;
      }
      case kNumStates: {
        break;
      }
    }
  }

  void MaybeChangeState() {
    auto& state = status_.state;
    auto& fault_code = status_.fault_code;
    auto& precharge_timeout_ms = status_.precharge_timeout_ms;
    auto& shutdown_timeout_ms = status_.shutdown_timeout_ms;
    auto& power_switch_status = status_.switch_status;

    int8_t desired_output = status_.switch_status;
    if (status_.switch_status == 1) {
      if (status_.force_output == 1) {
        desired_output = 0;
      } else if (status_.force_output == 2) {
        desired_output = 1;
      }
    }

    switch (state) {
      case kPowerOff: {
        fault_code = 0;
        if (power_switch_status == 1 ||
            desired_output == 1) {
          // We don't want to turn off if the user is trying to turn
          // us back on.
          shutdown_timeout_ms = kShutdownTimeoutMs;
        }
        if (desired_output == 1 &&
            status_.off_time_ms == kMinOffTimeMs) {
          precharge_timeout_ms = 100;
          state = kPrecharging;

          // Read ADC5 before we start powering anything.
          status_.isamp_offset = status_.isamp_average;
        }
        break;
      }
      case kPrecharging: {
        fault_code = 0;
        if (status_.tps2490_fault == 1) {
          state = kPowerOn;
        } else if (desired_output == 0) {
          state = kPowerOff;
        } else if (precharge_timeout_ms == 0) {
          fault_code = 1;
          state = kFault;
        }
        shutdown_timeout_ms = kShutdownTimeoutMs;
        break;
      }
      case kPowerOn: {
        fault_code = 0;
        if (status_.tps2490_fault == 0) {
          state = kFault;
          fault_code = 2;
        } else if (desired_output == 0 &&
                   status_.lock_time_100ms == 0) {
          state = kPowerOff;
        }
        shutdown_timeout_ms = kShutdownTimeoutMs;
        break;
      }
      case kFault: {
        if (desired_output == 0) {
          state = kPowerOff;
        }
        shutdown_timeout_ms = kShutdownTimeoutMs;
        break;
      }
      case kNumStates: {
        break;
      }
    }
  }


  DigitalOut gpio1_;
  DigitalOut gpio2_;

  micro::SizedPool<14000> pool_;

  fw::MillisecondTimer timer_;

  DigitalOut can_shdn_;
  fw::FDCan can_;
  fw::FDCanMicroServer fdcan_micro_server_;
  multiplex::MicroServer multiplex_protocol_;
  micro::AsyncStream* serial_ = multiplex_protocol_.MakeTunnel(1);
  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream_{serial_};
  micro::CommandManager command_manager_{&pool_, serial_, &write_stream_};
  char micro_output_buffer[2048] = {};
  micro::TelemetryManager telemetry_manager_{
    &pool_, &command_manager_, &write_stream_, micro_output_buffer};
  fw::Stm32G4Flash flash_interface_;
  micro::PersistentConfig persistent_config_{
    pool_, command_manager_, flash_interface_, micro_output_buffer};
  fw::Uuid uuid_{persistent_config_};
  fw::GitInfo git_info_;
  CanConfig can_config_, old_can_config_;

  // Initialize this as bogus so we always update at least once.
  uint8_t old_multiplex_id_ = 255;

  fw::FirmwareInfo firmware_info_{pool_, telemetry_manager_, 0, 0};


  DigitalOut led1_{DEBUG_LED1, 1};
  DigitalOut led2_{DEBUG_LED2, 1};

  DigitalOut switch_led_{PWR_LED};
  DigitalIn power_switch_{PWR_SW};
  InterruptIn tps2490_flt_{TPS2490_FLT};

  // We start overriding 3V3, but not overall power.
  DigitalOut override_pwr_{OVERRIDE_PWR, 0};
  DigitalOut override_3v3_{OVERRIDE_3V3, 1};


  OpAmpBuffer opamp1_{OPAMP1, 2};  // PA7 == VINP2
  OpAmpBuffer opamp2_{OPAMP2, 1};  // PB14 == VINP1
  OpAmpBuffer opamp3_{OPAMP3, 0, OpAmpBuffer::kExternal};  // PB0 == VINP0, output = PB1
  OpAmpInvertingAmplifier opamp5_{OPAMP5};  // PB15 == VINM0

  Config config_;
  Status status_;

  std::array<uint16_t, 16> isamp_sample_window_ = {};
  int isamp_sample_offset_ = 0;

  uint32_t old_time_ = 0;

  float vsamp_divide_ =
              (fw::g_measured_hw_rev <= 2 ? (200.0f / (200.0f + 3000.0f)) :
               (fw::g_measured_hw_rev == 3 ? (200.0f / (200.0f + 4700.0f)) :
                1.0f));

  const uint16_t* ts_cal1_addr_ = reinterpret_cast<const uint16_t*>(0x1fff75a8);
  const uint16_t* ts_cal2_addr_ = reinterpret_cast<const uint16_t*>(0x1fff75ca);
  const uint16_t ts_cal1_ = *ts_cal1_addr_;
  const uint16_t ts_cal2_ = *ts_cal2_addr_;

  bool discard_all_ = false;
};

void RunRev2() {
  PowerDist power_dist;
  power_dist.Run();
}
}

ADC_TypeDef* const g_adc5 = ADC5;
volatile uint32_t rcc_csr = 0;

namespace fw {
volatile uint8_t g_measured_hw_rev;
}

int main(void) {
  rcc_csr = RCC->CSR;

#if POWER_DIST_HW_REV >= 2
  SetClock2();
#else
#error "Unsupported target"
#endif

  // We use ADC5 for VSAMP_OUT
  __HAL_RCC_ADC12_CLK_ENABLE();
  __HAL_RCC_ADC345_CLK_ENABLE();

  fw::MillisecondTimer timer;

  DigitalIn hwrev0(HWREV_PIN0, PullUp);
  DigitalIn hwrev1(HWREV_PIN1, PullUp);
  DigitalIn hwrev2(HWREV_PIN2, PullUp);

  timer.wait_ms(2);

  const uint8_t this_hw_pins =
      0x07 & (~(hwrev0.read() |
                (hwrev1.read() << 1) |
                (hwrev2.read() << 2)));
  const uint8_t measured_hw_rev = [&]() {
    int i = 0;
    for (auto rev_pins : kHardwareInterlock) {
      if (rev_pins == this_hw_pins) { return i; }
      i++;
    }
    return -1;
  }();

  fw::g_measured_hw_rev = measured_hw_rev;

  // Check if the detected board revision level is in our compatible
  // set.
  const bool compatible = [&]() {
    for (auto possible_version : kCompatibleHwRev) {
      if (measured_hw_rev == possible_version) { return true; }
    }
    return false;
  }();
  MJ_ASSERT(compatible);



#if POWER_DIST_HW_REV >= 2
  RunRev2();
#else
#error "Unsupported target"
#endif
}

extern "C" {
  void abort() {
    mbed_die();
  }
}
