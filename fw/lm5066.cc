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

#include "fw/lm5066.h"

#include "mjlib/micro/callback_table.h"

namespace micro = mjlib::micro;

namespace fw {
namespace {
// Maybe make this faster in order to get better average power
// measurements.
constexpr int kUpdatePeriodMs = 100;

constexpr int PMBUS_READ = 0x01;

constexpr float CURRENT_SENSE_MOHM = 0.3f;

enum {
  DEVICE_SETUP = 0xd9,
  BLOCK_READ = 0xda,
};

template <typename T>
uint32_t u32(T value) {
  return reinterpret_cast<uint32_t>(value);
}
}

class Lm5066::Impl {
 public:
  Impl(micro::PersistentConfig* config,
       micro::TelemetryManager* telemetry,
       MillisecondTimer* timer,
       const Options& options)
      : timer_(timer),
        i2c_(options.sda, options.scl),
        smba_(options.smba) {

    // From CubeMX for 100kHz w/ 16MHz clock frequency
    smbus_.Instance = I2C2;
    smbus_.Init.Timing = 0x00303D5B;
    smbus_.Init.AnalogFilter = SMBUS_ANALOGFILTER_ENABLE;
    smbus_.Init.PacketErrorCheckMode = SMBUS_PEC_ENABLE;
    smbus_.Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_SMBUS_HOST;
    smbus_.Init.SMBusTimeout = 0;

    HAL_SMBUS_Init(&smbus_);

    ev_callback_ = micro::CallbackTable::MakeFunction(
        [this]() {
          HAL_SMBUS_EV_IRQHandler(&smbus_);
        });
    er_callback_ = micro::CallbackTable::MakeFunction(
        [this]() {
          HAL_SMBUS_ER_IRQHandler(&smbus_);
        });

    const IRQn_Type smbus_ev_irq = I2C2_EV_IRQn;
    const IRQn_Type smbus_er_irq = I2C2_ER_IRQn;

    NVIC_SetVector(smbus_ev_irq, u32(ev_callback_.raw_function));
    NVIC_SetVector(smbus_er_irq, u32(er_callback_.raw_function));

    HAL_NVIC_EnableIRQ(smbus_ev_irq);
    HAL_NVIC_EnableIRQ(smbus_er_irq);

    status_update_ = telemetry->Register("lm5066", &status_);

    // Configure the device.
    uint8_t device_setup = 0
        | (2 << 5)  // Retry setting = 010 (retry 1 time)
        | (1 << 4)  // Current limit setting = 1 (low 26mv)
        | (0 << 3)  // CB/CL ratio = 0 (low setting 1.9x)
        | (1 << 2)  // Current limit configuration = 1 (Use SMBus)
        | 0;
    SmbusWrite(DEVICE_SETUP, &device_setup, 1);

    uint8_t verify_device_setup[2] = {};
    SmbusRead(DEVICE_SETUP, verify_device_setup, 1);
    if (verify_device_setup[0] != device_setup) { mbed_die(); }
  }

  void PollMillisecond() {
    count_--;
    if (count_) { return; }

    count_ = kUpdatePeriodMs;
    uint8_t buf[16] = {};

    SmbusRead(BLOCK_READ, buf, 13);

    const auto& br = &buf[1];
    Status& s = status_;
    s.vout_uv_warn = br[0] & 0x80;
    s.i_op_warn = br[0] & 0x40;
    s.vin_uv_warn = br[0] & 0x20;
    s.vin_ov_warn = br[0] & 0x10;
    s.power_good = br[0] & 0x08;
    s.otemp_warn = br[0] & 0x04;
    s.timer_latched_off = br[0] & 0x02;
    s.ext_mosfet_shorted = br[0] & 0x01;

    s.config_preset = br[1] & 0x80;
    s.device_off = br[1] & 0x40;
    s.vin_uv_fault = br[1] & 0x20;
    s.vin_ov_fault = br[1] & 0x10;
    s.i_oc_fault = br[1] & 0x08;
    s.otemp_fault = br[1] & 0x04;
    s.cml_fault = br[1] & 0x02;
    s.cb_fault = br[1] & 0x01;

    const int16_t iin = (br[3] << 8) | br[2];
    const int16_t vout = (br[5] << 8) | br[4];
    const int16_t vin = (br[7] << 8) | br[6];
    const int16_t pin = (br[9] << 8) | br[8];
    const int16_t temp = (br[11] << 8) | br[10];

    status_.iin_raw = iin;
    status_.vout_raw = vout;
    status_.vin_raw = vin;
    status_.pin_raw = pin;
    status_.temperature_raw = temp;

    // These constants were calibrated by hand from the following dataset:
    //  10 - 0A
    //  27 - 0.9A
    //  38 - 1.2A
    //  54 - 1.5A
    //  68 - 1.8A
    //  81 - 2.1A
    status_.iin_10mA = static_cast<int16_t>(
        100.0f * ((iin * 100.0f + 1300.0f) / (15130.0f * CURRENT_SENSE_MOHM)));

    status_.vout_10mv = static_cast<int16_t>(
        100.0f * (vout * 100.0f + 2400.0f) / 4587.0f);
    status_.vin_10mv = static_cast<int16_t>(
        100.0f * (vin * 100.0f + 1200.0f) / 4578.0f);

    // Calibrated from the following empirical dataset:
    //  4 - 14W
    //  8 - 24W
    //  15 - 38W
    //  22 - 54W
    //  32 - 74W
    //  44 - 97W
    status_.pin_100mW = static_cast<int16_t>(
        10.0f * (pin * 1000.0f + 3500.0f) / (1606.4f * CURRENT_SENSE_MOHM));

    status_.temperature_C = static_cast<int16_t>(
        (temp * 1000.0f) / 16000.0f);

    status_update_();
  }

  void SmbusRead(uint8_t reg, uint8_t* data, size_t size) {
    if (HAL_SMBUS_Master_Transmit_IT(
            &smbus_, address_ << 1, &reg, 1, SMBUS_FIRST_FRAME) != HAL_OK) {
      mbed_die();
    }
    while (HAL_SMBUS_GetState(&smbus_) != HAL_SMBUS_STATE_READY);

    if (HAL_SMBUS_Master_Receive_IT(
            &smbus_, address_ << 1, data, size, SMBUS_LAST_FRAME_NO_PEC) != HAL_OK) {
      mbed_die();
    }
    while (HAL_SMBUS_GetState(&smbus_) != HAL_SMBUS_STATE_READY);
  }

  void SmbusWrite(uint8_t reg, uint8_t* data, size_t size) {
    outbuf_[0] = reg;
    std::memcpy(&outbuf_[1], data, size);
    if (HAL_SMBUS_Master_Transmit_IT(
            &smbus_, address_ << 1, outbuf_, size + 1, SMBUS_LAST_FRAME_NO_PEC) != HAL_OK) {
      mbed_die();
    }
    while (HAL_SMBUS_GetState(&smbus_) != HAL_SMBUS_STATE_READY);
  }

  fw::MillisecondTimer* const timer_;

  I2C i2c_;  // we use this just to initialize the pins appropriately
  SMBUS_HandleTypeDef smbus_ = {};

  DigitalIn smba_;
  int address_ = 0x40;

  Status status_;
  mjlib::base::inplace_function<void()> status_update_;
  int count_ = kUpdatePeriodMs;

  micro::CallbackTable::Callback ev_callback_;
  micro::CallbackTable::Callback er_callback_;

  uint8_t outbuf_[17] = {};
};

Lm5066::Lm5066(micro::Pool* pool,
               micro::PersistentConfig* config,
               micro::TelemetryManager* telemetry,
               MillisecondTimer* timer,
               const Options& options)
    : impl_(pool, config, telemetry, timer, options) {}

Lm5066::~Lm5066() {}

void Lm5066::PollMillisecond() {
  impl_->PollMillisecond();
}

}
