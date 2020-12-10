// Copyright 2019-202 Josh Pieper, jjp@pobox.com.
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

#include "mbed.h"

#include "mjlib/base/assert.h"
#include "mjlib/micro/async_exclusive.h"
#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/telemetry_manager.h"
#include "mjlib/multiplex/micro_server.h"
#include "mjlib/multiplex/micro_stream_datagram.h"

#include "fw/fdcan.h"
#include "fw/fdcan_micro_server.h"
#include "fw/git_info.h"
#include "fw/lm5066.h"
#include "fw/millisecond_timer.h"
#include "fw/power_dist_hw.h"
#include "fw/stm32g4_flash.h"

namespace micro = mjlib::micro;
namespace multiplex = mjlib::multiplex;

namespace {

const uint32_t kPrechargeMs = 100;
const float kMaxPrechargedVoltage = 10.0f;

enum State {
  kPowerOff,
  kPrecharging,
  kPowerOn,
  kFault,
};

void SetClock() {
  // RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  // Select HSI as system clock source and configure the HCLK, PCLK1
  // and PCLK2 clocks dividers.
  RCC_ClkInitStruct.ClockType = (
      RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 |
      RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // 16 MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  // 16 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  // 16 MHz
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    return;
  }

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  {
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

    PeriphClkInit.PeriphClockSelection =
        RCC_PERIPHCLK_FDCAN |
        RCC_PERIPHCLK_I2C2 |
        RCC_PERIPHCLK_ADC345;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      mbed_die();
    }
  }

  SystemCoreClockUpdate();
}

#if POWER_DIST_HW_REV < 1
void RunRev0() {
  fw::FDCan::Filter filters[] = {
    { 0x10005, 0xffffff, fw::FDCan::FilterMode::kMask,
      fw::FDCan::FilterAction::kAccept,
      fw::FDCan::FilterType::kExtended,
    },
  };

  fw::FDCan can(
      [&]() {
        fw::FDCan::Options options;
        options.td = CAN_TX;
        options.rd = CAN_RX;
        options.slow_bitrate = 125000;
        options.fast_bitrate = 125000;

        options.filter_begin = &filters[0];
        options.filter_end = &filters[0] + (sizeof(filters) / sizeof(filters[0]));

        return options;
      }());

  DigitalOut led1(DEBUG_LED1, 1);
  DigitalOut led2(DEBUG_LED2, 0);

  DigitalOut switch_led(PWR_LED);
  DigitalIn power_switch(PWR_SW, PullUp);

  DigitalOut fet_precharge(FET_PRECHARGE);
  DigitalOut fet_main(FET_MAIN);

  // We use this merely to configure the pins.
  AnalogIn vsamp_out_do_not_use(VSAMP_OUT);

  fw::MillisecondTimer timer;

  State state = kPowerOff;
  uint32_t precharge_start = 0;

  uint32_t last_can = 0;

  char can_status_data[8] = {
    0, // switch status
    0, // lock time in 0.1s
  };

  char can_command_data[8] = {};

  char& power_switch_status = can_status_data[0];
  uint8_t& lock_time = reinterpret_cast<uint8_t&>(can_status_data[1]);

  FDCAN_RxHeaderTypeDef can_rx_header;

  // Configure ADC5
  {
    // Disable it to ensure we are in a known state.
    if (ADC5->CR & ADC_CR_ADEN) {
      ADC5->CR |= ADC_CR_ADDIS;
      while (ADC5->CR & ADC_CR_ADEN);
    }
    ADC345_COMMON->CCR = 0;  // no divisor
    ADC5->CR &= ~ADC_CR_DEEPPWD;
    ADC5->CR |= ADC_CR_ADVREGEN;
    timer.wait_us(20);
    ADC5->CR |= ADC_CR_ADCAL;
    while (ADC5->CR & ADC_CR_ADCAL);
    timer.wait_us(1);

    ADC5->ISR |= ADC_ISR_ADRDY;
    ADC5->CR |= ADC_CR_ADEN;
    while (!(ADC5->ISR & ADC_ISR_ADRDY));

    ADC5->ISR |= ADC_ISR_ADRDY;
    ADC5->CFGR &= ~(ADC_CFGR_CONT);
    ADC5->CFGR2 &= ~(
        ADC_CFGR2_SMPTRIG |
        ADC_CFGR2_BULB |
        ADC_CFGR2_SWTRIG |
        ADC_CFGR2_GCOMP |
        ADC_CFGR2_ROVSM |
        ADC_CFGR2_TROVS |
        ADC_CFGR2_JOVSE |
        ADC_CFGR2_ROVSE |
        0);


    const int sqr = 1;  // PA8 is channel 1
    ADC5->SQR1 =
        (0 << ADC_SQR1_L_Pos) | // length 1
        (sqr << ADC_SQR1_SQ1_Pos);
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
    ADC5->SMPR1 = make_cycles(2);  // 12 ADC cycles
    ADC5->SMPR2 = make_cycles(2);
  }


  for (;;) {
    {
      const auto now = timer.read_ms() / 100;
      if (now != last_can) {
        last_can = now;

        can.Send(0x10004, std::string_view(can_status_data,
                                           sizeof(can_status_data)));

        if (lock_time > 0) { lock_time--; }
      }

      if (can.Poll(&can_rx_header, can_command_data)) {
        if (can_command_data[1] > 0) {
          lock_time = static_cast<uint8_t>(can_command_data[1]);
        }
      }
    }

    switch (state) {
      case kPowerOff: {
        switch_led.write(1);  // inverted
        fet_main.write(0);
        fet_precharge.write(0);
        break;
      }
      case kPrecharging: {
        switch_led.write((timer.read_ms() / 20) % 2);
        fet_main.write(0);
        fet_precharge.write(1);
        break;
      }
      case kPowerOn: {
        switch_led.write(0);
        fet_main.write(1);
        fet_precharge.write(1);
        break;
      }
      case kFault: {
        fet_main.write(0);
        fet_precharge.write(0);
        switch_led.write((timer.read_ms() / 200) % 2);
        break;
      }
    }

    power_switch_status = (power_switch.read() == 0) ? 1 : 0;

    // Sample the ADC.
    ADC5->CR |= ADC_CR_ADSTART;
    while ((ADC5->ISR & ADC_ISR_EOC) == 0);
    const uint16_t raw_adc = ADC5->DR;
    const float out_voltage =
        (static_cast<float>(raw_adc) /
         4096.0f * 3.3f) / VSAMP_DIVIDE;

    // Handle state transitions.
    if (power_switch_status == 1) {
      // The switch is on.  Try to get into the power on state.
      switch (state) {
        case kPowerOff: {
          precharge_start = timer.read_ms();
          state = kPrecharging;
          break;
        }
        case kPrecharging: {
          const uint32_t elapsed_ms = timer.read_ms() - precharge_start;
          // Either move to power on or fault.  We fault if the
          // voltage did not get low enough during the pre-charge
          // cycle.
          if (elapsed_ms > kPrechargeMs) {
            if (out_voltage > kMaxPrechargedVoltage) {
              state = kFault;
            } else {
              state = kPowerOn;
            }
          }
          break;
        }
        case kPowerOn: {
          // Nothing to do here.
          break;
        }
        case kFault: {
          break;
        }
      }
    } else {
      if (lock_time == 0) {
        // Only turn off once our lock time has expired.
        state = kPowerOff;
      }
    }
  }
}
#endif

#if POWER_DIST_HW_REV >= 1
void RunRev1() {
  micro::SizedPool<14000> pool;

  // fw::FDCan::Filter filters[] = {
  //   { 0x10005, 0xffffff, fw::FDCan::FilterMode::kMask,
  //     fw::FDCan::FilterAction::kAccept,
  //     fw::FDCan::FilterType::kExtended,
  //   },
  // };

  fw::MillisecondTimer timer;

  fw::FDCan can(
      [&]() {
        fw::FDCan::Options options;
        options.td = CAN_TX;
        options.rd = CAN_RX;
        options.slow_bitrate = 125000;
        options.fast_bitrate = 125000;
        options.fdcan_frame = true;
        options.bitrate_switch = true;

        // options.filter_begin = &filters[0];
        // options.filter_end = &filters[0] + (sizeof(filters) / sizeof(filters[0]));

        return options;
      }());

  fw::FDCanMicroServer fdcan_micro_server(&can);
  multiplex::MicroServer multiplex_protocol(&pool, &fdcan_micro_server, {});

  micro::AsyncStream* serial = multiplex_protocol.MakeTunnel(1);

  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream(serial);
  micro::CommandManager command_manager(&pool, serial, &write_stream);
  micro::TelemetryManager telemetry_manager(
      &pool, &command_manager, &write_stream);
  fw::Stm32G4Flash flash_interface;
  micro::PersistentConfig persistent_config(pool, command_manager, flash_interface);

  persistent_config.Register("id", multiplex_protocol.config(), [](){});

  fw::GitInfo git_info;

  fw::Lm5066 lm5066(
      &pool, &persistent_config, &telemetry_manager, &timer,
      []() {
        fw::Lm5066::Options options;
        options.sda = PA_8;
        options.scl = PA_9;
        options.smba = PA_10;
        return options;
      }());

  telemetry_manager.Register("git", &git_info);

  persistent_config.Load();


  DigitalOut led1(DEBUG_LED1, 1);
  DigitalOut led2(DEBUG_LED2, 1);

  DigitalOut switch_led(PWR_LED);
  DigitalIn power_switch(PWR_SW, PullUp);

  DigitalOut pshdn(PSHDN, 1);

  uint32_t last_can = 0;

  char can_status_data[8] = {
    0, // switch status
    0, // lock time in 0.1s
  };

  // char can_command_data[8] = {};

  char& power_switch_status = can_status_data[0];
  uint8_t& lock_time = reinterpret_cast<uint8_t&>(can_status_data[1]);

  // FDCAN_RxHeaderTypeDef can_rx_header;

  command_manager.AsyncStart();
  multiplex_protocol.Start(nullptr);

  auto old_time = timer.read_ms();

  while (true) {
    {
      const auto now = timer.read_ms() / 100;
      if (now != last_can) {
        last_can = now;

        fw::FDCan::SendOptions send_options;
        send_options.fdcan_frame = fw::FDCan::Override::kDisable;
        send_options.bitrate_switch = fw::FDCan::Override::kDisable;
        can.Send(0x10004, std::string_view(can_status_data,
                                           sizeof(can_status_data)));

        if (lock_time > 0) { lock_time--; }
      }

      // if (can.Poll(&can_rx_header, can_command_data)) {
      //   if (can_command_data[1] > 0) {
      //     lock_time = static_cast<uint8_t>(can_command_data[1]);
      //   }
      // }
      led1.write(!((now % 10) == 0));
    }

    power_switch_status = (power_switch.read() == 0) ? 1 : 0;

    if (power_switch_status == 1) {
      pshdn.write(0);
      led2.write(0);
      switch_led.write(0);
    } else if (lock_time == 0) {
      pshdn.write(1);
      led2.write(1);
      switch_led.write(1);
    }

    const auto new_time = timer.read_ms();
    if (new_time != old_time) {
      telemetry_manager.PollMillisecond();
      lm5066.PollMillisecond();

      old_time = new_time;
    }

    fdcan_micro_server.Poll();
  }
}
#endif

}

ADC_TypeDef* const g_adc5 = ADC5;

int main(void) {
  // Drop our speed down to nothing, because we don't really need to
  // go fast for this and we might as well save the battery.
  SetClock();

  // We use ADC5 for VSAMP_OUT
  __HAL_RCC_ADC345_CLK_ENABLE();

  DigitalIn hwrev0(HWREV_PIN0, PullUp);
  DigitalIn hwrev1(HWREV_PIN1, PullUp);
  DigitalIn hwrev2(HWREV_PIN2, PullUp);

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

  // Check if the detected board revision level is in our compatible
  // set.
  const bool compatible = [&]() {
    for (auto possible_version : kCompatibleHwRev) {
      if (measured_hw_rev == possible_version) { return true; }
    }
    return false;
  }();
  MJ_ASSERT(compatible);



#if POWER_DIST_HW_REV < 1
  RunRev0();
#else
  RunRev1();
#endif
}

extern "C" {
  void abort() {
    mbed_die();
  }
}
