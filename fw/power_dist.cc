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

template <typename T>
void Store(T& out, T value) {
  std::memcpy(&out, &value, sizeof(value));
}

const uint32_t kPrechargeMs = 100;
const float kMaxPrechargedVoltage = 10.0f;

enum State {
  kPowerOff,
  kPrecharging,
  kPowerOn,
  kFault,
};

#if POWER_DIST_HW_REV < 1
void SetClock0() {
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
        RCC_PERIPHCLK_ADC12 |
        RCC_PERIPHCLK_ADC345;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      mbed_die();
    }
  }

  SystemCoreClockUpdate();
}
#endif

#if POWER_DIST_HW_REV >= 2
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
#endif

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

  ADC12_COMMON->CCR = 0;  // no divisor
  ADC345_COMMON->CCR = 0;  // no divisor

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
  adc->SMPR1 = make_cycles(2);  // 12 ADC cycles
  adc->SMPR2 = make_cycles(2);
}

void RunRev2() {
  micro::SizedPool<14000> pool;

  fw::MillisecondTimer timer;

  DigitalOut can_shdn(CAN_SHDN, 0);

  fw::FDCan can(
      [&]() {
        fw::FDCan::Options options;
        options.td = CAN_TX;
        options.rd = CAN_RX;
        options.slow_bitrate = 1000000;
        options.fast_bitrate = 5000000;
        options.fdcan_frame = true;
        options.bitrate_switch = true;

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

  telemetry_manager.Register("git", &git_info);

  persistent_config.Load();


  DigitalOut led1(DEBUG_LED1, 1);
  // DigitalOut led2(DEBUG_LED2, 1);

  DigitalOut switch_led(PWR_LED);
  DigitalIn power_switch(PWR_SW, PullUp);

  // We start overriding 3V3, but not overall power.
  DigitalOut override_pwr(OVERRIDE_PWR, 0);
  DigitalOut override_3v3(OVERRIDE_3V3, 1);

  // Initialize our analog in pins:
  {
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

  uint32_t last_can = 0;

  char can_status_data[12] = {
    0, // switch status
    0, // lock time in 0.1s
    0, 0,  // voltage
    0, 0,  // isamp_out
    0, 0, 0, 0,  // energy in uW*hr
  };

  char can_command_data[8] = {};

  char& power_switch_status = can_status_data[0];
  uint8_t& lock_time = reinterpret_cast<uint8_t&>(can_status_data[1]);
  int16_t& out_input_10mV = reinterpret_cast<int16_t&>(can_status_data[2]);
  int16_t& out_output_10mV = reinterpret_cast<int16_t&>(can_status_data[4]);
  int16_t& out_isamp_10mA = reinterpret_cast<int16_t&>(can_status_data[6]);
  uint32_t& out_energy_uW_hr = reinterpret_cast<uint32_t&>(can_status_data[8]);

  uint32_t energy_uW_hr = 0;

  FDCAN_RxHeaderTypeDef can_rx_header;

  command_manager.AsyncStart();
  multiplex_protocol.Start(nullptr);

  auto old_time = timer.read_ms();

  // Analog Mappings:
  //  VSAMP_OUT -> PA7 -> OPAMP1_VINP -> ADC1/IN13
  //  VSAMP_IN -> PB14 -> OPAMP2_VINP -> ADC2/IN16
  //  ISAMP -> PB0 -> OPAMP3 -> PB1 -> ADC3/IN1 -> PB15 -> OPAMP5 -> PA8 -> ADC5/IN1
  //  DAC1 -> PA4 -> ISAMP_BIAS -> PA1 -> ADC12_IN2
  //  DAC3 -> internal
  //  DAC4 -> internal -> OPAMP5/VINP

  ConfigureDAC1(&timer);
  ConfigureDAC3(&timer);
  ConfigureDAC4(&timer);

  ConfigureADC(ADC1, 13, &timer);
  ConfigureADC(ADC2, 16, &timer);
  ConfigureADC(ADC3, 1, &timer);
  ConfigureADC(ADC5, 1, &timer);

  OpAmpBuffer opamp1(OPAMP1, 2);  // PA7 == VINP2
  OpAmpBuffer opamp2(OPAMP2, 1);  // PB14 == VINP1
  OpAmpBuffer opamp3(OPAMP3, 0, OpAmpBuffer::kExternal);  // PB0 == VINP0, output = PB1
  OpAmpInvertingAmplifier opamp5(OPAMP5);  // PB15 == VINM0

  timer.wait_us(200);

  // Read ADC5 while hopefully the switch is still off.
  ADC5->CR |= ADC_CR_ADSTART;
  while ((ADC5->ISR & ADC_ISR_EOC) == 0);
  const uint16_t isamp_offset = ADC5->DR;


  while (true) {
    const auto now = timer.read_ms() / 100;
    {
      if (now != last_can) {
        last_can = now;

        const auto status = can.status();
        if (status.BusOff) {
          can.RecoverBusOff();
        }

        fw::FDCan::SendOptions send_options;
        send_options.fdcan_frame = fw::FDCan::Override::kDisable;
        send_options.bitrate_switch = fw::FDCan::Override::kDisable;

        can.Send(0x10004, std::string_view(can_status_data,
                                           sizeof(can_status_data)));

        if (lock_time > 0) { lock_time--; }
      }


      if (fdcan_micro_server.Poll(&can_rx_header, can_command_data)) {
        if (can_command_data[1] > 0) {
          lock_time = static_cast<uint8_t>(can_command_data[1]);
        }
      }
    }

    power_switch_status = (power_switch.read() == 0) ? 1 : 0;



    if (power_switch_status == 1) {
      override_pwr.write(1);
      led1.write(0);
      lock_time = 3;
    } else if (lock_time == 0) {
      override_pwr.write(0);
      // override_3v3.write(0);
      led1.write(1);
    }

    // TODO Update the switch LED.


    const auto new_time = timer.read_ms();
    if (new_time != old_time) {
      telemetry_manager.PollMillisecond();

      old_time = new_time;

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
          static_cast<float>(vsamp_out_raw) / 4096.0f * 3.3f / VSAMP_DIVIDE;
      const float vsamp_in =
          static_cast<float>(vsamp_in_raw) / 4096.0f * 3.3f / VSAMP_DIVIDE;
      constexpr float V_per_A = 0.0005f * 8 * 7;
      const float isamp =
          -((static_cast<float>(isamp_in) - isamp_offset) / 4096.0f * 3.3f) / V_per_A;

      Store(out_input_10mV, static_cast<int16_t>(vsamp_in * 100.0f));
      Store(out_output_10mV, static_cast<int16_t>(vsamp_out * 100.0f));
      Store(out_isamp_10mA, static_cast<int16_t>(isamp * 100.0f));

      if (vsamp_out > 4.0f) {
        const float delta_energy_uW_hr = vsamp_in * isamp * 0.001f / 3600.0f * 1e6f;
        energy_uW_hr += static_cast<int32_t>(delta_energy_uW_hr);
      }

      Store(out_energy_uW_hr, energy_uW_hr);
    }
  }
}
#endif

}

ADC_TypeDef* const g_adc5 = ADC5;

int main(void) {
#if POWER_DIST_HW_REV < 1
  // Drop our speed down to nothing, because we don't really need to
  // go fast for this and we might as well save the battery.
  SetClock0();
#elif POWER_DIST_HW_REV >= 2
  SetClock2();
#else
#error "Unsupported target"
#endif

  // We use ADC5 for VSAMP_OUT
  __HAL_RCC_ADC12_CLK_ENABLE();
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
#elif POWER_DIST_HW_REV >= 2
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
