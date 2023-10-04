#include "math.h"
#include "mbed.h"
#include <chrono>

#include "AnalogIn.h"
#include "DigitalOut.h"
#include "mbed_wait_api.h"
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "ACAN2517FD.h"
#include "CANSerialBridge.hpp"
#include "MbedHardwareSPI.h"

#include "Controller.hpp"
#include "MbedHardwareSerial.hpp"
#include "SerialBridge.hpp"

#include "mdc_client/MDCClient.hpp"

#include "Mecanum.hpp"

#include "pid.hpp"
#include "md.hpp"
#include "Encoder.hpp"

Timer timer;
double pre_timer = 0.01;

#define ENCODER_REVOLUTION 1296

using namespace acan2517fd;

const double PI = 3.1415;

uint32_t getMillisecond() {
  return (uint32_t)duration_cast<std::chrono::milliseconds>(
             timer.elapsed_time())
      .count();
}

uint32_t getMicrosecond() {
  return (uint32_t)duration_cast<std::chrono::microseconds>(
             timer.elapsed_time())
      .count();
}

SPI spi(PC_1, PB_14, PC_7);
MbedHardwareSPI hardware_dev0(spi, PB_9);
ACAN2517FD dev0_can(hardware_dev0, getMillisecond);
CANSerialBridge serial(&dev0_can);

MDCClient mdc_client(&serial, 0);

DigitalOut acknowledge_0(PC_8);

SerialDev *dev =
    new MbedHardwareSerial(new BufferedSerial(PC_10, PC_11, 115200));
SerialBridge serial_control(dev, 1024);
Controller msc;

DigitalOut led(PA_5);

MecanumWheel mw;

/*
    [0] --→ 左前　 (FrontLeft)  [FL]
    [1] --→ 右前　 (FrontRight) [FR]
    [2] --→ 左後ろ (RearLeft)   [RL]
    [3] --→ 右後ろ (RearRight)  [RR]
*/

static uint32_t gUpdateDate = 0;
static uint32_t gSentDate = 0;

int main() {

  serial_control.add_frame(0, &msc);

  timer.start();

  //  set up
  ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_4MHz, 125UL * 1000UL,
                              DataBitRateFactor::x8);

  settings.mRequestedMode = ACAN2517FDSettings::NormalFD;

  settings.mDriverTransmitFIFOSize = 5;
  settings.mDriverReceiveFIFOSize = 5;

  settings.mBitRatePrescaler = 1;
  //  Arbitation Bit Rate
  settings.mArbitrationPhaseSegment1 = 255;
  settings.mArbitrationPhaseSegment2 = 64;
  settings.mArbitrationSJW = 64;
  //  Data Bit Rate
  settings.mDataPhaseSegment1 = 31;
  settings.mDataPhaseSegment2 = 8;
  settings.mDataSJW = 8;

  //--- RAM Usage
  printf("MCP2517FD RAM Usage: %d [bytes]\n\r", settings.ramUsage());

  printf("initializing device 0...\n\r");
  const uint32_t errorCode0 = dev0_can.begin(settings);
  if (errorCode0 == 0) {
    printf("initialized device 0!\n\r");
  } else {
    printf("Configuration error 0x%x\n\r", errorCode0);
  }

  printf("all configuration completed!\n\r");

  setting_struct_t mdc_settings_0 = {OperatorMode::PID_OPERATOR,
                                     EncoderType::VELOCITY,
                                     ENCODER_REVOLUTION,
                                     true,
                                     1.1,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0};

  setting_struct_t mdc_settings_1 = {OperatorMode::PID_OPERATOR,
                                     EncoderType::VELOCITY,
                                     ENCODER_REVOLUTION,
                                     false,
                                     1.1,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0};

  setting_struct_t mdc_settings_2 = {OperatorMode::PID_OPERATOR,
                                     EncoderType::VELOCITY,
                                     ENCODER_REVOLUTION,
                                     true,
                                     1.1,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0};

  setting_struct_t mdc_settings_3 = {OperatorMode::PID_OPERATOR,
                                     EncoderType::VELOCITY,
                                     ENCODER_REVOLUTION,
                                     false,
                                     1.1,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0};

  mdc_client.update_setting(0, mdc_settings_0);
  wait_us(250 * 1000);
  mdc_client.update_setting(1, mdc_settings_1);
  wait_us(250 * 1000);
  mdc_client.update_setting(2, mdc_settings_2);
  wait_us(250 * 1000);
  mdc_client.update_setting(3, mdc_settings_3);
  wait_us(250 * 1000);

  while (1) {

    serial_control.update();

    dev0_can.poll();

    if (msc.was_updated()) {

      led = !led;

      // Joystickの値を取得(値域を±0.5から±1にする)
      double joyXValue = (msc.data.x - 0.5) * 2;
      double joyYValue = (msc.data.y - 0.5) * 2;

      uint32_t t_ = getMicrosecond();

      if (joyXValue < 0.1 && joyXValue > -0.1) {
        joyXValue = 0;
      }

      if (joyYValue < 0.1 && joyYValue > -0.1) {
        joyYValue = 0;
      }

      // ボタンの状態を取得(Lならマイナス,Rならプラス)
      double turn = ((msc.data.l * 0.3) - (msc.data.r * 0.3));

      if (msc.data.n == 1) {
        turn = msc.data.r - msc.data.l;
      }

      // Joystickのベクトル化
      double targetSpeed = sqrt(joyXValue * joyXValue + joyYValue * joyYValue);
      double targetRotation = atan2(joyYValue, joyXValue) - (PI / 4);

      // targetSpeedが1,-1を超えないようにする
      if (targetSpeed > 1) {
        targetSpeed = 1;
      } else if (targetSpeed < -1) {
        targetSpeed = -1;
      }

      // targetSpeedが0.1以下の時に起動しないようにする

      if (targetSpeed < 0.1 && targetSpeed > -0.1) {
        targetSpeed = 0;
      }

      // targetRotationがマイナスにならないように2πたす
      if (targetRotation < 0) {
        targetRotation += (2 * PI);
      }

      // 目標速度, 回転速度, 回転方向を設定
      mw.control(targetSpeed, targetRotation, turn);

      // printf("%u\n\r", getMicrosecond() - t_);

      mdc_client.set_target(0, mw.getSpeed(0));
      mdc_client.set_target(1, mw.getSpeed(1));
      mdc_client.set_target(2, mw.getSpeed(2));
      mdc_client.set_target(3, mw.getSpeed(3));

      mdc_client.send_target();
    }

    serial.update();
    

    gSentDate = getMillisecond();

    // 周期調整用 (ここを変えるならDELTA_Tも変える)
    // ThisThread::sleep_for(70ms);
  }
}
