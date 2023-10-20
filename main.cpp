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
#include "DebugMessage.hpp"
#include "MbedHardwareSerial.hpp"
#include "SerialBridge.hpp"

#include "mdc_client/MDCClient.hpp"

#include "Mecanum.hpp"

#include "Encoder.hpp"
#include "md.hpp"
#include "pid.hpp"

#include "Servo.hpp"

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

// mosi,miso,sck
SPI spi(PB_15, PB_14, PB_13);
MbedHardwareSPI hardware_dev0(spi, PB_12);
ACAN2517FD dev0_can(hardware_dev0, getMillisecond);
CANSerialBridge serial(&dev0_can);

MDCClient mdc_client(&serial, 0);
MDCClient mdc_client_2(&serial, 1);

DigitalOut acknowledge_0(PA_4);

SerialDev *dev =
    new MbedHardwareSerial(new BufferedSerial(USBTX, USBRX, 115200));
SerialBridge serial_control(dev, 1024);
Controller msc;
DebugMessage debug_msg;

DigitalOut led(PA_5);

MecanumWheel mw;

Encoder *encoder[3];
PID *pid[3];
MD *md[4];
Servo *servo[2];

double c_1,c_2;

void modules();

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
  serial_control.add_frame(10, &debug_msg);

  void modules();

  timer.start();

  //  set up
  ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_4MHz, 125UL * 1000UL, DataBitRateFactor::x8);

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

  setting_struct_t mdc_settings_0 = {OperatorMode::MD_OPERATOR,
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

  setting_struct_t mdc_settings_1 = {OperatorMode::MD_OPERATOR,
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

  setting_struct_t mdc_settings_2 = {OperatorMode::MD_OPERATOR,
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

  setting_struct_t mdc_settings_3 = {OperatorMode::MD_OPERATOR,
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

  setting_struct_t mdc_settings_4 = {OperatorMode::MD_OPERATOR,
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

  setting_struct_t mdc_settings_5 = {OperatorMode::MD_OPERATOR,
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

  setting_struct_t mdc_settings_6 = {OperatorMode::MD_OPERATOR,
                                     EncoderType::ANGLE,
                                     ENCODER_REVOLUTION,
                                     false,
                                     1.1,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0};

  setting_struct_t mdc_settings_7 = {OperatorMode::MD_OPERATOR,
                                     EncoderType::ANGLE,
                                     ENCODER_REVOLUTION,
                                     false,
                                     1.1,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0};

  wait_us(2000 * 1000);

  mdc_client.update_setting(0, mdc_settings_0);
  wait_us(250 * 1000);
  mdc_client.update_setting(1, mdc_settings_1);
  wait_us(250 * 1000);
  mdc_client.update_setting(2, mdc_settings_2);
  wait_us(250 * 1000);
  mdc_client.update_setting(3, mdc_settings_3);
  wait_us(250 * 1000);
  mdc_client_2.update_setting(0, mdc_settings_4);
  wait_us(250 * 1000);
  mdc_client_2.update_setting(1, mdc_settings_5);
  wait_us(250 * 1000);
  mdc_client_2.update_setting(2, mdc_settings_6);
  wait_us(250 * 1000);
  mdc_client_2.update_setting(3, mdc_settings_7);
  wait_us(250 * 1000);

  while (1) {

    serial_control.update();

    dev0_can.poll();

    if (mdc_client.update() || mdc_client_2.update()) {
      acknowledge_0 = !acknowledge_0;
    }

    if (msc.was_updated()) {

      led = !led;

      // Joystickの値を取得(値域を±0.5から±1にする)

      double joyLxValue = msc.data.Lx;
      double joyLyValue = msc.data.Ly;
      double joyRxValue = msc.data.Rx;
      double joyRyValue = msc.data.Ry;

      bool joyL1Value = msc.data.L1;
      bool joyR1Value = msc.data.R1;

      double joyL2Value = msc.data.L2;
      double joyR2Value = msc.data.R2;

      bool triangle = msc.data.triangle;
      bool square = msc.data.square;
      bool circle = msc.data.circle;
      bool cross = msc.data.cross;

      uint32_t t_ = getMicrosecond();

      /*
      if (joyXValue < 0.1 && joyXValue > -0.1) {
        joyXValue = 0;
      }

      if (joyYValue < 0.1 && joyYValue > -0.1) {
        joyYValue = 0;
      }
      */

      // ボタンの状態を取得(Lならマイナス,Rならプラス)
      double turn = joyRxValue * -1;

      // Joystickのベクトル化
      double targetSpeed =
          sqrt(joyLxValue * joyLxValue + joyLyValue * joyLyValue);
      double targetRotation = atan2(joyLyValue, joyLxValue) - (PI / 4);

      // targetSpeedが1,-1を超えないようにする
      if (targetSpeed > 1) {
        targetSpeed = 1;
      } else if (targetSpeed < -1) {
        targetSpeed = -1;
      }

      // targetSpeedが0.03以下の時に起動しないようにする

      if (targetSpeed < 0.03 && targetSpeed > -0.03) {
        targetSpeed = 0;
      }

      // targetRotationがマイナスにならないように2πたす
      if (targetRotation < 0) {
        targetRotation += (2 * PI);
      }

      // 目標速度, 回転速度, 回転方向を設定
      mw.control(targetSpeed, targetRotation, turn);

      // 上部展開(49)
      double updown = (triangle - cross) * 49;

      // キャタ逆転
      if(joyL1Value == 1){
        c_1 = joyL2Value * -1;
      }else{
        c_1 = joyL2Value;
      }

      if(joyR1Value == 1){
        c_2 = joyR2Value * -1;
      }else{
        c_2 = joyR2Value;
      }



      // printf("%u\n\r", getMicrosecond() - t_);

      mdc_client.set_target(0, mw.getSpeed(3));
      mdc_client.set_target(1, mw.getSpeed(2));
      mdc_client.set_target(2, mw.getSpeed(1));
      mdc_client.set_target(3, mw.getSpeed(0));

      mdc_client_2.set_target(0, c_1);
      mdc_client_2.set_target(1, c_2);
      mdc_client_2.set_target(2, updown);
      mdc_client_2.set_target(3, updown);

      mdc_client.send_target();
      mdc_client_2.send_target();

      //  DEBUG
      // memset(debug_msg.data.str, 0, 64);
      // sprintf(debug_msg.data.str, "%.2lf",b);
      // serial_control.write(10);
    }

    serial.update();

    gSentDate = getMillisecond();

    // 周期調整用 (ここを変えるならDELTA_Tも変える)
    // ThisThread::sleep_for(70ms);
  }
}

void modules() {
  // MD用PIDゲイン調整 {kp(比例), ki(積分), kd(微分), reverse(逆転)}
  pid[0] = new PID(1.1, 0, 0, 0);
  pid[1] = new PID(1.1, 0, 0, 0);
  pid[2] = new PID(1.1, 0, 0, 0);

  // エンコーダーの制御ピン (a, b)
  encoder[0] = new Encoder(PB_2, PC_6);
  encoder[1] = new Encoder(PB_10, PA_8);
  encoder[2] = new Encoder(PA_9, PC_7);

  // MDの制御ピン (pwmピン, dirピン, 逆転モード)

  md[0] = new MD(PC_9, PC_5, 0);
  md[1] = new MD(PC_8, PC_4, 0);
  md[2] = new MD(PA_0, PA_6, 0);
  md[3] = new MD(PA_1, PA_7, 0);

  // servo
  servo[0] = new Servo(PB_0, 0);
  servo[1] = new Servo(PB_6, 0);
}
