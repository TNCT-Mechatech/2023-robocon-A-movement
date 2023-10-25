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
#include "MovementFeedback.hpp"
#include "PIDGain.hpp"
#include "MbedHardwareSerial.hpp"
#include "SerialBridge.hpp"

#include "mdc_client/MDCClient.hpp"

#include "Mecanum.hpp"
#include "Servo.hpp"
#include "SpeedController.hpp"

#include "MD.hpp"
//#include "Encoder.hpp"
//#include "PID.hpp"

Timer timer;
double pre_timer = 0.01;

#define ENCODER_REVOLUTION 4647

using namespace acan2517fd;

const double PI = 3.1415;

uint32_t getMillisecond() {
  return (uint32_t) duration_cast<std::chrono::milliseconds>
    (timer.elapsed_time()).count();
}

uint32_t getMicrosecond() {
  return (uint32_t) duration_cast<std::chrono::microseconds>
    (timer.elapsed_time()).count();
}

// mosi,miso,sck
SPI spi(PB_15, PB_14, PB_13);
MbedHardwareSPI hardware_dev0(spi, PB_12);
ACAN2517FD dev0_can(hardware_dev0, getMillisecond);
CANSerialBridge serial(&dev0_can);

MDCClient mdc_client   (&serial, 0);
MDCClient mdc_client_2 (&serial, 1);

DigitalOut acknowledge_0(PA_4);

SerialDev *dev =
  new MbedHardwareSerial(new BufferedSerial(USBTX, USBRX, 115200));
SerialBridge serial_control(dev, 1024);
Controller msc;
DebugMessage debug_msg;
MovementFeedback movement_feedback_msg[2];
PIDGain pid_gain_msg;

DigitalOut led(PA_5);

MecanumWheel mw;

//台形加速
SpeedController sc0(0.01, 0.05);
SpeedController sc1(0.01, 0.05);
SpeedController sc2(0.01, 0.05);
SpeedController sc3(0.01, 0.05);
SpeedController sc4(0.2,  0.3);
SpeedController sc5(0.2,  0.3);
SpeedController sc6(0.1,  0.2);
SpeedController sc7(0.1,  0.2);
SpeedController sc8(0.1,  0.2);

//MD *md[4];
//Encoder *encoder[3];
//PID::ctrl_param_t pid_param[3];
//PID::ctrl_variable_t v_vel[3];
//PID *pid[3];

Servo servo(PB_2, false);
MD md(PC_9, PC_5, 1.0, false);

//Servo *servo;
//SpeedController *sc[8];
setting_struct_t *mdc_settings[8];

double c_1, c_2;
double updown;
double ougigataniagaruyatu;
double nobiruyatu;

//void modules();

static uint32_t gUpdateDate = 0;
static uint32_t gSentDate = 0;

inline void toggleAcknowledge() {
  acknowledge_0 = !acknowledge_0;
}

int main() {

  serial_control.add_frame(0, &msc);
  serial_control.add_frame(1, &movement_feedback_msg[0]);
  serial_control.add_frame(2, &movement_feedback_msg[1]);
  serial_control.add_frame(3, &pid_gain_msg);
  //  debug message instead of stdio
  serial_control.add_frame(10, &debug_msg);

  timer.start();

  //  set up
  ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_4MHz, 125UL * 1000UL, DataBitRateFactor::x8);

  settings.mRequestedMode = ACAN2517FDSettings::NormalFD;

  settings.mDriverTransmitFIFOSize = 8;
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

  void modules();

  setting_struct_t mdc_settings[8] = {
    //  Azure 0 -> no.0
      {OperatorMode::MD_OPERATOR,
       EncoderType::VELOCITY,
       ENCODER_REVOLUTION,
       false,
       1.7,
       0,
       0,
       0,
       0,
       0,
       0},
      //  Azure 0 -> no.1
      {OperatorMode::MD_OPERATOR,
       EncoderType::VELOCITY,
       ENCODER_REVOLUTION,
       false,
       1.9,
       0,
       0,
       0,
       0,
       0,
       0},
      //  Azure 0 -> no.2
      {OperatorMode::MD_OPERATOR,
       EncoderType::VELOCITY,
       ENCODER_REVOLUTION,
       false,
       1.8,
       0,
       0,
       0,
       0,
       0,
       0},
      //  Azure 0 -> no.3
      {OperatorMode::MD_OPERATOR,
       EncoderType::VELOCITY,
       ENCODER_REVOLUTION,
       false,
       1.9,
       0,
       0,
       0,
       0,
       0,
       0},
      //////////////////
      //  Azure 1 -> no.0
      {OperatorMode::MD_OPERATOR,
       EncoderType::VELOCITY,
       ENCODER_REVOLUTION,
       false,
       1.1,
       0,
       0,
       0,
       0,
       0,
       0},
      //  Azure 1 -> no.1
      {OperatorMode::MD_OPERATOR,
       EncoderType::VELOCITY,
       ENCODER_REVOLUTION,
       true,
       1.1,
       0,
       0,
       0,
       0,
       0,
       0},
      //  Azure 1 -> no.2
      {OperatorMode::MD_OPERATOR,
       EncoderType::VELOCITY,
       ENCODER_REVOLUTION,
       false,
       0.1,
       0,
       0,
       0,
       0,
       0,
       0},
      //  Azure 1 -> no.3
      {OperatorMode::MD_OPERATOR,
       EncoderType::VELOCITY,
       ENCODER_REVOLUTION,
       false,
       0.1,
       0,
       0,
       0,
       0,
       0,
       0}};

  wait_us(2000 * 1000);

  mdc_client.update_setting(0, mdc_settings[0]);
  wait_us(250 * 1000);
  mdc_client.update_setting(1, mdc_settings[1]);
  wait_us(250 * 1000);
  mdc_client.update_setting(2, mdc_settings[2]);
  wait_us(250 * 1000);
  mdc_client.update_setting(3, mdc_settings[3]);
  wait_us(250 * 1000);
  mdc_client_2.update_setting(0, mdc_settings[4]);
  wait_us(250 * 1000);
  mdc_client_2.update_setting(1, mdc_settings[5]);
  wait_us(250 * 1000);
  mdc_client_2.update_setting(2, mdc_settings[6]);
  wait_us(250 * 1000);
  mdc_client_2.update_setting(3, mdc_settings[7]);
  wait_us(250 * 1000);

  while (1) {

    serial_control.update();

    dev0_can.poll();

    if (pid_gain_msg.was_updated()) {
      uint8_t id = pid_gain_msg.data.id;
      if (0 <= id && id < 8) {
        //  update setting
        mdc_settings[id].kp = pid_gain_msg.data.gain.kp;
        mdc_settings[id].ki = pid_gain_msg.data.gain.ki;
        mdc_settings[id].kd = pid_gain_msg.data.gain.kd;
        mdc_settings[id].forward_gain = pid_gain_msg.data.gain.fg;

        if (id < 4) {
          mdc_client.update_setting(id, mdc_settings[id]);
        } else {
          mdc_client_2.update_setting(id - 4, mdc_settings[id]);
        }

        // DEBUG
        memset(debug_msg.data.str, 0, 64);
        sprintf(debug_msg.data.str, "gain: %d", id);
        serial_control.write(10);
      }
    }

    if (mdc_client.update()) {
      //  set target value
      movement_feedback_msg[0].data.target.a = mw.getSpeed(3);
      movement_feedback_msg[0].data.target.b = mw.getSpeed(2);
      movement_feedback_msg[0].data.target.c = mw.getSpeed(1);
      movement_feedback_msg[0].data.target.d = mw.getSpeed(0);

      //  set feedback value
      movement_feedback_msg[0].data.output.a = mdc_client.feedback.data.node[0].velocity;
      movement_feedback_msg[0].data.output.b = mdc_client.feedback.data.node[1].velocity;
      movement_feedback_msg[0].data.output.c = mdc_client.feedback.data.node[2].velocity;
      movement_feedback_msg[0].data.output.d = mdc_client.feedback.data.node[3].velocity;

      //  send
      serial_control.write(1);

      //  toggle led
      toggleAcknowledge();
    }

    if (mdc_client_2.update()) {
      //  set target value
      movement_feedback_msg[1].data.target.a = c_1;
      movement_feedback_msg[1].data.target.b = c_2;
      movement_feedback_msg[1].data.target.c = ougigataniagaruyatu;
      movement_feedback_msg[1].data.target.d = updown;

      //  set feedback value
      movement_feedback_msg[1].data.output.a = mdc_client.feedback.data.node[0].velocity;
      movement_feedback_msg[1].data.output.b = mdc_client.feedback.data.node[1].velocity;
      movement_feedback_msg[1].data.output.c = mdc_client.feedback.data.node[2].angle;
      movement_feedback_msg[1].data.output.d = mdc_client.feedback.data.node[3].angle;

      //  send
      serial_control.write(2);

      //  toggle led
      toggleAcknowledge();
    }

    if (msc.was_updated()) {

      led = !led;

      // DEBUG
      memset(debug_msg.data.str, 0, 64);
      sprintf(debug_msg.data.str, "0");
      serial_control.write(10);

      // Joystickの値を取得

      double joyLxValue = msc.data.Lx;
      double joyLyValue = msc.data.Ly;
      double joyRxValue = msc.data.Rx;
      double joyRyValue = msc.data.Ry;

      bool   joyL1Value = msc.data.L1;
      bool   joyR1Value = msc.data.R1;

      double joyL2Value = msc.data.L2;
      double joyR2Value = msc.data.R2;

      bool   triangle   = msc.data.triangle;
      bool   square     = msc.data.square;
      bool   circle     = msc.data.circle;
      bool   cross      = msc.data.cross;

      bool   lc_up      = msc.data.lc_up;
      bool   lc_down    = msc.data.lc_down;
      bool   lc_left    = msc.data.lc_left;
      bool   lc_right   = msc.data.lc_right;

      // ボタンの状態を取得(Lならマイナス,Rならプラス)
      double turn = joyRxValue * -1;

      // Joystickのベクトル化
      double joySpeed    = sqrt (joyLxValue * joyLxValue + joyLyValue * joyLyValue);
      double joyRotation = atan2(joyLyValue, joyLxValue) - (PI / 4);

      // targetSpeedが1,-1を超えないようにする
      if (joySpeed > 1) {
        joySpeed = 1;
      } else if (joySpeed < -1) {
        joySpeed = -1;
      }

      // targetSpeedが0.03以下の時に起動しないようにする
      if (joySpeed < 0.03 && joySpeed > -0.03) {
        joySpeed = 0;
      }

      // targetRotationがマイナスにならないように2πたす
      if (joyRotation < 0) {
        joyRotation += (2 * PI);
      }

      // 目標速度, 回転速度, 回転方向を設定
      mw.control(joySpeed, joyRotation, turn);

      
      // 上部展開[台形ネジ](49回転で伸びるよ)
      updown = (triangle - cross) * 1.0;

      // 上部周り(ougigataniagaruyatuは角度制御してもよし)
      ougigataniagaruyatu = ((lc_up * 0.15) - (lc_down * 0.1));
      nobiruyatu = (lc_left - lc_right) * 0.1;

      // キャタピラ
      c_1 = joyL2Value;
      c_2 = joyR2Value;

      // キャタピラ逆転用
      if(joyL1Value){
        c_1 *= -1;
      }

      if(joyR1Value){
        c_2 *= -1;
      }

      // サーボ(手) numは50~500くらいが無難
      servo.drive((square - circle) * 50);

      // 台形加速
      sc0.set_target(mw.getSpeed(3));
      sc1.set_target(mw.getSpeed(2));
      sc2.set_target(mw.getSpeed(1));
      sc3.set_target(mw.getSpeed(0));

      sc4.set_target(c_1);
      sc5.set_target(c_2);

      sc6.set_target(ougigataniagaruyatu);
      sc7.set_target(updown);


      // Azusa[0] 送信値設定

      // メカナム
      mdc_client.set_target(0, sc0.step() * -1);
      mdc_client.set_target(1, sc1.step());
      mdc_client.set_target(2, sc2.step() * -1);
      mdc_client.set_target(3, sc3.step());

      // mdc_client.set_target(0, mw.getSpeed(3) * -1);
      // mdc_client.set_target(1, mw.getSpeed(2));
      // mdc_client.set_target(2, mw.getSpeed(1) * -1);
      // mdc_client.set_target(3, mw.getSpeed(0));

      // Azusa[1] 送信値設定

      // キャタピラ
      mdc_client_2.set_target(0, sc4.step());
      mdc_client_2.set_target(1, sc5.step());

      //  扇型にあがるやつ
      mdc_client_2.set_target(2, ougigataniagaruyatu);
      // mdc_client_2.set_target(2, sc6.step());

      //  上部展開[台形ネジ]
      mdc_client_2.set_target(3, updown);
      // mdc_client_2.set_target(3, sc7.step());

      //  伸びるやつ
      md.drive(nobiruyatu);

      //  Azusa 送信
      mdc_client.send_target();
      mdc_client_2.send_target();
    }

    serial.update();

    // 周期調整用 (ROSで調整済み[10ms])
    // ThisThread::sleep_for(70);
  }
}
