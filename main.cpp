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

#include "Encoder.hpp"
#include "MD.hpp"
#include "PID.hpp"


Timer timer;
double pre_timer = 0.01;

#define ENCODER_REVOLUTION 4647

using namespace acan2517fd;

const double PI = 3.1415;

uint32_t getMillisecond() {
  return (uint32_t) duration_cast<std::chrono::milliseconds>(
    timer.elapsed_time())
    .count();
}

uint32_t getMicrosecond() {
  return (uint32_t) duration_cast<std::chrono::microseconds>(
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
MovementFeedback movement_feedback_msg[2];
PIDGain pid_gain_msg;

DigitalOut led(PA_5);

MecanumWheel mw;

MD *md[4];
//Encoder *encoder[3];
//PID::ctrl_param_t pid_param[3];
//PID::ctrl_variable_t v_vel[3];
//PID *pid[3];

Servo *servo;
SpeedController *sc[8];
setting_struct_t *mdc_settings[8];

double c_1, c_2;
double updown;

void modules();

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

  void modules();

  setting_struct_t mdc_settings[8] = {
    //  Azure 0 -> no.0
      {OperatorMode::PID_OPERATOR,
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
      {OperatorMode::PID_OPERATOR,
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
      {OperatorMode::PID_OPERATOR,
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
      {OperatorMode::PID_OPERATOR,
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
      {OperatorMode::NO_OPERATOR,
       EncoderType::ANGLE,
       ENCODER_REVOLUTION,
       false,
       1.1,
       0,
       0,
       0,
       0,
       0,
       0},
      //  Azure 1 -> no.3
      {OperatorMode::NO_OPERATOR,
       EncoderType::ANGLE,
       ENCODER_REVOLUTION,
       false,
       1.1,
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
      movement_feedback_msg[1].data.target.c = updown;
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

      // Joystickの値を取得(値域を+0.5から±1にする)

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

      bool lc_up = msc.data.lc_up;
      bool lc_down = msc.data.lc_down;
      bool lc_left = msc.data.lc_left;
      bool lc_right = msc.data.lc_right;

      uint32_t t_ = getMicrosecond();

      // ボタンの状態を取得(Lならマイナス,Rならプラス)
      double turn = joyRxValue * -1;

      // Joystickのベクトル化
      double joySpeed = sqrt(joyLxValue * joyLxValue + joyLyValue * joyLyValue);
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
      mw.control(joySpeed * 2, joyRotation, turn);

      // 上部展開(49)
      updown = (triangle - cross) * 49;

      // 上部周り
      double oogigataniagaruyatu = (lc_up - lc_down) * 0.7;
      double nobiruyatu = (lc_left - lc_right) * 3.5;

      servo->drive((square - circle) * 50);

      sc[0]->drive(mw.getSpeed(3), true);
      sc[1]->drive(mw.getSpeed(2), false);
      sc[2]->drive(mw.getSpeed(1), true);
      sc[3]->drive(mw.getSpeed(0), false);

      sc[4]->drive(joyL2Value, joyL1Value);
      sc[5]->drive(joyR2Value, joyR1Value);

      sc[6]->drive(updown, false);
      sc[7]->drive(updown, false);

      mdc_client.set_target(0, sc[0]->return_Speed());
      mdc_client.set_target(1, sc[1]->return_Speed());
      mdc_client.set_target(2, sc[2]->return_Speed());
      mdc_client.set_target(3, sc[3]->return_Speed());

      mdc_client_2.set_target(0, sc[4]->return_Speed());
      mdc_client_2.set_target(1, sc[5]->return_Speed());
      mdc_client_2.set_target(2, sc[6]->return_Speed());
      mdc_client_2.set_target(3, sc[7]->return_Speed());

      mdc_client.send_target();
      mdc_client_2.send_target();
    }

    serial.update();

    gSentDate = getMillisecond();

    // 周期調整用 (ここを変えるならDELTA_Tも変える)
    // ThisThread::sleep_for(70ms);
  }
}

void modules() {

  // 台形加速
  sc[0] = new SpeedController(0.01);
  sc[1] = new SpeedController(0.01);
  sc[2] = new SpeedController(0.01);
  sc[3] = new SpeedController(0.01);
  sc[4] = new SpeedController(0.01);
  sc[5] = new SpeedController(0.01);
  sc[6] = new SpeedController(0.1);
  sc[7] = new SpeedController(0.1);

  //  pid gain
//  pid_param[0] = PID::ctrl_param_t {
//    0.2,
//    0.0,
//    0.0,
//    0.125,
//    false
//  };
//  pid_param[1] = PID::ctrl_param_t {
//    0.2,
//    0.0,
//    0.0,
//    0.125,
//    false
//  };
//  pid_param[2] = PID::ctrl_param_t {
//    0.2,
//    0.0,
//    0.0,
//    0.125,
//    false
//  };
//
//  // MD用PIDゲイン調整 {kp(比例), ki(積分), kd(微分), reverse(逆転)}
//  pid[0] = new PID(v_vel[0], pid_param[0]);
//  pid[1] = new PID(v_vel[1], pid_param[1]);
//  pid[2] = new PID(v_vel[2], pid_param[2]);
//
//  // エンコーダーの制御ピン (a, b)
//  encoder[0] = new Encoder(PB_2,  PC_6, ?);
//  encoder[1] = new Encoder(PB_10, PA_8, ?);
//  encoder[2] = new Encoder(PA_9,  PC_7, ?);

  // MDの制御ピン (PWMピン, DIRピン, 逆転モード)
  md[0] = new MD(PC_9, PC_5, 1.0, false);
  md[1] = new MD(PC_8, PC_4, 1.0, false);
  md[2] = new MD(PA_0, PA_6, 1.0, false);
  md[3] = new MD(PA_1, PA_7, 1.0, false);

  // servo (PWMピン, 逆転モード)
  servo = new Servo(PB_0, false);
};