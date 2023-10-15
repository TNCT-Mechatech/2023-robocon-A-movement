#ifndef _SERVO_HPP_
#define _SERVO_HPP_

#include "mbed.h"
#include "math.h"

class Servo
{
  private:
    double _servo;
    PwmOut servo;
    bool reverse;

  public:
  Servo(PinName Servo_pin, bool reverse)
  : servo(Servo_pin), reverse(reverse), _servo(0)
  {
    servo.period(0.003);
  }

  void adjust()
  {
    if(_servo > (0.81)){
        _servo = (0.81);
    }else if(_servo < (0.18)){
        _servo = (0.18);
    }
  }

// num分足す 高い値入れたらダメ

  void drive(double num)
  {
    _servo += num;

    adjust();

    if(reverse){
      _servo *= -1;
    }

    servo = _servo;
  }
};

#endif