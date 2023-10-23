#ifndef _SPEEDCONTROLLER_HPP_
#define _SPEEDCONTROLLER_HPP_

#include "mbed.h"
#include "math.h"

class SpeedController {

private:

  double num;
  double feedback;
  bool reverse;

public:

  SpeedController(double num)
  : num(num), feedback(0)
  {};

  void drive(double targetSpeed, bool reverse)
  {
    if(reverse){
      targetSpeed *= -1;
    }

    if(targetSpeed > feedback){
      feedback += num;
    }else if(targetSpeed < feedback){
      feedback -= num;
    }
  };

  double return_Speed()
  {
    return feedback;
  };
};

#endif
