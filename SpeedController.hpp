#ifndef _SPEEDCONTROLLER_HPP_
#define _SPEEDCONTROLLER_HPP_

#include "mbed.h"
#include "math.h"

class SpeedController {

private:

  double num;
  double feedback;

public:

  SpeedController(double num)
  : num(num), feedback(0)
  {};

  void drive(
    double targetSpeed,
    bool reverse
    )
  {
    if(targetSpeed > behind){
      feedback += num;
    }else if(targetSpeed < bihind){
      feedback -= num;
    }
  };

  double return_Speed()
  {
    return feedback;
  };



};

#endif
