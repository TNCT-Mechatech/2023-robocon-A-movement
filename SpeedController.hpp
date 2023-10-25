#ifndef _SPEEDCONTROLLER_HPP_
#define _SPEEDCONTROLLER_HPP_

#include "mbed.h"
#include "math.h"
#include "cstdio"

class SpeedController {

private:

  double _increment;
  double _target;
  double _feedback;
  double _width;

public:

    /**
     * コンストラクター
     * @param increment １ループで増減する値
     * @param width 許容する値（荒ぶり防止）
     */
  SpeedController(double increment, double width)
  : _increment(increment), _width(width), _feedback(0), _target(0)
  {}

  void set_target(double target)
  {
      _target = target;
  }

  double step()
  {
      if (std::abs(_target - _feedback) < _width) {
          _feedback = _target;
      }
      else if((_target - _feedback) > 0)
      {
          _feedback += _increment;
      }
      else {
          _feedback -= _increment;
      }

      return _feedback;
  }
};

#endif
