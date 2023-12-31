#ifndef _MECANUM_HPP_
#define _MECANUM_HPP_

#include "mbed.h"
#include "math.h"

class MecanumWheel {

private:

    double _wheel[4];

public:

    MecanumWheel()
    {};


    void control(double _joySpeed, double _joyRotation, double _turn)
    {
        _wheel[0] = cos(_joyRotation) * _joySpeed - _turn;
        _wheel[1] = sin(_joyRotation) * _joySpeed + _turn;
        _wheel[2] = sin(_joyRotation) * _joySpeed - _turn;
        _wheel[3] = cos(_joyRotation) * _joySpeed + _turn;
    }


    double getSpeed(int unit) {
        if(unit < 0 || unit > 3) {
            return 0.0;
        }
        return _wheel[unit];
    }
};

#endif
