#ifndef _PID_GAIN_HPP_
#define _PID_GAIN_HPP_

#include <stdint.h>
#include <Message.hpp>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float fg;
} gain_t;

typedef struct PIDGainType
{
    uint8_t id;
    gain_t gain;
} pid_gain_t;

//  create message
typedef sb::Message<pid_gain_t> PIDGain;

#endif