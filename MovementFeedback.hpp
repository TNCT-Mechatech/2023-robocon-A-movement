#ifndef _MOVEMENT_FEEDACK_HPP_
#define _MOVEMENT_FEEDACK_HPP_

#include <Message.hpp>

typedef struct Vector4Type
{
    float a;
    float b;
    float c;
    float d;
} vector4_t;

typedef struct MovementFeedbackType
{
    //  target
    vector4_t target;
    //  output
    vector4_t output;
} movement_feedback_t;

//  create message
typedef sb::Message<movement_feedback_t> MovementFeedback;

#endif