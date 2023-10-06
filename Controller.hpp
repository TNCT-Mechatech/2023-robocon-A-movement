#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <Message.hpp>

typedef struct ControllerType {
  double Lx;
  double Rx;
  double Ly;
  double Ry;

  bool square;
  bool cross;
  bool circle;
  bool triangle;

  bool L1;
  bool L2;
  bool R1;
  bool R2;

  bool lc_up;
  bool lc_down;
  bool lc_left;
  bool lc_right;
} controller_t;

//  create message
typedef sb::Message <controller_t> Controller;

#endif