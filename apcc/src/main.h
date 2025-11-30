#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>


enum MotorState {
  STOPPED,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT
};

enum Action {
  accelerate,
  decelerate,
  maintain
};

#endif
