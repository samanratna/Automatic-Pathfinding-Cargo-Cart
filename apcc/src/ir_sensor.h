#ifndef IRSENSOR_H
#define IRSENSOR_H

#include <Arduino.h>

// Pin definitions
#define IR_RIGHT_1 A1
#define IR_RIGHT_2 A2
#define IR_RIGHT_3 A3
#define IR_RIGHT_4 A4
#define IR_RIGHT_5 A5
#define IR_RIGHT_6 A6
#define IR_RIGHT_7 A7

#define IR_LEFT_1 A9
#define IR_LEFT_2 A10
#define IR_LEFT_3 A11
#define IR_LEFT_4 A12
#define IR_LEFT_5 A13
#define IR_LEFT_6 A14
#define IR_LEFT_7 A15



void ir_setup(void);
int ir_test(void);
byte start_ir_reading(void);

#endif
