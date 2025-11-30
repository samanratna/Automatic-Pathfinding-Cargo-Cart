#ifndef IRSENSOR_H
#define IRSENSOR_H

#include <Arduino.h>

// Pin definitions
#define IR_RIGHT_1 A0
#define IR_RIGHT_2 A1
#define IR_RIGHT_3 A2

#define IR_LEFT_1 A3
#define IR_LEFT_2 A4   
#define IR_LEFT_3 A5


void ir_setup(void);
void ir_reading_test(void);
uint8_t run_line_following_logic(void);

#endif
