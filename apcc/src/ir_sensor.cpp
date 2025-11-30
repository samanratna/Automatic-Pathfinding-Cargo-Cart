#include "ir_sensor.h"
#include "main.h"

void ir_setup() {

    pinMode(IR_RIGHT_1, INPUT);
    pinMode(IR_RIGHT_2, INPUT);
    pinMode(IR_RIGHT_3, INPUT);

    pinMode(IR_LEFT_1, INPUT);
    pinMode(IR_LEFT_2, INPUT);
    pinMode(IR_LEFT_3, INPUT);
}

void ir_reading_test() {

    Serial.print("\n");Serial.print(digitalRead(IR_LEFT_1));
    Serial.print(" ");Serial.print(digitalRead(IR_LEFT_2));
    Serial.print(" ");Serial.print(digitalRead(IR_LEFT_3));

    Serial.print(" | ");Serial.print(digitalRead(IR_RIGHT_1));
    Serial.print(" ");Serial.print(digitalRead(IR_RIGHT_2));
    Serial.print(" ");Serial.print(digitalRead(IR_RIGHT_3));
}

uint8_t run_line_following_logic()
{
    uint8_t left_ir_1 = digitalRead(IR_LEFT_1);
    uint8_t left_ir_2 = digitalRead(IR_LEFT_2);
    uint8_t left_ir_3 = digitalRead(IR_LEFT_3);

    uint8_t left_ir_weight = left_ir_1 + left_ir_2 + left_ir_3;

    uint8_t right_ir_1 = digitalRead(IR_RIGHT_1);
    uint8_t right_ir_2 = digitalRead(IR_RIGHT_2);
    uint8_t right_ir_3 = digitalRead(IR_RIGHT_3);

    uint8_t right_ir_weight = right_ir_1 + right_ir_2 + right_ir_3;

    if (left_ir_weight > right_ir_weight)
    {
        return LEFT;
    }
    else if (right_ir_weight > left_ir_weight)
    {
        return RIGHT;
    }
    else
    {
        return FORWARD;
    }
}