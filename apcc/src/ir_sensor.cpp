#include "ir_sensor.h"

void ir_setup() {

    pinMode(IR_RIGHT_1, INPUT);
    pinMode(IR_RIGHT_2, INPUT);
    pinMode(IR_RIGHT_3, INPUT);
    pinMode(IR_RIGHT_4, INPUT);
    pinMode(IR_RIGHT_5, INPUT);
    pinMode(IR_RIGHT_6, INPUT);
    pinMode(IR_RIGHT_7, INPUT);

    pinMode(IR_LEFT_1, INPUT);
    pinMode(IR_LEFT_2, INPUT);
    pinMode(IR_LEFT_3, INPUT);
    pinMode(IR_LEFT_4, INPUT);
    pinMode(IR_LEFT_5, INPUT);
    pinMode(IR_LEFT_6, INPUT);
    pinMode(IR_LEFT_7, INPUT);
}

void ir_reading_test() {

    Serial.println("IR Sensors Read Test Completed");
    Serial.print("\n");Serial.print(digitalRead(IR_LEFT_1));
    Serial.print(" ");Serial.print(digitalRead(IR_LEFT_2));
    Serial.print(" ");Serial.print(digitalRead(IR_LEFT_3));
    Serial.print(" ");Serial.print(digitalRead(IR_LEFT_4));
    Serial.print(" ");Serial.print(digitalRead(IR_LEFT_5));
    Serial.print(" ");Serial.print(digitalRead(IR_LEFT_6));
    Serial.print(" ");Serial.print(digitalRead(IR_LEFT_7));
}