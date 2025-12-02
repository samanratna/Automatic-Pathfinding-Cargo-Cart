#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

// === Ultrasonic sensor pin definitions ===
#define US1_TRIG 5
#define US1_ECHO 4

// === Constants ===
#define OBSTACLE_THRESHOLD_CM 10

// === Function declarations ===
void ultrasonic_setup(void);
long getDistanceCM(int trigPin, int echoPin);
long getDistanceForLineFollowing();

#endif
