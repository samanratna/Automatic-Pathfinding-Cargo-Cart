#include "distance_sensor.h"

void ultrasonic_setup() {
  pinMode(US1_TRIG, OUTPUT);
  pinMode(US1_ECHO, INPUT);

  Serial.println("Ultrasonic sensors initialized...");
}

long getDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000UL); // 30 ms timeout (~5m)
  if (duration == 0) return -1;

  return duration / 58; // convert microseconds to cm
}

long getDistanceForLineFollowing() {
  
  // uint8_t cumulation_count = 0;
  // long distance_sum = 0;

  // for (int i = 0; i < 5; i++)
  // {
  //   long raw_distance = getDistanceCM(US1_TRIG, US1_ECHO);
  //   if (raw_distance >= 0)
  //   {
  //     cumulation_count = cumulation_count + 1;
  //     distance_sum = distance_sum + raw_distance;
  //   }
  // }

  // long average_distance = distance_sum / cumulation_count;
  // return average_distance;

  long raw_distance = getDistanceCM(US1_TRIG, US1_ECHO);
  return raw_distance;
}
