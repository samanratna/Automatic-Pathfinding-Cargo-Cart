#include <Arduino.h>
#include <SoftwareSerial.h>
#include "main.h"
#include "ir_sensor.h"
#include "rfid_reader.h"
#include "distance_sensor.h"

// test flags
bool motor_testing = 0;
bool ir_testing = 1;
bool rfid_testing = 1;
bool distance_sensor_testing = 1;
bool bluetooth_testing = 0;
bool obstacle_avoidance_testing = 0;
/// @brief /////

// States for obstacle avoidance
#define FIRST_LEFT 800
#define FIRST_FORWARD 1000
#define FIRST_RIGHT 800
#define SECOND_FORWARD 1000
#define SECOND_RIGHT 800
#define THIRD_FORWARD 1000
#define SECOND_LEFT 800

int _first_left = FIRST_LEFT;
int _first_forward = _first_left + FIRST_FORWARD;
int _first_right = _first_forward + FIRST_RIGHT;
int _second_forward = _first_right + SECOND_FORWARD;
int _second_right = _second_forward + SECOND_RIGHT;
int _third_forward = _second_right + THIRD_FORWARD;
int _second_left = _third_forward + SECOND_LEFT;

uint8_t speed_constant = 80;
uint8_t speed_constant_obstacle = 150;
uint8_t speed_initial = 0;

uint8_t current_speed = 0;

uint8_t acceleration_step = 5;
uint8_t deceleration_step = 10;

// ir variables
const uint8_t SensorCount = 14;
uint16_t sensorValues[SensorCount];

uint8_t _state = STOPPED;

const int delay_after_movement = 2000; // milliseconds

// motor definitions
const uint8_t MOTOR_RIGHT_A = 8;
const uint8_t MOTOR_RIGHT_B = 10;
const uint8_t MOTOR_LEFT_A = 15;
const uint8_t MOTOR_LEFT_B = 17;

// pwm definitions
const uint8_t MOTOR_RIGHT_A_PWM = 2;  
const uint8_t MOTOR_RIGHT_B_PWM = 3;
// const uint8_t MOTOR_LEFT_A_PWM = 4;
// const uint8_t MOTOR_LEFT_B_PWM = 5;

long time_elapsed = 0;
long start_time = 0;
long current_time = 0;

uint8_t detected_obstacle = 0;
long obs_current_time = 0;
long obs_start_time = 0;


// function prototypes
void motor_setup(void);
void forward(byte spd, uint8_t action);
void backward(byte sp, uint8_t action);
void left(byte spd, uint8_t action);
void right(byte spd, uint8_t action);
void stop(void);
void setSpeed(uint8_t speed);
void moveForward(void);
void moveBackward(void);
void moveLeft(void);
void moveRight(void);
void motorStop(void);
uint8_t getCurrentSpeed(byte spd, uint8_t action);

// pin definitions
void setup() {
  
  Serial.begin(9600);
  start_time = millis();


  // Bluetooth Serial setup
  Serial1.begin(9600);         // For HC-05 communication


  motor_setup();
  ir_setup();
  rfid_setup();
  ultrasonic_setup();
}


void loop() {

  // Movement sequence
  if (motor_testing == true){

    current_time = millis();
    time_elapsed = current_time - start_time;
  
    if (time_elapsed <= 5000){
      forward(speed_constant, accelerate);
    } else if (time_elapsed > 5000 && time_elapsed <= 10000){
      left(speed_constant, accelerate);
    } else if (time_elapsed > 10000 && time_elapsed <= 15000){
      stop();
    } else if(time_elapsed > 15000 && time_elapsed <= 20000){
      right(speed_constant, accelerate);
    } else {
      stop();
    }
  }

  if (bluetooth_testing == true){

    Serial.println("...");
    // Bluetooth → Serial Monitor
    if (Serial1.available()) {
      Serial.print("Bluetooth Serial is available");

      char inputvalue = char(Serial1.read());
      Serial.println(inputvalue);

      if (inputvalue == 'F') {
        forward(150, accelerate);
      }
      else if (inputvalue == 'B') {
        backward(150, accelerate);
      }

      else if (inputvalue == 'L') {
        left(150, accelerate);
      }

      else if (inputvalue == 'R') {
        right(150, accelerate);
      }

      else if (inputvalue == 'S') {
        motorStop();
      }
    }
  }

  // left ir sensor reading test
  if (ir_testing == true){
    Serial.println("test1");
    uint8_t movement_state = run_line_following_logic();
    if (movement_state == LEFT){
      left(speed_constant, maintain);
    } else if (movement_state == RIGHT){
      right(speed_constant-10, maintain);
    } else if (movement_state == FORWARD){
      forward(speed_constant-10, maintain);
    }
  }

  // obstacle avoidance test
  if (obstacle_avoidance_testing == true)
  {

    long distance_cm = getDistanceForLineFollowing();



    // if (distance_cm > 0 && distance_cm < OBSTACLE_THRESHOLD_CM) {

    //   left(speed_constant_obstacle, maintain);
    //   delay(1100);
    //   forward(speed_constant_obstacle, maintain);
    //   delay(1100);
    //   right(speed_constant_obstacle, maintain);
    //   delay(1100);
    //   forward(speed_constant_obstacle, maintain);
    //   delay(2600);
    //   right(speed_constant_obstacle, maintain);
    //   delay(1100);
    //   forward(speed_constant_obstacle, maintain);
    //   delay(1100);
    //   left(speed_constant_obstacle, maintain);
    //   delay(1100);
    // } else {
    //   forward(speed_constant_obstacle, maintain);
    // }

    Serial.print("Distance (cm): "); Serial.println(distance_cm);

    if (distance_cm < 0 || distance_cm > OBSTACLE_THRESHOLD_CM)
    {
      forward(speed_constant_obstacle, maintain);
    }

    else if (distance_cm > 0 && distance_cm < OBSTACLE_THRESHOLD_CM)
    {
      if (detected_obstacle == false){
          long obs_start_time = millis();
      }
      detected_obstacle = true;
    }

    if (detected_obstacle == true)
    {
      Serial.println("Obstacle Avoidance Maneuver Initiated");
      long obs_current_time = millis();
      time_elapsed = obs_current_time - obs_start_time;

      if (time_elapsed >= 0 && time_elapsed < _first_left)
      {
        left(speed_constant_obstacle, maintain);
      }
      else if (time_elapsed >= _first_left && time_elapsed < _first_forward)
      {
        {
          forward(speed_constant_obstacle, maintain);
        }
      }
      else if (time_elapsed >= _first_forward && time_elapsed < _first_right)
      {
        right(speed_constant_obstacle, maintain);
      }
      else if (time_elapsed >= _first_right && time_elapsed < _second_forward)
      {
        {
          forward(speed_constant_obstacle, maintain);
        }
      } 
      else if (time_elapsed >= _second_forward && time_elapsed < _second_right)
      {
        right(speed_constant_obstacle, maintain);
      }
      else if (time_elapsed >= _second_right && time_elapsed < _third_forward)
      {
        {
          forward(speed_constant_obstacle, maintain);
        }
      } else if (time_elapsed >= _third_forward && time_elapsed < _second_left)
      {
        left(speed_constant_obstacle, maintain);
      } else if (time_elapsed >= _second_left)
      {
        detected_obstacle = false;
        obs_start_time = 0;
        obs_current_time = 0;
      }
    }
  }

  // Distance sensor reading test
  if (distance_sensor_testing == true)
  {
    long distance_cm = getDistanceForLineFollowing(); 
    Serial.print("Distance (cm): "); Serial.println(distance_cm);

    if (distance_cm > 0 && distance_cm < OBSTACLE_THRESHOLD_CM)
    {
      speed_constant = 55;        // decreased the speed here
      Serial.println("Obstacle detected within threshold!");

      // RFID reading test
      if (rfid_testing == true)
      {
        uint8_t tag_value = getRFIDTagValue();
        Serial.print("Tag Value: "); Serial.println(tag_value);

        if (tag_value == 1)
        {
          rfid_testing = false;
          ir_testing = false;
          distance_sensor_testing = false;

          speed_constant = 60;

          // routine to execute upon detecting tag 1 i.e. turn right
          backward(speed_constant, maintain);
          delay(3000);
          right(speed_constant, maintain);
          delay(3000);
          forward(speed_constant, maintain);
          delay(2000);
        }
        else if (tag_value == 2)
        {
          rfid_testing = false;
          ir_testing = false;
          distance_sensor_testing = false;

          // routine to execute upon detecting tag 2 i.e. turn left
          speed_constant = 60;
          backward(speed_constant, maintain);
          delay(3000);
          left(speed_constant, maintain);
          delay(3000);
          forward(speed_constant, maintain);
          delay(2000);
        }
        
      }
    }
  }

}

void motor_setup() {

  pinMode(8, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(17, OUTPUT);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
}

uint8_t getCurrentSpeed(byte spd, uint8_t action) {
  if (action == accelerate) {
    if (current_speed <= spd) {
      current_speed += acceleration_step;
      if (current_speed > spd) {
        current_speed = spd;
      }
    }
  } else if (action == decelerate) {
    if (current_speed > spd) {
      current_speed -= deceleration_step;
      if (current_speed < spd) {
        current_speed = spd;
      }
    }
  } else if (action == maintain) {
    current_speed = spd;
  }
  return current_speed;
}

void forward(byte spd, uint8_t action)
{
  Serial.println("Moving Forward");
  // Serial.print("Current Speed: "); Serial.println(current_speed);

  _state = FORWARD;

  current_speed = getCurrentSpeed(spd, action);

  setSpeed(current_speed);
  moveForward();
}

void backward(byte spd, uint8_t action)
{
  Serial.println("Moving Backward");
  // Serial.print("Current Speed: "); Serial.println(current_speed);

  _state = BACKWARD;

  current_speed = getCurrentSpeed(spd, action);

  setSpeed(current_speed);
  moveBackward();
}

void left(byte spd, uint8_t action)
{
  Serial.println("Turning Left");
  // Serial.print("Current Speed: "); Serial.println(current_speed);

  _state = LEFT;

  current_speed = getCurrentSpeed(spd, action);

  setSpeed(current_speed);
  moveLeft();
}

void right(byte spd, uint8_t action)
{
  Serial.println("Turning Right");
  // Serial.print("Current Speed: "); Serial.println(current_speed);

  _state = RIGHT;

  current_speed = getCurrentSpeed(spd, action);

  setSpeed(current_speed);
  moveRight();
}

void moveForward() {
  digitalWrite(MOTOR_RIGHT_A, HIGH);
  digitalWrite(MOTOR_RIGHT_B, LOW);
  digitalWrite(MOTOR_LEFT_A, HIGH);
  digitalWrite(MOTOR_LEFT_B, LOW);
}

void moveBackward() {
  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, HIGH);
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, HIGH);
}

void moveLeft() {
  digitalWrite(MOTOR_RIGHT_A, HIGH);
  digitalWrite(MOTOR_RIGHT_B, LOW);
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, HIGH);
}

void moveRight() {
  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, HIGH);
  digitalWrite(MOTOR_LEFT_A, HIGH);
  digitalWrite(MOTOR_LEFT_B, LOW);
}

void setSpeed(uint8_t speed) {
  analogWrite(MOTOR_RIGHT_A_PWM, speed);
  analogWrite(MOTOR_RIGHT_B_PWM, speed);
  // analogWrite(MOTOR_LEFT_A_PWM, speed);
  // analogWrite(MOTOR_LEFT_B_PWM, speed);
}


void backward(byte spd)
{
  // Serial.println("Moving Backward");

  analogWrite(MOTOR_RIGHT_A_PWM, spd);
  analogWrite(MOTOR_RIGHT_B_PWM, spd);
  // analogWrite(MOTOR_LEFT_A_PWM, spd);
  // analogWrite(MOTOR_LEFT_B_PWM, spd);

  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, HIGH);
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, HIGH);
}

void left(byte spd)
{
  // Serial.println("Turning Left");

  analogWrite(MOTOR_RIGHT_A_PWM, spd);
  analogWrite(MOTOR_RIGHT_B_PWM, spd);
  // analogWrite(MOTOR_LEFT_A_PWM, spd);
  // analogWrite(MOTOR_LEFT_B_PWM, spd);

  digitalWrite(MOTOR_RIGHT_A, HIGH);
  digitalWrite(MOTOR_RIGHT_B, LOW);
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, HIGH);
}

void right(byte spd)
{
  // Serial.println("Turning Right");

  analogWrite(MOTOR_RIGHT_A_PWM, spd);
  analogWrite(MOTOR_RIGHT_B_PWM, spd);
  // analogWrite(MOTOR_LEFT_A_PWM, spd);
  // analogWrite(MOTOR_LEFT_B_PWM, spd);

  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, HIGH);
  digitalWrite(MOTOR_LEFT_A, HIGH);
  digitalWrite(MOTOR_LEFT_B, LOW);
}

void stop()
{
  Serial.println("Stopping Motors");

  if (_state == FORWARD){
    forward(0, decelerate);
  } else if (_state == BACKWARD){
    backward(0, decelerate);
  } else if (_state == LEFT){
    left(0, decelerate);
  } else if (_state == RIGHT){
    right(0, decelerate);
  }
}

void motorStop(void)
{
  // Serial.println("Turning Left");

  // analogWrite(MOTOR_RIGHT_A_PWM, spd);
  // analogWrite(MOTOR_RIGHT_B_PWM, spd);
  // analogWrite(MOTOR_LEFT_A_PWM, spd);
  // analogWrite(MOTOR_LEFT_B_PWM, spd);

  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, LOW);
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, LOW);
}