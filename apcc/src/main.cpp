#include <Arduino.h>

uint8_t speed_constant = 70;
uint8_t speed_initial = 0;

const int delay_after_movement = 2000; // milliseconds

// motor definitions
const uint8_t MOTOR_RIGHT_A = 8;
const uint8_t MOTOR_RIGHT_B = 10;
const uint8_t MOTOR_LEFT_A = 15;
const uint8_t MOTOR_LEFT_B = 17;

// pwm definitions
const uint8_t MOTOR_RIGHT_A_PWM = 2;
const uint8_t MOTOR_RIGHT_B_PWM = 3;
const uint8_t MOTOR_LEFT_A_PWM = 4;
const uint8_t MOTOR_LEFT_B_PWM = 5;


// function prototypes
void motor_setup(void);
void forward(byte spd);
void backward(byte spd);
void left(byte spd);
void right(byte spd);
void stop(void);

// pin definitions
void setup() {

  motor_setup();
  Serial.begin(9600);
}


void loop() {

  forward(speed_constant);
  delay(delay_after_movement);
  stop();
  delay(delay_after_movement);

  // backward(speed_constant);
  // delay(delay_after_movement);
  // stop();
  // delay(delay_after_movement);

  left(speed_constant);
  delay(delay_after_movement);
  stop();
  delay(delay_after_movement);

  // right(speed_constant);
  // delay(delay_after_movement);
  // stop();
  // delay(delay_after_movement);
  
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

void forward(byte spd)
{
  Serial.println("Moving Forward");

  analogWrite(MOTOR_RIGHT_A_PWM, spd);
  analogWrite(MOTOR_RIGHT_B_PWM, spd);
  analogWrite(MOTOR_LEFT_A_PWM, spd);
  analogWrite(MOTOR_LEFT_B_PWM, spd);

  digitalWrite(MOTOR_RIGHT_A, HIGH);
  digitalWrite(MOTOR_RIGHT_B, LOW);
  digitalWrite(MOTOR_LEFT_A, HIGH);
  digitalWrite(MOTOR_LEFT_B, LOW);
}


void backward(byte spd)
{
  Serial.println("Moving Backward");

  analogWrite(MOTOR_RIGHT_A_PWM, spd);
  analogWrite(MOTOR_RIGHT_B_PWM, spd);
  analogWrite(MOTOR_LEFT_A_PWM, spd);
  analogWrite(MOTOR_LEFT_B_PWM, spd);

  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, HIGH);
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, HIGH);
}

void left(byte spd)
{
  Serial.println("Turning Left");

  analogWrite(MOTOR_RIGHT_A_PWM, spd);
  analogWrite(MOTOR_RIGHT_B_PWM, spd);
  analogWrite(MOTOR_LEFT_A_PWM, spd);
  analogWrite(MOTOR_LEFT_B_PWM, spd);

  digitalWrite(MOTOR_RIGHT_A, HIGH);
  digitalWrite(MOTOR_RIGHT_B, LOW);
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, HIGH);
}

void right(byte spd)
{
  Serial.println("Turning Right");

  analogWrite(MOTOR_RIGHT_A_PWM, spd);
  analogWrite(MOTOR_RIGHT_B_PWM, spd);
  analogWrite(MOTOR_LEFT_A_PWM, spd);
  analogWrite(MOTOR_LEFT_B_PWM, spd);

  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, HIGH);
  digitalWrite(MOTOR_LEFT_A, HIGH);
  digitalWrite(MOTOR_LEFT_B, LOW);
}

void stop()
{
  Serial.println("Stopping Motors");

  analogWrite(MOTOR_RIGHT_A_PWM, 0);
  analogWrite(MOTOR_RIGHT_B_PWM, 0);
  analogWrite(MOTOR_LEFT_A_PWM, 0);
  analogWrite(MOTOR_LEFT_B_PWM, 0);

  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, LOW);
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, LOW);
}