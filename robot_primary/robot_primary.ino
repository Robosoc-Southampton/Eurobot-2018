
#include <Servo.h>
#include "driver.h"
#include "UltraSonic.h"
#include "button.h"
#include "LED.h"

#define pin1 30
#define pin2 31
#define pin3 32
#define pin4 33
#define delaytime 5

#define LAUNCHER_SPEED_PIN 8 // ENA - PWM
#define LAUNCHER_DIR_PIN_1 25 // IN1
#define LAUNCHER_DIR_PIN_2 24 // IN2

#define DESTICKIFIER_SPEED_PIN 9 // ENB - PWM
#define DESTICKIFIER_DIR_PIN_1 23 // IN3
#define DESTICKIFIER_DIR_PIN_2 22 // IN4

#define OPENING_SERVO_PIN 10

const int maxServoPosition = 60;
int currentServoPosition = 0;

const float Pp = 0.5; // 0.5
const float Pi = 0;
const float Pd = 0;
const float Pp_t = 0.6; // 0.6
const float Pi_t = 0;
const float Pd_t = 0;

const int limit_correction = 15; // (min value of 15)
const int limit_correction_turning = 90;

const int circumference = 314; // [mm]
const int wheel_dist = 329; // [mm] initialy 235

UltraSonic sensor1(12, 11);
UltraSonic sensor2(40, 41);
UltraSonic sensors[] = {sensor1, sensor2};

Driver driver(sensors, 2, Pp, Pi, Pd, Pp_t, Pi_t, Pd_t, limit_correction, limit_correction_turning, circumference, wheel_dist);

Button startButton(52);
Button sideSwitch(48);

LED orangeLED(44);
LED greenLED(45);

bool onOrangeSide = true;

Servo *openingServo;

void setSide() {
  onOrangeSide = sideSwitch.state();

  if (onOrangeSide) {
    orangeLED.on();
    greenLED.off();
  }
  else {
    orangeLED.off();
    greenLED.on();
  }
}

void finaliseSide() {
  if (onOrangeSide) orangeLED.blink(2);
  else greenLED.blink(2);
}

void setupLauncherMotorPins() {
  pinMode(LAUNCHER_SPEED_PIN, OUTPUT);
  pinMode(LAUNCHER_DIR_PIN_1, OUTPUT);
  pinMode(LAUNCHER_DIR_PIN_2, OUTPUT);
  
  digitalWrite(LAUNCHER_DIR_PIN_1, LOW);
  digitalWrite(LAUNCHER_DIR_PIN_2, HIGH);
}

void setupDestickifierMotorPins() {
  pinMode(DESTICKIFIER_SPEED_PIN, OUTPUT);
  pinMode(DESTICKIFIER_DIR_PIN_1, OUTPUT);
  pinMode(DESTICKIFIER_DIR_PIN_2, OUTPUT);

  digitalWrite(DESTICKIFIER_DIR_PIN_1, HIGH);
  digitalWrite(DESTICKIFIER_DIR_PIN_2, LOW);
}

void setDestickifierDirection(bool forward) {
  digitalWrite(DESTICKIFIER_DIR_PIN_1, forward ? LOW : HIGH);
  digitalWrite(DESTICKIFIER_DIR_PIN_2, forward ? HIGH : LOW);
}

void setupStepperMotorPins() {
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);
}

void setupOpeningServo() {
  pinMode(OPENING_SERVO_PIN, OUTPUT);

  openingServo = new Servo();
  openingServo->write(0);
  openingServo->attach(OPENING_SERVO_PIN);
  delay(50);
}

void setLauncherMotorSpin(bool spinning) {
  analogWrite(LAUNCHER_SPEED_PIN, spinning ? 80 : 0);
}

void setDestickifierMotorSpin(bool spinning) {
  analogWrite(DESTICKIFIER_SPEED_PIN, spinning ? 50 : 0);
}

void spinStepperMotor() {
  step_OFF();
  for (int steps = 120; steps > 0; --steps) {
    backward();
  }
}

void openServo() {
  openingServo->write(40);
  delay(1000);
}

void closeServo() {
  openingServo->write(180);
  delay(1000);
}

void jiggle() {
  for (int i = 0; i < 8; ++i) {
    openingServo->write(150);
    delay(300);
    openingServo->write(125);
    delay(300);
  }
}

void beginLaunching() {
  setLauncherMotorSpin(true);
  setDestickifierMotorSpin(true);
  delay(5000);

  for (int i = 0; i < 15; ++i) {
    spinStepperMotor();
    setDestickifierDirection(i % 2 == 0);
  }

  setDestickifierMotorSpin(false);
  setLauncherMotorSpin(false);
}

void forceDriverForward(int distance, bool sense = true) {
  for (; distance > 10; distance -= driver.forward(distance, 5000, sense));
  delay(200);
}

void setup() {
  setupLauncherMotorPins();
  setupDestickifierMotorPins();
  setupStepperMotorPins();
  setupOpeningServo();
  driver.setup();
  
  openServo();

  setSide();
  while (!startButton.state()) {
    setSide();
  }
}

void loop() {
  // move to ball tube position
  forceDriverForward(onOrangeSide ? 385 : 485, true);
  driver.turnAtSpot(onOrangeSide ? 90 : -90);
  driver.forward(-130, 5000, false); // ultrasonic sensing turned off to avoid detecting the side board and stopping
  // get balls into robot
  closeServo();
  jiggle();
  delay(2000);
  // move into launching position
  forceDriverForward(130, true);
  driver.turnAtSpot(onOrangeSide ? 90 : -100);
  // launch balls
  beginLaunching();

  while (true) {}
}

