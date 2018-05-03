
/* Arduino pins:
 *  
 *  52 - grey wire - breadboard/start switch
 *  48 - yellow wire - breadboard/side switch
 *  44 - orange wire - breadboard/orange LED
 *  45 - green wire - breadboard/green LED
 *  40 - grey wire - to back of robot
 *  41 - white wire - to back of robot
 *  30-33 - green, blue, purple, grey wires - to back of robot
 *  22-25 - yellow, green, blue, purple wires - to motor controller breadboard
 *  20 - grey wire - to back of robot
 *  21 - white wire - to back of robot
 *  8 - grey wire - to motor controller breadboard
 *  9 - orange wire - to motor controller breadboard
 *  10 - orange wire - to back of robot
 *  11 - brown wire - to ultrasonic sensor
 *  12 - green wire - to ultrasonic sensor
 */

#include <Servo.h>
#include "driver.h"
#include "UltraSonic.h"
#include "button.h"
#include "LED.h"

#define pin1 30
#define pin2 31
#define pin3 32
#define pin4 33
#define delaytime 3

#define LAUNCHER_SPEED_PIN 8 // ENA - PWM
#define LAUNCHER_DIR_PIN_1 25 // IN1
#define LAUNCHER_DIR_PIN_2 24 // IN2

#define LAUNCHER_SPEED 255

#define DESTICKIFIER_SPEED_PIN 9 // ENB - PWM
#define DESTICKIFIER_DIR_PIN_1 23 // IN3
#define DESTICKIFIER_DIR_PIN_2 22 // IN4

#define OPENING_SERVO_PIN 10

const int maxServoPosition = 15;
int currentServoPosition = 0;

const float Pp = 0.5; // 0.5
const float Pi = 0;
const float Pd = 0;
const float Pp_t = 0.6; // 0.6
const float Pi_t = 0;
const float Pd_t = 0;

const int limit_correction = 30; // (min value of 15)
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

// updates the onOrangeSide variable based on the state of the side switch
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

// various setup functions:

void setupLauncherMotorPins() {
  pinMode(LAUNCHER_SPEED_PIN, OUTPUT);
  pinMode(LAUNCHER_DIR_PIN_1, OUTPUT);
  pinMode(LAUNCHER_DIR_PIN_2, OUTPUT);
  
  digitalWrite(LAUNCHER_DIR_PIN_1, HIGH);
  digitalWrite(LAUNCHER_DIR_PIN_2, LOW);
}

void setupDestickifierMotorPins() {
  pinMode(DESTICKIFIER_SPEED_PIN, OUTPUT);
  pinMode(DESTICKIFIER_DIR_PIN_1, OUTPUT);
  pinMode(DESTICKIFIER_DIR_PIN_2, OUTPUT);

  digitalWrite(DESTICKIFIER_DIR_PIN_1, HIGH);
  digitalWrite(DESTICKIFIER_DIR_PIN_2, LOW);
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

// set the spinning direction of the cable tie motor (note a value of true for `forward` doesn't necessarily push the balls forward)
void setDestickifierDirection(bool forward) {
  digitalWrite(DESTICKIFIER_DIR_PIN_1, forward ? LOW : HIGH);
  digitalWrite(DESTICKIFIER_DIR_PIN_2, forward ? HIGH : LOW);
}

// set the state of the ball launcher motor (spinning = true -> will spin motor and launch balls)
void setLauncherMotorSpin(bool spinning) {
  analogWrite(LAUNCHER_SPEED_PIN, spinning ? LAUNCHER_SPEED : 0);
}

// set the state of the cable tie motor (spinning = true -> will spin motor)
void setDestickifierMotorSpin(bool spinning) {
  analogWrite(DESTICKIFIER_SPEED_PIN, spinning ? 50 : 0);
}

// turns the stepper motor by one ball cycle
void spinStepperMotor() {
  step_OFF();
  for (int steps = 120; steps > 0; --steps) {
    forward();
  }
}

// move the top servo to the open position (curved part over robot)
void openServo() {
  openingServo->write(40);
  delay(1000);
}

// move the top servo to the closed position (curved part over the ball entrance) (use let balls out of tube)
void closeServo() {
  openingServo->write(180);
  delay(1000);
}

// move top servo back and forth around the area at which it opens the ball tube
void jiggle() {
  for (int i = 0; i < 2; ++i) {
    openingServo->write(150);
    delay(300);
    openingServo->write(125);
    delay(300);
  }
  openingServo->write(150);
}

// begin the whole launching cycle, should be called once
void beginLaunching() {
  setLauncherMotorSpin(true);
  // setDestickifierMotorSpin(true);
  setDestickifierDirection(true);
  delay(2000);
  setDestickifierMotorSpin(false);
  delay(2000);

  for (int i = 0; i < 15; ++i) {
    // setDestickifierMotorSpin(i % 2 == 0);
    setDestickifierDirection(i % 4 == 0);
    spinStepperMotor();

    /*if (i == 6) {
      setDestickifierMotorSpin(true);
      delay(2000);
      setDestickifierMotorSpin(false);
    }*/
  }

  setDestickifierMotorSpin(false);
  setLauncherMotorSpin(false);
}

// move forward by a distance in millimeters, `sense` will enable the ultrasonic sensors
void moveForward(int distance, bool sense = true) {
  int timeout = 5000;
  int start_time = millis();
  
  while (true) {
    int distanceMoved = driver.forward(distance, timeout, sense);
    int dt = millis() - start_time;
 
    start_time = millis();
    timeout -= dt;
    distance -= distanceMoved;
    
    if (distance > -50 && distance < 50) break;
    if (timeout < 0) break;
  }

  delay(500);
}

void setup() {
  Serial.begin(9600);

  orangeLED.on();
  greenLED.on();
  
  setupLauncherMotorPins();
  setupDestickifierMotorPins();
  setupStepperMotorPins();
  setupOpeningServo();
  driver.setup();

  openServo();

  setLauncherMotorSpin(true);

  while (true) {}

  setSide();
  while (!startButton.state()) {
    setSide();
  }
}

void loop() {
  // move to ball tube position
  /*
  moveForward(onOrangeSide ? 400 : 440, true);
  driver.turnAtSpot(onOrangeSide ? 90 : -90);
  driver.forward(onOrangeSide ? -125 : -130, 5000, false); // ultrasonic sensing turned off to avoid detecting the side board and stopping
  //*/
  // get balls into robot
  /*
  closeServo();
  delay(1000);
  jiggle();
  delay(1000);
  //*/
  // move into launching position
  /*
  moveForward(onOrangeSide ? 130 : 150, true);
  openServo();
  driver.turnAtSpot(onOrangeSide ? 85 : -95);
  moveForward(200);
  //*/
  // launch balls
  beginLaunching();

  // wait infinitely
  while (true) {}
}

