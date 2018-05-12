
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
#define delaytime 4

#define LAUNCHER_SPEED_PIN 8 // ENA - PWM
#define LAUNCHER_DIR_PIN_1 25 // IN1
#define LAUNCHER_DIR_PIN_2 24 // IN2

#define LAUNCHER_SPEED_ORANGE 83 // 84
#define LAUNCHER_SPEED_GREEN 83 // 84

#define DESTICKIFIER_SPEED_PIN 9 // ENB - PWM
#define DESTICKIFIER_DIR_PIN_1 23 // IN3
#define DESTICKIFIER_DIR_PIN_2 22 // IN4

#define OPENING_SERVO_PIN 10

#define TIGHTENING_SERVO_PIN 13
#define TIGHTENING_SERVO_LOOSE 48
#define TIGHTENING_SERVO_TAUGHT 12

unsigned long START_TIME = 0;

const int maxServoPosition = 15;
int currentServoPosition = 0;
const int d = 00; // delay between driving actions

const float Pp = 0.13; // 0.5
const float Pi = 0;
const float Pd = 0;
const float Pp_t = 0.13; // 0.6
const float Pi_t = 0;
const float Pd_t = 0;

const int limit_correction = 15; // (min value of 15)
const int limit_correction_turning = 90;

const int circumference = 314; // [mm]
const int wheel_dist = 329; // [mm] initialy 235

UltraSonic sensor1(12, 11); // front
UltraSonic sensor2(40, 41);
UltraSonic sensor3(3, 4); // left
UltraSonic sensor4(38, 39); // right
UltraSonic sensors[] = {sensor1, sensor2, sensor3, sensor4};

Driver driver(sensors, 4, Pp, Pi, Pd, Pp_t, Pi_t, Pd_t, limit_correction, limit_correction_turning, circumference, wheel_dist);

Button startButton(52);
Button sideSwitch(48);

LED orangeLED(44);
LED greenLED(45);

bool onOrangeSide = true;

Servo *openingServo;
Servo *stringServo;

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

void setupStringServo() {
  pinMode(TIGHTENING_SERVO_PIN, OUTPUT);

  stringServo = new Servo();
  stringServo->write(TIGHTENING_SERVO_LOOSE);
  stringServo->attach(TIGHTENING_SERVO_PIN);
  delay(50);
}

// set the spinning direction of the cable tie motor (note a value of true for `forward` doesn't necessarily push the balls forward)
void setDestickifierDirection(bool forward) {
  digitalWrite(DESTICKIFIER_DIR_PIN_1, forward ? LOW : HIGH);
  digitalWrite(DESTICKIFIER_DIR_PIN_2, forward ? HIGH : LOW);
}

// set the state of the ball launcher motor (spinning = true -> will spin motor and launch balls)
void setLauncherMotorSpin(bool spinning) {
  if (spinning) {
    analogWrite(LAUNCHER_SPEED_PIN, 150);
    delay (150);
  }
  analogWrite(LAUNCHER_SPEED_PIN, spinning ? (onOrangeSide ? LAUNCHER_SPEED_ORANGE : LAUNCHER_SPEED_GREEN) : 0);
  delay(200);
}

// set the state of the cable tie motor (spinning = true -> will spin motor)
void setDestickifierMotorSpin(bool spinning) {
  analogWrite(DESTICKIFIER_SPEED_PIN, spinning ? 50 : 0);
}

// turns the stepper motor by one ball cycle
void spinStepperMotor() {
  step_OFF();
  for (int steps = 120; steps > 0; --steps) {
    forwardFast();
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

void loosenString() {
  stringServo->write(TIGHTENING_SERVO_LOOSE);
  delay(1000);
}

void tightenString() {
  stringServo->write(TIGHTENING_SERVO_TAUGHT);
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

void setupLaunching() {
  setLauncherMotorSpin(true);
  setDestickifierMotorSpin(true);
  setDestickifierDirection(true);
  delay(1000);
  setDestickifierMotorSpin(false);
  delay(1000);
}

// begin the whole launching cycle, should be called once
void beginLaunching() {
  for (int i = 0; true; ++i) {
    if (millis() - START_TIME >= 84000) break;
    setDestickifierMotorSpin(i % 2 == 0);
    setDestickifierDirection(i % 4 == 0);
    spinStepperMotor();
    delay(100);
  }

  //setDestickifierMotorSpin(false);
  setLauncherMotorSpin(false);
  setDestickifierMotorSpin(false);
}

// move forward by a distance in millimeters, `sense` will enable the ultrasonic sensors
void moveForward(int distance, bool sense = true) {
  unsigned long timeout = 90000;
  unsigned long start_time = millis();
  
  while (true) {
    int distanceMoved = driver.forward(distance, timeout < 5000 ? timeout : 5000, sense);
    unsigned long dt = millis() - start_time;
 
    start_time = millis();
    timeout -= dt;
    distance -= distanceMoved;
    
    if (distance > -50 && distance < 50) break;
    if (timeout < 0) break;
  }

  delay(250);
}

void setup() {
  Serial.begin(9600);
  
  orangeLED.on();
  greenLED.on();

  setupLauncherMotorPins();
  setupDestickifierMotorPins();
  setupStepperMotorPins();
  setupOpeningServo();
  setupStringServo();
  
  driver.setup();

  openServo();

  /*while (true) {
    Serial.print(sensor3.getValue());
    Serial.print(", ");
    Serial.print(sensor1.getValue());
    Serial.print(", ");
    Serial.println(sensor4.getValue());
  }*/

  setSide();
  while (!startButton.state()) {
    setSide();
  }

  setupLaunching();

  while (true) {
    beginLaunching();
    START_TIME = millis();
  }

  if (START_TIME == 0)
    START_TIME = millis();
}

void loop() {
  // move to ball tube position

  moveForward(onOrangeSide ? 395 : 445);
  driver.turnAtSpot(onOrangeSide ? 90 : -90);
  moveForward(-125, false); // ultrasonic sensing turned off to avoid detecting the side board and stopping
  //
  // get balls into robot
  
  closeServo();
  delay(1000);
  jiggle();
  delay(500);
  //
  // move into launching position
  
  moveForward(onOrangeSide ? 130 : 150);
  openServo();
  setupLaunching();
  
  driver.turnAtSpot(onOrangeSide ? 90 : -97);
  //moveForward(50);
  //moveForward(200);
  //
  // launch balls
  beginLaunching();

  // wait infinitely
  while (true) {}
}

