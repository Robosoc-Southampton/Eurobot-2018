/* This is a high level code for the odometry assignment at University of Southamampton, Nov 2017. It was written by Daniel Hausner and debugged with a help of team members,
   namely Georgios Hadjigeorgiou, Taiwo Tony Khourie, Shadi Hamou and Bethany Harding.
   dh4n16@soton.ac.uk
*/

// Note, the robot drives backwards by default so all distances must be reversed.

#include "LED.h"
#include "MD25.h"
#include "driver.h"
#include "button.h"
#include "myservo.h"
#include "PID.h"
#include "UltraSonic.h"

#define MD25ADDRESS         0x58                              // Address of the MD25
#define SPEED1              0x00                              // Byte to send speed to both motors for forward and backwards motion if operated in MODE 2 or 3 and Motor 1 Speed if in MODE 0 or 1
#define SPEED2              0x01                              // Byte to send speed for turn speed if operated in MODE 2 or 3 and Motor 2 Speed if in MODE 0 or 1
#define ENCODERONE          0x02                              // Byte to read motor encoder 1
#define ENCODERTWO          0x06                              // Byte to read motor encoder 2
#define ACCELERATION        0xE                               // Byte to define motor acceleration
#define CMD                 0x10                              // Byte to reset encoder values
#define MODE_SELECTOR       0xF                               // Byte to change between control MODES

const int ALLOWED_ERROR = 5;

const float Pp = 0.5; // 0.5
const float Pi = 0.0;
const float Pd = 0.3;
const float Pp_t = 0.5; // 0.6
const float Pi_t = 0;
const float Pd_t = 0.3;

int limit_correction = 75; // (min value of 15)
int limit_correction_turning = 90;

int circumference = 314; // [mm]
int wheel_dist = 215; // [mm] initialy 235

bool onOrangeSide = true;

// create objects
const unsigned int SENSOR_COUNT = 2;

Button startButton(6);
Button sideSwitch(7);

LED orangeSideLED(8);
LED greenSideLED(9);

Servo *beePushingArmServo;

UltraSonic distanceSensor(2, 3);
UltraSonic rearDistanceSensor(5, 4);
UltraSonic distanceSensors[SENSOR_COUNT] = {rearDistanceSensor, distanceSensor};

Driver driver(distanceSensors, SENSOR_COUNT, Pp, Pi, Pd, Pp_t, Pi_t, Pd_t, limit_correction, limit_correction_turning, circumference, wheel_dist);

void setSide() {
  onOrangeSide = sideSwitch.state();
  
  if (onOrangeSide) orangeSideLED.on(); else orangeSideLED.off();
  if (onOrangeSide) greenSideLED.off(); else greenSideLED.on();
}

void raiseBeePushingArm() {
  beePushingArmServo->write(80);
  delay(1000);
}

void lowerBeePushingArm() {
  beePushingArmServo->write(0);
  delay(1000);
}

void setup() {
  // Serial.begin(9600); // start serial commuication
  // Serial.println("starting set up");

  Serial.begin(9600);

  while (false) {
    Serial.print(distanceSensor.allBelowThreshold(10.0));
    Serial.print(" ");
    Serial.println(distanceSensor.getValue());
  }

  driver.setup(); // start I2C, setup MD25 to mode 0
  pinMode(10, OUTPUT);
  beePushingArmServo = new Servo();
  beePushingArmServo->write(0);
  beePushingArmServo->attach(10);
  delay(50);
  
  raiseBeePushingArm();

  pinMode(LED_BUILTIN, OUTPUT);

  setSide();

  while (!startButton.state()) {
    setSide();
  }
}

void forceForward(int distance, bool sense = true) {
  distance = -distance;
  int timeout = 5000;
  int dt = 0;
  int start = millis();
  for (; distance < -10; distance -= driver.forward(distance, timeout, sense)) {
    dt = millis() - start;
    start = millis();
    timeout -= dt;
  }
  delay(300);
}

void moveForward(int distance, bool sense = true) {
  int timeout = 5000;
  int start_time = millis();

  distance = -distance;
  
  while (true) {
    int distanceMoved = driver.forward(distance, timeout, sense);
    int dt = millis() - start_time;

    Serial.print("Moved ");
    Serial.println(distanceMoved);
    
    start_time = millis();
    timeout -= dt;
    distance -= distanceMoved;
    
    if (distance > -ALLOWED_ERROR && distance < ALLOWED_ERROR) break;
    if (timeout < 0) break;
  }

  Serial.print("Error ");
  Serial.println(distance);

  delay(500);
}

void loop() {
  if (onOrangeSide) {
    moveForward(900);
    driver.turnAtSpot(-90);
    driver.forward(200, 1000, false);
    moveForward(800);
    driver.turnAtSpot(-45);
    moveForward(1050);
    driver.turnAtSpot(-135);
    moveForward(-180);
    driver.turnAtSpot(-93);
    driver.forward(500); // should stop when it detects a thing
    lowerBeePushingArm();
    moveForward(320);
    raiseBeePushingArm();
  }
  else {
    moveForward(800);
    driver.turnAtSpot(90);
    driver.forward(200, 1000, false);
    moveForward(790);
    driver.turnAtSpot(45);
    moveForward(900);
    driver.turnAtSpot(135);
    moveForward(-310, false);
    driver.turnAtSpot(-90);
    moveForward(300, false);
    lowerBeePushingArm();
    moveForward(-300);
    raiseBeePushingArm();
  }

  while (true) {}
}

