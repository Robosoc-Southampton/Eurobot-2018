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
#include "UltraSonic.h"

#define MD25ADDRESS         0x58                              // Address of the MD25
#define SPEED1              0x00                              // Byte to send speed to both motors for forward and backwards motion if operated in MODE 2 or 3 and Motor 1 Speed if in MODE 0 or 1
#define SPEED2              0x01                              // Byte to send speed for turn speed if operated in MODE 2 or 3 and Motor 2 Speed if in MODE 0 or 1
#define ENCODERONE          0x02                              // Byte to read motor encoder 1
#define ENCODERTWO          0x06                              // Byte to read motor encoder 2
#define ACCELERATION        0xE                               // Byte to define motor acceleration
#define CMD                 0x10                              // Byte to reset encoder values
#define MODE_SELECTOR       0xF                               // Byte to change between control MODES

const int ALLOWED_ERROR = 20;

const float Pp = 0.3; // 0.5
const float Pi = 0.0;
const float Pd = 0.2;
const float Pp_t = 0.5; // 0.6
const float Pi_t = 0;
const float Pd_t = 0.3;

int limit_correction = 90; // (min value of 15)
int limit_correction_turning = 90;

int circumference = 314; // [mm]
int wheel_dist = 215; // [mm] initialy 235

bool onOrangeSide = true;

// create objects
const unsigned int SENSOR_COUNT = 3;

Button startButton(6);
Button sideSwitch(7);

LED orangeSideLED(8);
LED greenSideLED(9);

Servo *beePushingArmServo;

UltraSonic distanceSensor(2, 3);
UltraSonic rearDistanceSensor(5, 4);
UltraSonic otherDistanceSensor(13, 12);
UltraSonic distanceSensors[SENSOR_COUNT] = {rearDistanceSensor, distanceSensor, otherDistanceSensor};

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
  beePushingArmServo->write(10);
  delay(1000);
}

void moveForward(int distance, bool sense = true) {
  int timeout = 90000;
  int start_time = millis();

  distance = -distance;
  
  while (true) {
    int distanceMoved = driver.forward(distance, timeout, sense);
    int dt = millis() - start_time;

    start_time = millis();
    timeout -= dt;
    distance -= distanceMoved;
    
    if (distance > -ALLOWED_ERROR && distance < ALLOWED_ERROR) break;
    if (timeout < 0) break;
  }

  delay(500);
}

void alignToWall(int distance) {
  driver.forward(-distance, 3000, false);
  delay(500);
}

void setup() {
  Serial.begin(9600);

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

void loop() {
  if (onOrangeSide) {
    moveForward(900);
    driver.turnAtSpot(-90);
    alignToWall(-200);
    moveForward(800);
    driver.turnAtSpot(-45);
    moveForward(1050);
    driver.turnAtSpot(-135);
    alignToWall(-500);
    moveForward(85);
    driver.turnAtSpot(-90);
    moveForward(-300);
    alignToWall(-200);
    lowerBeePushingArm();
    moveForward(320);
    raiseBeePushingArm();
    moveForward(-150);
    /*
    moveForward(-100);
    driver.turnAtSpot(90);
    moveForward(-100, false);
    moveForward(300);
    driver.turnAtSpot(-45);
    moveForward(800);
    driver.turnAtSpot(45);
    moveForward(1000, false);
    */
  }
  else {
    moveForward(800);
    driver.turnAtSpot(90);
    alignToWall(-200);
    moveForward(790);
    driver.turnAtSpot(45);
    moveForward(900);
    driver.turnAtSpot(135);
    moveForward(-150);
    alignToWall(-500);
    moveForward(85);
    driver.turnAtSpot(-90);
    moveForward(150);
    alignToWall(300);
    lowerBeePushingArm();
    moveForward(-300);
    raiseBeePushingArm();
    moveForward(150);
    /*
    moveForward(100);
    driver.turnAtSpot(90);
    moveForward(-150, false);
    moveForward(300);
    driver.turnAtSpot(45);
    moveForward(720);
    driver.turnAtSpot(-45);
    moveForward(900, false);
    */
  }

  while (true) {}
}

