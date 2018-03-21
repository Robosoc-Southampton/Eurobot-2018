/* This is a high level code for the odometry assignment at University of Southamampton, Nov 2017. It was written by Daniel Hausner and debugged with a help of team members,
   namely Georgios Hadjigeorgiou, Taiwo Tony Khourie, Shadi Hamou and Bethany Harding.
   dh4n16@soton.ac.uk
*/
#include "LED.h"
#include "MD25.h"
#include "driver.h"
#include "button.h"
#include "myservo.h"
#include "song.h"
#include "PID.h"
#include "QTRSensors.h"
#include "UltraSonic.h"

#define MD25ADDRESS         0x58                              // Address of the MD25
#define SPEED1              0x00                              // Byte to send speed to both motors for forward and backwards motion if operated in MODE 2 or 3 and Motor 1 Speed if in MODE 0 or 1
#define SPEED2              0x01                              // Byte to send speed for turn speed if operated in MODE 2 or 3 and Motor 2 Speed if in MODE 0 or 1
#define ENCODERONE          0x02                              // Byte to read motor encoder 1
#define ENCODERTWO          0x06                              // Byte to read motor encoder 2
#define ACCELERATION        0xE                               // Byte to define motor acceleration
#define CMD                 0x10                              // Byte to reset encoder values
#define MODE_SELECTOR       0xF                               // Byte to change between control MODES

const float Pp = 0.5; // 0.5
const float Pi = 0;
const float Pd = 0;
const float Pp_t = 0.6; // 0.6
const float Pi_t = 0;
const float Pd_t = 0;

int limit_correction = 15; // (min value of 15)
int limit_correction_turning = 90;

int circumference = 314; // [mm]
int wheel_dist = 225; // [mm] initialy 235

// create objects
const unsigned int SENSOR_COUNT = 1;

Button startButton(4);

UltraSonic distanceSensor(2, 3);
UltraSonic distanceSensors[SENSOR_COUNT] = {distanceSensor};

Driver driver(distanceSensors, SENSOR_COUNT, Pp, Pi, Pd, Pp_t, Pi_t, Pd_t, limit_correction, limit_correction_turning, circumference, wheel_dist);

// PololuQTRSensorsRC qtr((char[]) {14, 15, 16}, 3);

void setup() {
  Serial.begin(9600); // start serial commuication
  Serial.println("starting set up");

  driver.setup(); // start I2C, setup MD25 to mode 0

  Serial.println("calibrating qtr");

  for (int i = 0; i < 250; ++i) {
    // qtr.calibrate();
    // delay(20);
  }

  Serial.println("set up done");
  
  pinMode(LED_BUILTIN, OUTPUT);

  while (!startButton.state()) {
    Serial.println("Waiting for button");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(300);
    digitalWrite(LED_BUILTIN, LOW);
    delay(300);
  }
}

void forceForward(int distance, bool sense = true) {
  for (; distance > 10; distance -= driver.forward(distance, 5000, sense));
  delay(300);
}

void loop() {
  // int dist = driver.forward(300);

  while (false) {
    driver.turnAtSpot((float) random(30, 150) * (random(0, 1) * 2 - 1));
    for (int dist = random(500, 1500); dist > 10; dist -= driver.forward(dist));
    delay(1000);
  }

  //*
  forceForward(350);
  driver.turnAtSpot(45.0);
  forceForward(700);
  driver.turnAtSpot(90.0);
  forceForward(930);
  driver.turnAtSpot(-45.0);
  forceForward(150, false);
  driver.turnAtSpot(-90.0);
  forceForward(400);
  delay(300);
  driver.forward(-400);
  delay(300);
  driver.turnAtSpot(-45.0);
  forceForward(950);
  driver.turnAtSpot(-45.0);
  forceForward(1000, false);
  // */

  // driver.forward(1000, 5000, false);

  while (!startButton.state()) {
    Serial.println("Waiting for button");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(300);
    digitalWrite(LED_BUILTIN, LOW);
    delay(300);
  }

  while (false) {
    // unsigned int sensors[3];
    // int position = qtr.readLine(sensors);

    for (int i = 0; i < 3; ++i) {
      // Serial.print(sensors[i]);
      // Serial.print(" ");
    }

    Serial.println();
  }
}
