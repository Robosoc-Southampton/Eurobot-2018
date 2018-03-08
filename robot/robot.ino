/* This is a high level code for the odometry assignment at University of Southamampton, Nov 2017. It was written by Daniel Hausner and debugged with a help of team members,
 * namely Georgios Hadjigeorgiou, Taiwo Tony Khourie, Shadi Hamou and Bethany Harding.
 * dh4n16@soton.ac.uk
 */
#include "LED.h"
#include "MD25.h"
#include "driver.h"
#include "button.h"
#include "myservo.h"
#include "song.h"
#include "PID.h"

#define MD25ADDRESS         0x58                              // Address of the MD25
#define SPEED1              0x00                              // Byte to send speed to both motors for forward and backwards motion if operated in MODE 2 or 3 and Motor 1 Speed if in MODE 0 or 1
#define SPEED2              0x01                              // Byte to send speed for turn speed if operated in MODE 2 or 3 and Motor 2 Speed if in MODE 0 or 1
#define ENCODERONE          0x02                              // Byte to read motor encoder 1
#define ENCODERTWO          0x06                              // Byte to read motor encoder 2
#define ACCELERATION        0xE                               // Byte to define motor acceleration
#define CMD                 0x10                              // Byte to reset encoder values
#define MODE_SELECTOR       0xF                               // Byte to change between control MODES

#define servo_pin 9
#define led_pin 13
#define led_battery_pin 8
#define button_pin 12
#define servo_pin 2

float Pp = 0.5; // 0.5
float Pi = 0;
float Pd = 0;
float Pp_t = 0.6; // 0.6
float Pi_t = 0;
float Pd_t = 0;

int limit_correction = 70; // (min value of 15)
int limit_correction_turning = 90;

int circumference = 321; // [mm]
int wheel_dist = 230; // [mm] initialy 235

// create objects
LED led(led_pin);
LED led_battery(led_battery_pin);
MD25 md(0);
Driver driver(Pp, Pi, Pd, Pp_t, Pi_t, Pd_t, limit_correction, limit_correction_turning, circumference, wheel_dist);
MyServo servo(servo_pin);
Button button(button_pin);
Song song(9);

void setup() {
  Serial.begin(9600); // start serial commuication
  driver.setup(); // start I2C, setup MD25 to mode 0
  servo.attach(); // attach servo
  servo.setPosition(0); // zero postion for servo
  delay(100);
  Serial.println("set up done");
  led.blink(200); // indicate that setup is done
//  if (md.volts() < 120) { // red LED if battery voltage is under 12 V, does not continue execution
//    led_battery.on();
//    while (true);
//  }
//  while(!button.state()); // button for start
  while (true) driver.forward(428);
}

void loop() {
// FINDING PID CONSTANTS
//  driver.turnAtSpot(90);
//  led.blink(500);
//  driver.forward(300);
//  led.blink(500);
//  while(!button.state());

  
  // FINAL CODE - navigation instruction
  int period = 500; // period for blink
  driver.forward(428); // -> 12
  led.blink(period);
  driver.forward(360); // -> 11
  driver.turnAtSpot(142.6);
  led.blink(period);
  driver.turn(212, 288, 'L'); // -> 10      270 degs
  servo.setPosition(1);
  led.blink(period);
  driver.turnAtSpot(-70);
//  while(!button.state());
  driver.forward(180); // -> 9
  led.blink(period);
//  while(!button.state());
  driver.turnAtSpot(140);
  driver.forward(620); // -> 8
  servo.setPosition(2);
  led.blink(period);
  driver.turnAtSpot(40);
  driver.forward(400); // -> 7
  led.blink(period);
  driver.turnAtSpot(90);
  driver.forward(400); // -> 6
  servo.setPosition(3);
  led.blink(period);
  driver.turnAtSpot(90);
  driver.forward(400); // -> 5
  led.blink(period);
  driver.turnAtSpot(90);
  driver.forward(660); // -> 4
  servo.setPosition(4);
  led.blink(period);
  driver.turnAtSpot(-90);
  driver.turn(220, 90, 'L'); // -> 3
  driver.forward(70);
  led.blink(period);
//  while(!button.state());
  driver.turnAtSpot(93);
  driver.forward(500); // -> 2
  servo.setPosition(5);
  led.blink(period);
  driver.turnAtSpot(90);
  driver.forward(260); // -> 1
  led.blink(period);
  driver.turnAtSpot(-90);
  driver.forward(340); // -> finish

  // BE HAPPY
  for (int i = 0; i < 3; i++) {
    led.blink(200);
    led_battery.blink(200);
    delay(200);
  }
  driver.turnAtSpot(360);
  song.singSongAndBeHappy();
  while(true);





  
//  // DEBUGGING PID
//  int target_value = driver.getEncVal(500);
//  int enc_cur = md.encoder1();
//  Serial.print("target_value: ");
//  Serial.println(target_value);
//  Serial.print("enc_cur: ");
//  Serial.println(enc_cur);
//  driver.calculatePid(enc_cur, target_value);
//  driver.printPid();
//  int enc = md.encoder1();
//  Serial.println("\n");
//  delay(500);


// // PID PRINTING
//  driver.calculatePid(10, 1000);
//  driver.printPid();
//  driver.calculatePid(100, 1000);
//  driver.printPid();
//  driver.calculatePid(500, 1000);
//  driver.printPid();
//  driver.calculatePid(900, 1000);
//  driver.printPid();
//  driver.calculatePid(1100, 1000);
//  driver.printPid();
//  while(true);





//// FIRST TEST
//  int t = 0;
//  led.blink(400);
//  do{
//    md.setSpeed(0+cor, 0+cor);
//    delay(500);
//    t += 500;
//  } while (t <= 5000);
//  md.setSpeed(128, 128);
//  delay(500);
//  t = 0;
//  do{
//    md.setSpeed(255-cor, 255-cor);
//    delay(500);
//    t += 500;
//  } while (t <= 5000);
//  md.setSpeed(128, 128);
//  delay(500);
//
//  int encodeVal1 = md.encoder1();
//  Serial.print("encoder1: ");
//  Serial.print(encodeVal1, DEC);
//  Serial.print("\t");
//  int encodeVal2 = md.encoder2();
//  Serial.print("encoder2: ");
//  Serial.println(encodeVal2, DEC);
//  delay(3000);
//  md.encReset();


}
