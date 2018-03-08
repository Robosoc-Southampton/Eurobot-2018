/* This library was created by Taiwo Tony Khourie and debugged with help of Georgios Hadjigeorgiou in Nov 2017 for odometry assignment
 * at University of Southampton
 * ttk1g16@soton.ac.uk
 */

#ifndef MyServo_H
#define MyServo_H

#include <Arduino.h>
#include <Servo.h>

class MyServo {
public:
	MyServo(byte pin); // constructor
  void attach();
	void setPosition(int pos, int del=50); // sets position with set delay which is applied between every angle
private:
	byte _pin;
	Servo* servo;
	int cur_angle, temp_pos;
};

#endif
