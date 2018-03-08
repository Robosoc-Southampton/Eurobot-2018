/* This library was created by Daniel Hausner in Nov 2017 for odometry assignment with help of university-provided documents.
 * at University of Southampton. It eases ineraction with MD25 motor driver with EMG30 motors.
 * dh4n16@soton.ac.uk
 */

#ifndef MD25_H
#define MD25_H

#include <Arduino.h>

#include <SoftwareSerial.h>
#include <Wire.h>

class MD25 {
public:
	MD25(int mode, int acceleration=5); // constructor, mode is 0-2 and acceleration 1-10
	void setup(); // setups I2C and mode of MD22
	void forward(int speed, int encoderCount); /* drives both motors forward with set speed until encoders
	do not reach the specified number of pulses (encoderCount) (360 pulese per revolution), then it stops */
	void setSpeed(int speed1, int speed2); /* sends command to drive both motors with the set speeds,
	needs to be enclosed in while loop to drive constantly, otherwise it stops within 2 seconds */
	void stopMotors(); // stops both motors
	void encReset(); // resets both endocedrs
	long encoder1(); // returns value of encoder 1
	long encoder2(); // returns value of encoder 2
	int volts(); // returns battery voltage as a sum of ten readings
	void printEnc(); // reads and prints encoder values
private:
	int _mode;
	int _acceleration;
	int counter;
	int error_sum;
};

#endif
