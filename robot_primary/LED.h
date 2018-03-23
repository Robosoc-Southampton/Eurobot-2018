/* This library was created by Daniel Hausner in Nov 2017 for odometry assignment
 * at University of Southampton. It eases interaction with LEDs.
 * dh4n16@soton.ac.uk
 */


#ifndef LED_H
#define LED_H

#include <Arduino.h>

class LED {
public:
	LED(byte pin); // constructor
	void on(); // switch on
	void off(); // switch off
	void blink(int time); // blink with period of time
private:
	byte _pin;
};

#endif
