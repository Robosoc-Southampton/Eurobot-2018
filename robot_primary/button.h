/* This library for Arduino was created by Daniel Hausner with a help of George Hadjigeorgioua and Shadi Hamou in Nov 2017
 * for odometry assignment at University of Southampton. It eases interaction with buttons.
 * dh4n16@soton.ac.uk
 */


#ifndef button_H
#define button_H

#include <Arduino.h>

class Button {
public:
	Button(byte pin); // constructor
	bool state(); // returns state of the button
private:
	byte _pin;
};

#endif
