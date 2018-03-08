/* This library was created for odometry assignment by Shadi Hamou in Nov 2017 with help of online tutorial written by Gamevi Gamevi.
 * At University of Southampton,
 * sh8g16@soton.ac.uk
 */

#ifndef Song_H
#define Song_H

#include <Arduino.h>

class Song {
public:
	Song(byte pin); // constructor
	void singSongAndBeHappy(); // plays Mission Impossible song
private:
	byte _pin;
};

#endif
