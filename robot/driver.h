/* This library for Arduino was created by Daniel Hausner and debugged with a help of Taiwo Khourie, George Hadjigeorgioua
 * Shadi Hamou and Bethany Harding in Nov 2017 for odometry assignment at University of Southampton.
 * It eases interaction with MD25 driver.
 * dh4n16@soton.ac.uk
 */

/*	10*circumference = 3213 mm
	distance between wheels = 235 mm
	for our robot
*/

#ifndef Driver_H
#define Driver_H

#include <Arduino.h>
#include "MD25.h"
#include "QTRSensors.h"
#include "UltraSonic.h"
#include "LED.h"
#include "time.h"

const float PROXIMITY_THRESHOLD = 15.0;

class Driver {
public:
	// sensorCount should be either 1 or 2
	// if sensorCount is 1, sensors[1] should be a rear sensor
	// sensors[0] should always be a front sensor
	Driver(UltraSonic *sensors, unsigned int sensorCount, float Pp, float Pi, float Pd, float Pp_t, float Pi_t, float Pd_t, int limit_correction, int limit_correction_turning, int circumference, float wheel_dist); // constructor
	/* Pp, Pi, Pd, Pp_t, Pi_t, Pd_t: PID constants, limit_correction and limit_correction_turing: limits max speed of the motors for going straight and turning at spot respectively,
	 * circumference: circumference of the wheel, wheel_dist: distance between the wheels */
	void setup(); // calls setup from md25.h to start the serial communication, set acceleration and reset encoders
	int forward(int dist, long timeout=5000, bool sense = true); // drives forward using PID and help functions below
	// unsafe (won't stop using ultrasonic sensors):
	void turnAtSpot(float angle, long timeout=5000); // turns at spot until it reaches required angle (positive clockwise) using PID and help functions
	void turn(int rad, int angle, char side, long timeout=5000); // drives on arc with specified parametres

	bool senseLine(PololuQTRSensors sensor); // returns true if the line sensor detects a line
	int getDistance(int enc_val); // return the distance travelled using the encoder value

	// DEBUG FUCNTIONS
	void printPid(); // prints current PID values and error
	void printEnc(); // prints current ecoder values

	// HELP FUNCTIONS
	int getEncVal(int dist); // returns encoder count for the required distance
	int getSpeed(int speed); /* inputs speed from PID; if it is outside the limit
	[0+limit_correction, 255-limit_correction], returns the limit value, otherwise it returns the speed */
	int getSpeedTurn(int speed); // the same as above, but with variables for turning at spot
	void calculatePid(int enc_val_cur, int target_val); // calculates PID values
	void calculatePidTurn(int enc_val_cur, int target_val); // calculates PID values for turning at spot
	bool readingPeriod(); // returns true if time elapsed between current time and previous time is larger than period
	bool terminatePid(); // help function for terminating pid
	float pi;

private:
	UltraSonic *sensors;
	unsigned int sensorCount;
	float Kp; // constants for PID
	float Ki;
	float Kd;
	float Kp_t;
	float Ki_t;
	float Kd_t;
	int limit_cor;
  	int limit_cor_turn;
	int error; // error for PID (encoder count)
	int previous_error;
	int P; // proportional error for PID
	int I; // integral of error for PID
	int D; // differential of error for PID
	float PID_val; // value for PID (it is devided by 100 so that it does not require type float)
	int PID_speed_theor; // PID_speed = 128 + PID_val (automatically asigned in calculatePID())
	int PID_speed_limited; // limited PID speed so that the motor does not exceeds min and max speeds, performed by getSpeed(..)
	int cir; // circumference of the wheel [mm]
	float w_dist; // distance between wheels [mm]
	public: MD25* md; private:
	unsigned long cur_time;
	unsigned long prev_time;
	unsigned int period;
	int pid_prec; // if sum of 10 encoder reading is smaller than pid_prec, forward(..) terminates
	int counter;
	long error_sum;
};

#endif
