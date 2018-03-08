/* This library was created for odometry assignment by Shadi Hamou in Nov 2017 with help of online tutorial written by Gamevi Gamevi.
 * At University of Southampton,
 * sh8g16@soton.ac.uk
 */


#include "Song.h" //include the declaration for this class

Song::Song(byte pin){ // constructor
	_pin = pin; // set pin
	pinMode(_pin, OUTPUT); //make _pin an OUTPUT
}

//turn the Song on
void Song::singSongAndBeHappy(){ // plays Mission Impossible song
	tone(_pin,146.83,400);//D
	  delay(400);

	tone(_pin,146.83,400);//D
	   delay(400);

	tone(_pin,146.83,250);//D
	   delay(250);

	tone(_pin,174.61,250);//F
	   delay(250);

	tone(_pin,196.00,350);//G
	   delay(400);

	tone(_pin,146.83,350);//D
	  delay(400);

	tone(_pin,146.83,200);//D
	   delay(250);

	tone(_pin,146.83,200);//D
	   delay(250);

	 tone(_pin,130.81,400);//C
	   delay(450);

	 tone(_pin,130.81,400);//C
	   delay(450);

	tone(_pin,146.83,400);//D
	   delay(450);

	tone(_pin,146.83,400);//D
	   delay(450);

	 tone(_pin,146.83,400);//D
	   delay(450);

	tone(_pin,174.61,400);//F
	   delay(300);

	tone(_pin,196.00,400);//G
	   delay(450);

	 tone(_pin,146.83,400);//D
	   delay(450);

	tone(_pin,146.83,400);//D
	   delay(450);

	 tone(_pin,146.83,400);//D
	   delay(450);

	tone(_pin,130.81,400);//C
	   delay(300);

	 tone(_pin,130.81,400);//C
	   delay(300);

	tone(_pin,130.81,400);//C
	   delay(450);

	tone(_pin,174.61,400);//F
	   delay(450);

	tone(_pin,146.83,400);//D
	   delay(450);

	tone(_pin,220.00,400);//A
	   delay(450);

	tone(_pin,174.61,400);//D
	   delay(450);

	tone(_pin,146.83,400);//D
	   delay(450);

	tone(_pin,207.65,400);//GSHARP
	   delay(450);
	}
