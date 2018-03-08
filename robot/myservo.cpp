/* This library was created by Taiwo Tony Khourie and debugged with help of Georgios Hadjigeorgiou in Nov 2017 for odometry assignment
 * at University of Southampton
 * ttk1g16@soton.ac.uk
 */


#include "myservo.h" //include the declaration for this class
#include <Servo.h>


MyServo::MyServo(byte pin){ // constructor
	_pin = pin; // set pin
	pinMode(_pin, OUTPUT); //make _pin an OUTPUT
	servo = new Servo;
	cur_angle = 0;
	servo->write(0);
}

void MyServo::attach() {
	servo->attach(_pin);
  delay(50);
}

void MyServo::setPosition (int pos, int del) {
	int target_angle;
	switch (pos) {
    case 0: target_angle = 5;
       break;
		case 1: target_angle = 31;
       break;
		case 2: target_angle = 62;
       break;
		case 3: target_angle = 96;
       break;
		case 4: target_angle = 128;
       break;
		case 5: target_angle = 160;
       break;
	}
	for (temp_pos = cur_angle; temp_pos <= target_angle; temp_pos += 1) {
		servo->write(temp_pos);
		delay(del);
//    Serial.print("pos: ");
//    Serial.println(pos);
//    Serial.print("cur_pos: ");
//    Serial.println(pos);
	}
  cur_angle = temp_pos;
}


