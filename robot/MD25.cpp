/* This library was created by Daniel Hausner in Nov 2017 for odometry assignment with help of university-provided documents.
 * at University of Southampton. It eases ineraction with MD25 motor driver with EMG30 motors.
 * dh4n16@soton.ac.uk
 */


#include "MD25.h" //include the declaration for this class

#include <SoftwareSerial.h>
#include <Wire.h>

#define CMD 0x10
 // This is a but with arduino 1
#define MD25ADDRESS 0x58 // Address of the MD25
#define SOFTWAREREG 0x0D /* Byte to read the software version, Values of 0 eing sent using write
have to be cast as a byte to stop them being misinterperted as NULL */
#define SPEED1 (byte)0x00 // Byte to send speed to first motor
#define SPEED2 0x01 // Byte to send speed to second motor
#define ENCODERONE 0x02 // Byte to read motor encoder 1
#define ENCODERTWO 0x06 // Byte to read motor encoder 2
#define VOLTREAD 0x0A // Byte to read battery volts
#define RESETENCODERS 0x20
#define MODE_SELECTOR 0xF // Byte to change between control MODES
#define ACCELERATION 0xE // Byte to define motor acceleration

MD25::MD25(int mode, int acceleration) { // consturctor, mode of MD25 operation
	_mode = mode;
  _acceleration = acceleration;
}

void MD25::setup() {
	Wire.begin(); // Begin I2C bus
	// Serial.begin(9600); // Begin serial
	delay(100); // Wait for everything to power up

	Wire.beginTransmission(MD25ADDRESS); // Set MD25 operation MODE
	Wire.write(MODE_SELECTOR);
	Wire.write(_mode);
	Wire.endTransmission();
  	delay(100);

	Wire.beginTransmission(MD25ADDRESS); // Set MD25 operation MODE
	Wire.write(ACCELERATION);
	Wire.write(_acceleration);                                           
	Wire.endTransmission();
  	delay(100);

	encReset(); // Cals a function that resets the encoder values to 0 
}

void MD25::forward(int speed, int encoderCount=360) { // 360 is one revolution
	do { // Start loop to drive motors forward
		Wire.beginTransmission(MD25ADDRESS); // Drive motor 2 at speed value stored in speed
		Wire.write(SPEED2);
		Wire.write(speed);
		Wire.endTransmission();

		Wire.beginTransmission(MD25ADDRESS); // Drive motor 1 at speed value stored in speed
		Wire.write(SPEED1);
		Wire.write(speed);
		Wire.endTransmission();
	} while(abs(encoder1()) < abs(encoderCount));
	stopMotors();
}

void MD25::setSpeed(int speed1, int speed2) {
	Wire.beginTransmission(MD25ADDRESS); // Drive motor 1 at speed value stored in speed1
	Wire.write(SPEED1);
	Wire.write(speed1);
	Wire.endTransmission();

	Wire.beginTransmission(MD25ADDRESS); // Drive motor 2 at speed value stored in speed2
	Wire.write(SPEED2);
	Wire.write(speed2);
	Wire.endTransmission();
}

void MD25::stopMotors() { // Function to stop motors
	Wire.beginTransmission(MD25ADDRESS);
	Wire.write(SPEED2);
	Wire.write(128); // Sends a value of 128 to motor 2 this value stops the motor
	Wire.endTransmission();

	Wire.beginTransmission(MD25ADDRESS);
	Wire.write(SPEED1);
	Wire.write(128);
	Wire.endTransmission();
} 

void MD25::encReset() { // This function resets the encoder values to 0
	Wire.beginTransmission(MD25ADDRESS);
	Wire.write(CMD);
	Wire.write(0x20); // Putting the value 0x20 to reset encoders
	Wire.endTransmission();
}

long MD25::encoder1() { // Function to read value of encoder 1 as a long
	Wire.beginTransmission(MD25ADDRESS); // Send byte to get a reading from encoder 1
	Wire.write(ENCODERTWO);
	Wire.endTransmission();

	Wire.requestFrom(MD25ADDRESS, 4); // Request 4 bytes from MD25
	while(Wire.available() < 4) { Serial.print("Waiting "); Serial.print(Wire.available()); Serial.println(millis()); } // Wait for 4 bytes to arrive
	long poss1 = Wire.read(); // First byte for encoder 1, HH.
	poss1 <<= 8;
	poss1 += Wire.read(); // Second byte for encoder 1, HL
	poss1 <<= 8;
	poss1 += Wire.read(); // Third byte for encoder 1, LH
	poss1 <<= 8;
	poss1 +=Wire.read(); // Fourth byte for encoder 1, LL
  delay(5);
	return(poss1);
}

long MD25::encoder2() { // Function to read value of encoder 2 as a long
	Wire.beginTransmission(MD25ADDRESS);
	Wire.write(ENCODERTWO);
	Wire.endTransmission();

	Wire.requestFrom(MD25ADDRESS, 4); // Request 4 bytes from MD25
	while(Wire.available() < 4); // Wait for 4 bytes to become available
	long poss2 = Wire.read();
	poss2 <<= 8;
	poss2 += Wire.read();
	poss2 <<= 8;
	poss2 += Wire.read();
	poss2 <<= 8;
	poss2 +=Wire.read();
  delay(5);
	return(poss2);
}

int MD25::volts() { // Function to read battery volts as a single byte
	Wire.beginTransmission(MD25ADDRESS); // Send byte to read volts
	Wire.write(VOLTREAD);
	Wire.endTransmission();

	Wire.requestFrom(MD25ADDRESS, 1);
	while(Wire.available() < 1);
	int batteryVolts = Wire.read();

	return (batteryVolts); // sum of ten readings
}

void MD25::printEnc() {
	int encodeVal1 = encoder1();
	Serial.print("encoder1: ");
	Serial.print(encodeVal1, DEC);
	Serial.print("\t");
	int encodeVal2 = encoder2();
	Serial.print("encoder2: ");
	Serial.println(encodeVal2, DEC);
	Serial.println();
}

