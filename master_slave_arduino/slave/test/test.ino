#include "PololuQTRSensors.h"

// create an object for your type of sensor (RC or Analog)
// in this example we have three sensors on analog inputs 0 - 2, a.k.a. digital pins 14 - 16
PololuQTRSensorsRC qtr((char[]) {
  15, 16
}, 2);

// PololuQTRSensorsA qtr((char[]) {0, 1, 2}, 3);

void setup()
{
  // optional: wait for some input from the user, such as  a button press
  // then start calibration phase and move the sensors over both
  // reflectance extremes they will encounter in your application:
  int i;
  for (i = 0; i < 250; i++)  // make the calibration take about 5 seconds
  {
    qtr.calibrate();
    delay(20);
  }

  Serial.begin(9600);
  // optional: signal that the calibration phase is now over and wait for further
  // input from the user, such as a button press
}

void loop()
{
  unsigned int sensors[6];
  // get calibrated sensor values returned in the sensors array, along with the line position
  // position will range from 0 to 2000, with 1000 corresponding to the line over the middle sensor
  int position = qtr.readLine(sensors);
  // if all three sensors see very low reflectance, take some appropriate action for this situation

Serial.print(sensors[0]);
Serial.print("\t");
Serial.print(sensors[1]);
Serial.print("\t");
Serial.print(sensors[2]);
Serial.print("\t");
Serial.print(sensors[3]);
Serial.print("\t");
Serial.print(sensors[4]);
Serial.print("\t");
Serial.println(sensors[5]);
delay(50);


  
  if (sensors[0] > 750 && sensors[1] > 750 && sensors[2] > 750)
  {
    // do something.  Maybe this means we're at the edge of a course or about to fall off a table,
    // in which case, we might want to stop moving, back up, and turn around.
    return;
  }
  // compute our "error" from the line position.  We will make it so that the error is zero when
  // the middle sensor is over the line, because this is our goal.  Error will range from
  // -1000 to +1000.  If we have sensor 0 on the left and sensor 2 on the right,  a reading of -1000
  // means that we see the line on the left and a reading of +1000 means we see the line on
  // the right.
  int error = position - 1000;
  int leftMotorSpeed = 100;
  int rightMotorSpeed = 100;
  if (error < -500)  // the line is on the left
    leftMotorSpeed = 0;  // turn left
  if (error > 500)  // the line is on the right
    rightMotorSpeed = 0;  // turn right
  // set motor speeds using the two motor speed variables above
}


