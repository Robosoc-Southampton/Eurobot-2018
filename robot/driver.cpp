/* This library for Arduino was created by Daniel Hausner and debugged with a help of Taiwo Khourie, George Hadjigeorgioua
 * Shadi Hamou and Bethany Harding in Nov 2017 for odometry assignment at University of Southampton.
 * It eases interaction with MD25 driver.
 * dh4n16@soton.ac.uk
 */

#include "driver.h"


Driver::Driver(UltraSonic *sensors, unsigned int sensorCount, float Pp, float Pi, float Pd, float Pp_t, float Pi_t, float Pd_t, int limit_correction, int limit_correction_turning, int circumference, float wheel_dist) { // constructor, PID constants
  this->sensors = sensors;
  this->sensorCount = sensorCount;
  Kp = Pp;
  Ki = Pi;
  Kd = Pd;
  Kp_t = Pp_t;
  Ki_t = Pi_t;
  Kd_t = Pd_t;
  cir = circumference;
  w_dist = wheel_dist;
  limit_cor = limit_correction;
  limit_cor_turn = limit_correction_turning;
  error = 0;
  previous_error = 0;
  md = new MD25(0, 2); // parametres: mode, acceleration (ints 1-10)
  counter = 0; // counter for forward()
  error_sum = 0; // cumulated error for terminating the forward()
  pi = 3.14159;
  counter = 0;
  error_sum = 0;

  for (int i = 0; i < sensorCount; ++i) sensors[i].fillLastValues();
}

void Driver::setup() {
  md->setup();
}

int Driver::forward(int dist, long timeout, bool sense) {
  md->encReset(); // reset encoders
  delay(10);
  int enc1, enc_target;
  counter = error_sum = 0;
  long start_time = millis();

  do {
    // if (sense) sensors[0].getValue();
    // if (sense && sensorCount == 2) sensors[1].getValue();

    float sensorDistance = sensors[1].getValue();

    Serial.println(sensorDistance);

    if (sense && sensorCount == 2 && dist < 0 && sensors[1].allBelowThreshold(PROXIMITY_THRESHOLD)) {
      Serial.println("Stopping here");
      md->stopMotors();
      delay(2000);
      return getDistance(md->encoder1());
    }
    /*if (sense && dist > 0 && sensors[0].getValue() < PROXIMITY_THRESHOLD) {
      md->stopMotors();
      delay(50);
      return getDistance(md->encoder1());
    }*/
    
    enc_target = getEncVal(dist); // get target value for encoders
    enc1 = md->encoder1(); // asign current value of encoder1 to var enc1
    calculatePid(enc1, enc_target); // calculate PID value and assign it to private var PID_speed_limited
    md->setSpeed(PID_speed_limited, PID_speed_limited);
   
    if (terminatePid()) break;
  } while((millis() - start_time) < timeout);

  md->stopMotors();
  return getDistance(md->encoder1());
}

int Driver::forwardUntilLine(PololuQTRSensors sensor, long timeout) {
  md->encReset(); // reset encoders
  delay(10);
  counter = error_sum = 0;
  long start_time = millis();

  do {
    int speed = 255 - limit_cor;
    md->setSpeed(speed, speed);
    
    if (senseLine(sensor)) {
      break;
    }
  } while((millis() - start_time) < timeout);

  md->stopMotors();

  return getDistance(md->encoder1());
}

void Driver::turnAtSpot(float angle, long timeout) {
  md->encReset(); // reset encoders
  delay(10);
  int arc = int((angle/360) * (pi*w_dist));
  int dist = arc;
  int enc1, enc_target = 0;
  counter, error_sum = 0;
  long start_time = millis();
  do {
    enc_target = -getEncVal(dist); // get target value for encoders
    enc1 = md->encoder1();
    calculatePidTurn(enc1, enc_target);
    int spd1 = PID_speed_limited;
    int spd2 = 128 - (spd1 - 128);
    printEnc();
    md->setSpeed(spd2, spd1);
    if (terminatePid()) {
      break;
    }
  } while((millis() - start_time) < timeout);
  md->stopMotors();
}

void Driver::turn(int rad, int angle, char side, long timeout) {
  md->encReset();
  delay(10);
  float arc_portion = (float(angle)/360);
  int arc = round((((float(rad) + (w_dist/2)) )*arc_portion*2*pi)); // length of outer arc
  float speed_ratio = (float(rad) - (w_dist/2)) / (float(rad) + w_dist/2);
  int dist = arc;
  int spd1, spd2, enc, enc_target = 0;
  counter, error_sum = 0;
  long start_time = millis();
  do {
    if (side == 'L') {
      enc_target = getEncVal(dist); // get target value for encoders
      enc = md->encoder2();
      calculatePid(enc, enc_target);
      spd1 = round(float(PID_speed_limited - 128) * speed_ratio) + 128;
      spd2 = PID_speed_limited;
    }
    else {
      enc_target = getEncVal(dist); // get target value for encoders
      enc = md->encoder1();
      calculatePid(enc, enc_target);
      spd1 = PID_speed_limited;
      spd2 = round(float(PID_speed_limited - 128) * speed_ratio) + 128;
    }
    bool t = terminatePid();
    if (t) {
      break;
    }
    md->setSpeed(spd1, spd2);
  } while((millis() - start_time) < timeout);
// md->stopMotors();
}

void Driver::printPid() {
  /*Serial.print("error: ");
  Serial.println(error);
  Serial.print("P: ");
  Serial.println(P);
  Serial.print("I: ");
  Serial.println(I);
  Serial.print("D: ");
  Serial.println(D);
  Serial.print("PID_val: ");
  Serial.println(PID_val);
  Serial.print("PID_speed_theor: ");
  Serial.println(PID_speed_theor);
  Serial.print("PID_speed_limited: ");
  Serial.println(PID_speed_limited);
  Serial.println();*/
}

void Driver::printEnc() {
  /*int encodeVal1 = md->encoder1();
  Serial.print("encoder1: ");
  Serial.print(encodeVal1, DEC);
  Serial.print("\t");
  int encodeVal2 = md->encoder2();
  Serial.print("encoder2: ");
  Serial.println(encodeVal2, DEC);
  Serial.println();*/
}



// HELP FUNCTIONS

bool Driver::senseLine(PololuQTRSensors sensor) {
  unsigned int readings[3];
  unsigned int threshold = 800;

  sensor.readLine(readings);

  for (int i = 0; i < 3; ++i) {
    if (readings[i] > threshold) return true;
  }

  return false;
}

int Driver::getDistance(int enc_val) {
  return int(enc_val / 360.f * cir);
}

int Driver::getEncVal(int dist) { // returns encoder value to be set to drive required distance
  int enc_count = int(((float(dist)/cir) * 360));
  return enc_count;
}

int Driver::getSpeed(int speed) {
  if (speed >= 255 - limit_cor) {
    return (255 - limit_cor);
  }
  if (speed <= 0 + limit_cor) {
    return (0 + limit_cor);
  }
  return speed;
}

int Driver::getSpeedTurn(int speed) {
  if (speed >= 255 - limit_cor_turn) {
    return (255 - limit_cor_turn);
  }
  if (speed <= 0 + limit_cor_turn) {
    return (0 + limit_cor_turn);
  }
  return speed;
}

void Driver::calculatePid(int enc_val_cur, int target_val) {
  error = target_val - enc_val_cur;
  P = error; // proportional
  I = I + error; // integral
  D = error - previous_error; // derivative
  PID_val = (Kp*P) + (Ki*I) + (Kd*D); // not sure about the sign after 128
  PID_speed_theor = 128 + int(PID_val);
  PID_speed_limited = getSpeed(PID_speed_theor);
  previous_error = error;
}

void Driver::calculatePidTurn(int enc_val_cur, int target_val) {
  error = target_val - enc_val_cur;
  P = error; // proportional
  I = I + error; // integral
  D = error - previous_error; // derivative
  PID_val = (Kp_t*P) + (Ki_t*I) + (Kd_t*D); // not sure about the sign after 128
  PID_speed_theor = 128 + int(PID_val);
  PID_speed_limited = getSpeedTurn(PID_speed_theor);
  previous_error = error;
}

bool Driver::readingPeriod() {
  cur_time = millis();
  if (cur_time > (prev_time + period)) {
    prev_time = cur_time;
    return true;
  }
  return false;
}

bool Driver::terminatePid() {
  error_sum += abs(error);
  counter += 1;
  if (counter >= 20) {
    if (error_sum <= 20) {
      return true;
    }
    else {
      counter = 0;
      error_sum = 0;
      return false;
    }
  }
  return false;  
}

// 

