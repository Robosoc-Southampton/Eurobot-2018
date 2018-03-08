
#include "PID.h"

PID::PID(float Pp, float Pi, float Pd) {
	this->Pp = Pp;
	this->Pi = Pi;
	this->Pd = Pd;
	this->PID_value = 0.f;
	this->previous_error = 0.f;
}

void PID::calculatePID(float enc_val_cur, float target_val) {
	float error = target_val - enc_val_cur;
	float P = error; // proportional
	float I = I + error; // integral
	float D = error - previous_error; // derivative
	PID_value = (Pp*P) + (Pi*I) + (Pd*D); // not sure about the sign after 128
	previous_error = error;
}
