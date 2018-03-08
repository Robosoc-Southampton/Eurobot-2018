
#ifndef PID_h
#define PID_h

class PID {
	float Pp, Pi, Pd;
	float PID_value;
	float previous_error;

	PID(float Pp, float Pi, float Pd);

	void calculatePID(float enc_val_cur, float target_val);
};

#endif
