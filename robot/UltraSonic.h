
#ifndef UltraSonic_h
#define UltraSonic_h

#include <arduino.h>

const int MAX_VALUES = 4;

class UltraSonic {
public:
	int triggerPin, echoPin;
	unsigned int readings;
	float *lastValues;

	UltraSonic(int triggerPin, int echoPin);

	void fillLastValues();
	float getValue();

	bool allBelowThreshold(float threshold);
};

#endif
