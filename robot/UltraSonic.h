
#ifndef UltraSonic_h
#define UltraSonic_h

#include <Arduino.h>

const int MAX_VALUES = 2;

class UltraSonic {
public:
	int triggerPin, echoPin;
	unsigned int readings;
	float *lastValues;
  float lastValue;
  long lastTime;

	UltraSonic(int triggerPin, int echoPin);

	void fillLastValues();
	float getValue();

	bool allBelowThreshold(float threshold);
};

#endif
