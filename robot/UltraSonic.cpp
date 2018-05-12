
#include "UltraSonic.h"

UltraSonic::UltraSonic(int triggerPin, int echoPin) {
	this->triggerPin = triggerPin;
	this->echoPin = echoPin;
	pinMode(triggerPin, OUTPUT);
	pinMode(echoPin, INPUT);
	readings = 0;
	lastValues = (float*) calloc(MAX_VALUES, sizeof(float));
}

void UltraSonic::fillLastValues() {
  int i = 0;
	while (i < MAX_VALUES) {
	  lastValues[i++] = getValue();
    readings = i;
	}
}

float UltraSonic::getValue() {
  long time = millis();

  if (time - lastTime < 64) return lastValue;
  
	long duration;
	float distance;
	digitalWrite(triggerPin, LOW);
	delayMicroseconds(2);
	digitalWrite(triggerPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(triggerPin, LOW);

	duration = pulseIn(echoPin, HIGH, 250000);
	distance = duration * 0.017;

  if (distance < 2.0) distance = 100.0;

	// record value
	lastValues[(readings++) % MAX_VALUES] = distance;
  lastValue = distance;
  lastTime = time;

	return distance;
}

bool UltraSonic::allBelowThreshold(float threshold) {
	for (int idx = 0; idx < MAX_VALUES; ++idx) {
		if (lastValues[idx] > threshold) return false;
	}

	return true;
}
