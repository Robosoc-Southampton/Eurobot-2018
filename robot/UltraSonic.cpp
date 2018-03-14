
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
	while (readings < MAX_VALUES) getValue();
}

float UltraSonic::getValue() {
	long duration;
	int distance;
	digitalWrite(triggerPin, LOW);
	delayMicroseconds(2);
	digitalWrite(triggerPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(triggerPin, LOW);

	duration = pulseIn(echoPin, HIGH);
	distance = duration * 0.017;

	// record value
	lastValues[(readings++) % MAX_VALUES] = distance;

	return distance;
}

bool UltraSonic::allBelowThreshold(float threshold) {
	for (auto idx = 0; idx < readings; ++idx) {
		if (lastValues[idx] > threshold) return false;
	}

	return true;
}
