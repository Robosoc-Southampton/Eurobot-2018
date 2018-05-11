
#include "time.h"

void init() {
  START_TIME = millis();
}

void assertTimeLeft() {
  if (millis() - START_TIME >= 85000) {
    exit(0);
  }
}

