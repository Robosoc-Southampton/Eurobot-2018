
#include "time.h"

unsigned long START_TIME;

void init_timer() {
  START_TIME = millis();
}

void assertTimeLeft() {
  if (millis() - START_TIME >= 85000) {
    exit(0);
  }
}

