
#include "time.h"

void assertTimeLeft() {
  if (millis() - START_TIME >= 85000) {
    exit(0);
  }
}

