#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "driver.h"
#include "Obstacle.h"
#include "UltraSonic.h"

#define MAX_OBSTACLES 10
#define DESTINATION_ATTRACTION 2000
#define TIME_PER_RETARGET 500
#define TOLERANCE 10

class Navigator {
 public:
  Navigator(Driver *driver, UltraSonic *sensor, int x, int y, float heading, int xLength, int yLength);
  void navigate(int x, int y);
  void addObstacle(Obstacle *obstacle);
 private:
  Driver *driver;
  UltraSonic *sensor;
  int x, y;
  int xLength, yLength;
  float heading;
  int numObstacles;
  Obstacle *obstacles[MAX_OBSTACLES];
  void applyForce(int x1, int y1, int x2, int y2, int mass);
  int fx, fy;
};

#endif
