#ifndef CONTROL_H
#define CONTROL_H

#include "driver.h"
#include "Obstacle.h"

#define MAX_OBSTACLES 10
#define DESTINATION_ATTRACTION 2000;
#define TIME_PER_RETARGET 500;

class Navigator {
 public:
  Navigator(Driver *driver);
  void navigate(int x, int y);
  void addObstacle(Obstacle *obstacle);
 private:
  Driver *driver;
  int x, y;
  int xLength, yLength;
  float heading;
  int numObstacles;
  Obstacle obstacles[MAX_OBSTACLES];
  void applyForce(int x1, int y1, int x2, int y2, int mass);
  int fx, fy;
};

#endif
