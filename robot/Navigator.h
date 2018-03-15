#ifndef CONTROL_H
#define CONTROL_H

#define MAX_OBSTACLES 10

class Navigator {
 public:
  Navigator(Driver *driver);
  void navigate(int x, int y);
  void addObstacle(Obstacle *obstacle);
 private:
  Driver *driver;
  int x;
  int y;
  int width;
  int height;
  float heading;
  int numObstacles;
  Obstacle obstacles[MAX_OBSTACLES];
};

#endif
