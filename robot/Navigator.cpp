#include "Navigator.h"

Navigator::Navigator(Driver *driver, int x, int y, float heading, int xLength, int yLength) {
  this->driver = driver;
  this->x = x;
  this->y = y;
  this->heading = heading;
  this->xLength = xLength;
  this->yLength = yLength;
  numObstacles = 0;
}

void Navigator::navigate(int x, int y)
{
  int distance = TOLERANCE + 1;
  while(distance > TOLERANCE) {
    distance = (int)sqrt(sq((long)(x - this->x)) + sq((long)(y - this->y)));
    
    fx = 0;
    fy = 0;

    for(int i = 0; i < numObstacles && i < MAX_OBSTACLES; i++) {
      applyForce(this->x, this->y, obstacles[i]->x, obstacles[i]->y, -obstacles[i]->repulsion);
    }

    applyForce(this->x, this->y, x, y, DESTINATION_ATTRACTION);

    float absForce = sqrt(sq((float)fx) + sq((float)fy));
    float unitfx = (float)fx / absForce;
    float unitfy = (float)fy / absForce;
    float angle = atan2(unitfy, unitfx);
    driver->turnAtSpot(heading - angle);
    heading = angle;
    int travelled = driver->forward(distance, TIME_PER_RETARGET);
    x = (int)(unitfx * travelled);
    y = (int)(unitfy * travelled);
  }
}

void Navigator::applyForce(int x1, int y1, int x2, int y2, int mass) {
  int dx = x2 - x1;
  int dy = y2 - y1;
  int distance = (int)sqrt(sq((long)dx) + sq((long)dy));
  int force = (int)((long)mass / sq((long)distance));
  float unitX = (float)dx / distance;
  float unitY = (float)dy / distance;
  fx += unitX * force;
  fy += unitY * force;
}

void Navigator::addObstacle(Obstacle *obstacle)
{
  obstacles[numObstacles++ % MAX_OBSTACLES] = obstacle;
}
