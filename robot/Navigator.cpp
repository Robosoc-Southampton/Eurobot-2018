#include "Navigator.h"

Navigator::Navigator(Driver *driver, int x, int y, float heading, int width, int height)
  : driver(driver),
    x(x),
    y(y),
    heading(heading),
    width(width),
    height(height),
    numObstacles(0)
{}

void Navigator::navigate(int x, int y)
{
  
}

void Navigator::addObstacle(Obstacle *obstacle)
{
  obstacles[numObstacles++ % MAX_OBSTACLES] = *obstacle;
}
