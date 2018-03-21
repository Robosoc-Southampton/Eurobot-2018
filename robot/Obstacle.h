#ifndef OBSTACLE_H
#define OBSTACLE_H

#define DEFAULT_REPULSION 1000

class Obstacle {
 public:
  Obstacle(int x, int y, int repulsion = DEFAULT_REPULSION);
  int x, y, repulsion;
};

#endif
