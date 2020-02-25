#ifndef VISION_H
#define VISION_H

#include "main.h"

struct data {
  int width; // effective width of the cube
  int height; // effective height of the cube
  int size; // area, width * height
  int x;
  int y;
  int id;
  bool deepVisionCheck;
};

void vision_tracking(void*ignore);
void visionMovement(void*ignore);
extern pros::task_t vision_task;
extern pros::task_t visionMovement_task;
extern data currentCube;
const int CUBE_SIZE_THRESHOLD_MIN = 6708; // make bigger if detecting unwanted cubes, smaller if not picking up cubes
const int CUBE_SIZE_THRESHOLD_MAX = 6000; // make smaller if not intaking cubes
const int CENTER_X = -70;
// extern pros::vision_object_s_t;
#endif
