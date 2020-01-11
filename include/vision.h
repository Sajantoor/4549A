#ifndef VISION_H
#define VISION_H

#include "main.h"

void vision_tracking(void*ignore);
void visionMovement(void*ignore);
extern pros::task_t vision_task;
extern pros::task_t visionMovement_task;
// extern pros::vision_object_s_t;
#endif
