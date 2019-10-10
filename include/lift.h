#ifndef LIFT_H
#define LIFT_H
#include "main.h"

extern float lift_val;
extern bool liftTask;

void lift(float lift_target);
void lift_task_t(void*ignore);
extern pros::task_t lift_task;


#endif
