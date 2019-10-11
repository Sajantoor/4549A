#ifndef LIFT_H
#define LIFT_H
#include "main.h"

extern float lift_val;
extern bool liftTask;

void lift(int value);
void lift_task(void*ignore);
extern pros::task_t lift_task_init;


#endif
