#ifndef INTAKE_H
#define INTAKE_H
#include "main.h"

extern int speed;
extern int run_time;

extern pros::task_t loader_task;

void intake(void *ignore);

#endif
