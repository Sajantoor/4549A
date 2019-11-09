#ifndef ANGLER_H
#define ANGLER_H
#include "main.h"

extern float nextTarget;
void angler_pid_task(void*ignore);
void angler_pid(float target, float delayTime);

extern pros::task_t angler_task;

#endif
