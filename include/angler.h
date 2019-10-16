#ifndef ANGLER_H
#define ANGLER_H
#include "main.h"

extern float target;

void angler_pid_task(void*ignore);
void angler_pid(float target);

extern pros::task_t angler_task;

#endif
