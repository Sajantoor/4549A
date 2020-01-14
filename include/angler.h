#ifndef ANGLER_H
#define ANGLER_H
#include "main.h"

extern float nextTarget;
extern bool anglerBool;
void angler_pid_task(void*ignore);
void angler_pid(float target, float delayTime, float speed = 80, bool ApplyTorque = true);

extern pros::task_t angler_task;

#endif
