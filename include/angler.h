#ifndef ANGLER_H
#define ANGLER_H
#include "main.h"

extern float nextTarget;
extern bool anglerBool;
extern bool anglerIntakeThreshold;
extern bool anglerHold;
void angler_pid_task(void*ignore);
void angler_pid(float target, bool holdVal, float speed = 80, bool ApplyTorque = true, float delayTime = 0);

extern pros::task_t angler_task;

#endif
