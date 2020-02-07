#ifndef INTAKE_H
#define INTAKE_H
#include "main.h"

void sensor_outtake();
extern bool intakeBool;
void autoIntake(void*ignore);
void autoIntakeFunc(float speed);
void intakePID(void*ignore);
void intakePIDFunc(float target, float speed);
extern pros::task_t intake_task_init;
extern pros::task_t intake_task_pid_init;

#endif
