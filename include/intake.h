#ifndef INTAKE_H
#define INTAKE_H
#include "main.h"

extern float intakeSpeed;
extern bool intakeBool;
void autoIntake(void*ignore);
void autoIntakeFunc(float speed);
extern pros::task_t intake_task_init;

#endif
