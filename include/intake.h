#ifndef INTAKE_H
#define INTAKE_H
#include "main.h"

extern float intakeSpeed;
void autoIntake(void*ignore);
extern pros::task_t intake_task_init;

#endif
