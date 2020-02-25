#ifndef INTAKE_H
#define INTAKE_H
#include "main.h"

void sensor_outtake();
extern bool intakeTaskBool;
extern bool autoIntakeBool;
extern bool sensorOutakeBool;
extern pros::task_t intake_task_init;
extern pros::task_t intake_task_pid_init;
extern pros::task_t intake_sensor_task_init;
void autoIntake(void*ignore);
void autoIntakeFunc(float speed);
void intakePID(void*ignore);
void intakePIDFunc(float target, float speed);
void sensor_outtake_task(void*ignore);

#endif
