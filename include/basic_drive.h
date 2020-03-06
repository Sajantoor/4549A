#ifndef MOTOR_SETUP_H

#include "main.h"

void drive_set(int speed);
void turn_set(int speed);
void strafe(int speed);
void left_drive_set(int speed);
void right_drive_set(int speed);
void set_drive(int right_speed, int left_speed);
void reset_drive_encoders();

#endif
