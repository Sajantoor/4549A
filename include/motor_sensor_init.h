#ifndef MOTOR_SETUP_INIT_H
#define MOTOR_SETUP_INIT_H
#include "main.h"

#define DRIVE_LEFT 20
#define DRIVE_LEFT_B 18
#define DRIVE_RIGHT 11
#define DRIVE_RIGHT_B 13

#define LOADER_RIGHT 7
#define LOADER_LEFT 5
#define ANGLER 10
#define ARM 11



extern pros::Motor drive_left;
extern pros::Motor drive_left_b;
extern pros::Motor drive_right;
extern pros::Motor drive_right_b;
extern pros::Motor arm;
extern pros::Motor loader_left;
extern pros::Motor loader_right;
extern pros::Motor angler;

#define pot_port 1
#define gyro_port 2
#define top_port_back 3
#define bottom_port_back 4
#define top_port_right 5
#define bottom_port_right 6
#define top_port 7
#define bottom_port 8

extern pros::ADIPort potentiometer;
extern pros::ADIGyro gyro;
extern pros::ADIEncoder left_encoder;
extern pros::ADIEncoder right_encoder;
extern pros::ADIEncoder back_encoder;


#endif
