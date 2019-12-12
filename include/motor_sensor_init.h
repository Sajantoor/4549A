#ifndef MOTOR_SETUP_INIT_H
#define MOTOR_SETUP_INIT_H
#include "main.h"

#define DRIVE_LEFT 10
#define DRIVE_LEFT_B 6
#define DRIVE_RIGHT 1
#define DRIVE_RIGHT_B 5

#define LOADER_RIGHT 4
#define LOADER_LEFT 9
#define ANGLER 7
#define ARM 3

extern pros::Motor drive_left;
extern pros::Motor drive_left_b;
extern pros::Motor drive_right;
extern pros::Motor drive_right_b;
extern pros::Motor arm;
extern pros::Motor loader_left;
extern pros::Motor loader_right;
extern pros::Motor angler;

#define pot_port_arm 1
#define pot_port_angler 2
#define top_port_back 3
#define bottom_port_back 4
#define top_port_right 5
#define bottom_port_right 6
#define top_port 7
#define bottom_port 8

extern pros::ADIPort potentiometer_arm;
extern pros::ADIPort potentiometer_angler;
extern pros::ADIEncoder left_encoder;
extern pros::ADIEncoder right_encoder;
extern pros::ADIEncoder back_encoder;


#endif
