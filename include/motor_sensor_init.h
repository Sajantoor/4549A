#ifndef MOTOR_SETUP_INIT_H
#define MOTOR_SETUP_INIT_H
#include "main.h"

#define DRIVE_LEFT 9
#define DRIVE_LEFT_B 3
#define DRIVE_RIGHT 6
#define DRIVE_RIGHT_B 4

#define INTAKE_RIGHT 16
#define INTAKE_LEFT 13
#define ANGLER 10
#define ARM 5
#define VISION_PORT 17

extern pros::Motor drive_left;
extern pros::Motor drive_left_b;
extern pros::Motor drive_right;
extern pros::Motor drive_right_b;
extern pros::Motor arm;
extern pros::Motor intake_left;
extern pros::Motor intake_right;
extern pros::Motor angler;

#define LIGHT_SENSOR_PORT 1
#define GYRO_PORT 2
#define TOP_PORT_LEFT 7
#define BOTTOM_PORT_LEFT 8
#define TOP_PORT_RIGHT 5
#define BOTTOM_PORT_RIGHT 6
#define TOP_PORT_BACK 3
#define BOTTOM_PORT_BACK 4


extern pros::ADIGyro gyro;
extern pros::ADIPort light_sensor;
extern pros::ADIEncoder left_encoder;
extern pros::ADIEncoder right_encoder;
extern pros::ADIEncoder back_encoder;

extern pros::Vision vision_sensor;
extern pros::Controller controller;


#endif
