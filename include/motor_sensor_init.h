#ifndef MOTOR_SETUP_INIT_H
#define MOTOR_SETUP_INIT_H
#include "main.h"

#define DRIVE_LEFT 11
#define DRIVE_LEFT_B 18
#define DRIVE_RIGHT 12
#define DRIVE_RIGHT_B 4

#define LOADER_RIGHT 9
#define LOADER_LEFT 6
#define ANGLER 7
#define ARM 10
#define VISION_PORT 17

extern pros::Motor drive_left;
extern pros::Motor drive_left_b;
extern pros::Motor drive_right;
extern pros::Motor drive_right_b;
extern pros::Motor arm;
extern pros::Motor loader_left;
extern pros::Motor loader_right;
extern pros::Motor angler;

#define light_port_intake 1
#define pot_port_angler 10
#define top_port_back 3
#define bottom_port_back 4
#define top_port_right 5
#define bottom_port_right 6
#define top_port 7
#define bottom_port 8
#define GYRO_PORT 2 //2 is weird
// #define gyro_port 11;

extern pros::ADIPort light_sensor_intake;
extern pros::ADIGyro gyro;
extern pros::ADIPort potentiometer_angler;
extern pros::ADIEncoder left_encoder;
extern pros::ADIEncoder right_encoder;
extern pros::ADIEncoder back_encoder;

extern pros::Vision vision_sensor;
extern pros::Controller controller;


#endif
