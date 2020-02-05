#ifndef MOTOR_SETUP_INIT_H
#define MOTOR_SETUP_INIT_H
#include "main.h"

#define DRIVE_LEFT 9
#define DRIVE_LEFT_B 3
#define DRIVE_RIGHT 6
#define DRIVE_RIGHT_B 4

#define LOADER_RIGHT 20
#define LOADER_LEFT 12
#define ANGLER 10
#define ARM 5
#define VISION_PORT 20

extern pros::Motor drive_left;
extern pros::Motor drive_left_b;
extern pros::Motor drive_right;
extern pros::Motor drive_right_b;
extern pros::Motor arm;
extern pros::Motor loader_left;
extern pros::Motor loader_right;
extern pros::Motor angler;

#define light_port_intake 1
#define pot_port_angler 2
#define top_port_back 3
#define bottom_port_back 4
#define top_port_right 5
#define bottom_port_right 6
#define top_port 7
#define bottom_port 8
// #define gyro_port 11;

extern pros::ADIPort light_sensor_intake;
// extern pros::ADIGyro gryo;
extern pros::ADIPort potentiometer_angler;
extern pros::ADIEncoder left_encoder;
extern pros::ADIEncoder right_encoder;
extern pros::ADIEncoder back_encoder;

extern pros::Vision vision_sensor;
extern pros::Controller controller;


#endif
