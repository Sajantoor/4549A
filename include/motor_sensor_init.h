#ifndef MOTOR_SETUP_INIT_H

#include "main.h"

#define DRIVE_LEFT 20
#define DRIVE_LEFT_B 18
#define DRIVE_RIGHT 11
#define DRIVE_RIGHT_B 13

#define LOADER 7
#define ANGLER 10
#define STACKER 8
#define PUNCHER 5



extern pros::Motor drive_left;
extern pros::Motor drive_left_b;
extern pros::Motor drive_right;
extern pros::Motor drive_right_b;
extern pros::Motor puncher;
extern pros::Motor loader;
extern pros::Motor angler;
extern pros::Motor stacker;
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



// extern pros::ADIPort potentiometer (pot_port, pros::E_ADI_ANALOG_IN);
// extern pros::ADIGyro gyro (gyro_port, 0.9192647);
// extern pros::ADIEncoder left_encoder(top_port,bottom_port,false);
// extern pros::ADIEncoder right_encoder(top_port_right,bottom_port_right,true);



#endif
