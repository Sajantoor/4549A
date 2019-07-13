#include "main.h"
#include "motor_sensor_init.h"

pros::Motor drive_left(DRIVE_LEFT, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor drive_left_b(DRIVE_LEFT_B, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor drive_right(DRIVE_RIGHT, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor drive_right_b(DRIVE_RIGHT_B, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);


pros::Motor loader(LOADER, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor angler(ANGLER, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor stacker(STACKER, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor puncher(PUNCHER, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::ADIPort potentiometer (pot_port, pros::E_ADI_ANALOG_IN);
pros::ADIGyro gyro (gyro_port, 0.9192647);
pros::ADIEncoder left_encoder(top_port,bottom_port,false);
pros::ADIEncoder right_encoder(top_port_right,bottom_port_right,false);
pros::ADIEncoder back_encoder(top_port_back,bottom_port_back,true);
