#include "main.h"
#include "motor_sensor_init.h"

pros::Motor drive_left(DRIVE_LEFT, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor drive_left_b(DRIVE_LEFT_B, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor drive_right(DRIVE_RIGHT, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor drive_right_b(DRIVE_RIGHT_B, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);


pros::Motor loader_right(LOADER_RIGHT, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor loader_left(LOADER_LEFT, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor angler(ANGLER, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor arm(ARM, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::ADIPort potentiometer_arm (pot_port_arm, pros::E_ADI_ANALOG_IN);
pros::ADIPort potentiometer_angler (pot_port_angler, pros::E_ADI_ANALOG_IN);
pros::ADIEncoder left_encoder(top_port,bottom_port,false);
pros::ADIEncoder right_encoder(top_port_right,bottom_port_right,false);
pros::ADIEncoder back_encoder(top_port_back,bottom_port_back,true);
