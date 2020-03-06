#include "main.h"
#include "motor_sensor_init.h"

pros::Motor drive_left(DRIVE_LEFT, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor drive_left_b(DRIVE_LEFT_B, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor drive_right(DRIVE_RIGHT, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor drive_right_b(DRIVE_RIGHT_B, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor intake_right(INTAKE_RIGHT, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor intake_left(INTAKE_LEFT, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor angler(ANGLER, MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor arm(ARM, MOTOR_GEARSET_6, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::Vision vision_sensor(VISION_PORT);

pros::ADIGyro gyro(GYRO_PORT);
pros::ADIPort light_sensor(LIGHT_SENSOR_PORT, pros::E_ADI_ANALOG_IN);
pros::ADIEncoder left_encoder(TOP_PORT_LEFT, BOTTOM_PORT_LEFT, false);
pros::ADIEncoder right_encoder(TOP_PORT_RIGHT, BOTTOM_PORT_RIGHT, false);
pros::ADIEncoder back_encoder(TOP_PORT_BACK, BOTTOM_PORT_BACK, true);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
