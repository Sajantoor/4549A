#include "main.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"

void drive_set(int speed) {
  drive_left.move(speed);
  drive_left_b.move(speed);
  drive_right.move(speed);
  drive_right_b.move(speed);
}

void turn_set(int speed) {
  drive_left.move(speed);
  drive_left_b.move(speed);
  drive_right.move(-speed);
  drive_right_b.move(-speed);
}

void strafe(int speed) {
  drive_left.move(-speed);
  drive_left_b.move(speed);
  drive_right.move(speed);
  drive_right_b.move(-speed);
}

void left_drive_set(int speed) {
  drive_left.move(speed);
  drive_left_b.move(speed);
}

void right_drive_set(int speed) {
  drive_right.move(speed);
  drive_right_b.move(speed);
}

void reset_drive_encoders() {
  drive_left.tare_position();
  drive_left_b.tare_position();
  drive_right.tare_position();
  drive_right_b.tare_position();
  left_encoder.reset();
  right_encoder.reset();
}

void set_drive(int left_speed, int right_speed) {
  left_drive_set(left_speed);
  right_drive_set(right_speed);
}
