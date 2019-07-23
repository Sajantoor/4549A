#ifndef DRIVE_H
#define DRIVE_H

#include "main.h"

#define pi 3.14159265
extern float correction_turn;
extern float degrees_flag;
extern float prev_correction_turn;

extern float correction_drive;
extern float prev_correction_drive;
extern float drive_distance_correction;
extern float degrees_to_rad_left;
extern float degrees_to_rad_right;
extern float beginning_orientation;
extern float prev_inches_traveled_left;
extern float prev_inches_traveled_right;
extern float orientation;
struct vector
{
  float x;
  float y;
};
extern vector position;

struct polar
{
  float r;
  float theta;
};

template <typename T> int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

void drive_line_up (int speed, int run_time_drive);

void drive_pid(float target, unsigned int timeout = 150, int max_speed = 100);

void drive_pid_correction_switch(float target, unsigned int timeout, int max_speed, bool correction_switch);

void turn_pid(float degs, float Ki,  unsigned int timeout = 100);

//--------------------------------------ENCODERS-------------------------------------------------------

void drive_pid_encoder(float target, unsigned int timeout, int max_speed = 110, float Kp_C = 0.5);

void turn_pid_encoder_average(double target, unsigned int timeout);

//--------------------------------------POSITION-------------------------------------------------------

float nearestangle(float target_angle, float reference_angle);

void tracking_update(void*ignore);

void position_turn(float target, int timeout, float kp);

void position_face_point(float target_x, float target_y , int timeout);

void position_drive(float starting_point_x, float starting_point_y, float ending_point_x, float ending_point_y, float max_speed, float max_error, int timeout);

void position_drive_forward(float target_y, bool reverse, int timeout);

void math_test(float starting_point_x, float starting_point_y, float ending_point_x, float ending_point_y);

extern pros::task_t tracking_task;

#endif
