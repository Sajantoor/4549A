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
extern float prev_inches_traveled_back;
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

typedef struct _vel
{
	float a;
	float y;
	float x;
	unsigned long lstChecked;
	float lstPosA;
	float lstPosY;
	float lstPosX;
} sVel; // Velocity of the robot

extern sVel velocity;


typedef enum turnDir
{
	cw,
	ccw,
	ch
} tTurnDir;

void vectorToPolar(vector& vector, polar& polar);

void polarToVector(polar& polar, vector& vector);

void drive_line_up (int speed, int run_time_drive);

//--------------------------------------POSITION-------------------------------------------------------

float nearestangle(float target_angle, float reference_angle);

void tracking_update(void*ignore);

void tracking_velocity(void*ignore);

void position_turn(float target, int timeout, int max_speed);

void position_turn2(float target_angle, tTurnDir turnDir, float ratio_full, int coast_power, float stop_offset_deg);

void position_face_point2(float target_x, float target_y, tTurnDir turnDir, float ratio_full, float coast_power, float offset, float stopOffsetDeg);

void position_face_point(float target_x, float target_y , int timeout);

void position_drive(float ending_point_x, float ending_point_y, float target_angle, float max_power, unsigned int timeout);
extern pros::task_t tracking_task;
extern pros::task_t velocity_task;

#endif
