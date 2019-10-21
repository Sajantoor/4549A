#ifndef ALL_USED_H
#define ALL_USED_H

#include "main.h"
#include "drive.h"

#define pi 3.14159265


#define limit_to_val(input, val) (abs(input) > (val) ? (val) * sgn(input) : (input))

#define limit_to_val_set(input, val) input = limit_to_val(input, val)

void full_position_reset();
void resetPositionFull(vector& position, float y, float x, float a);

float flmod(float x, float y); // Floating point mod operation
float degToRad(float degrees); // Convert degrees to radians
float radToDeg(float radians); // Convert radians to degrees
float nearestangle(float target_angle, float reference_angle);
void reset_position_full(vector& gposition, float x, float y, float a); // Reset the position to a desired value and starts tracking

#endif
