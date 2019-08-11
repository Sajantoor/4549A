#include "main.h"
#include "all_used.h"
#include "drive.h"
#include "math.h"

float nearestangle(float target_angle, float reference_angle)
{
  return round((reference_angle-target_angle) / (2 * pi)) *  (2 * pi) + target_angle;
}

float flmod(float x, float y)
{
	int q = floor(x / y);
	return x - (float)q * y;
}

float degToRad(float degrees)
{
	return degrees * pi / 180;
}

float radToDeg(float radians)
{
	return radians * 180 / pi;
}
