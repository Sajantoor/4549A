#define PID_H
#include "main.h"

// pid value data struct
typedef struct pid_values {
   float Kp, Kd, Ki, integral_limit, error, last_error, power, integral_active_zone, calc_power;
   float integral, derivative, proportional, target;
   int max_power;
   // constructor
   pid_values(float Kp, float Kd, float Ki, float integral_limit, float integral_active_zone, int max_power):
      Kp(Kp), Kd(Kd), Ki(Ki), integral_limit(integral_limit), integral_active_zone(integral_active_zone), max_power(max_power), error(0), last_error(0), power(0), calc_power(0) {
   }
 } pid_values;

float power_limit(float allowed_speed, float actual_speed);
float pid_calc(pid_values *pid, float target, float sensor);
