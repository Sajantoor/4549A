#include "main.h"
#include "pid.h"

// checks allowed speed vs actual speed and returns as float
float power_limit(float allowed_speed, float actual_speed){
   if (actual_speed > allowed_speed) {
       actual_speed = allowed_speed;
   }

   else if (actual_speed < -allowed_speed) {
       actual_speed = -allowed_speed;
   }

   return actual_speed;
 }

// struct constructor definition example
//  pid_values name(0.7, 0.5, 0.25, 123, 123, 123);

// Calculates power and returns power as float
 float pid_calc(pid_values *pid, float target, float sensor) {
   pid->error = target - sensor;
   float derivative = (pid->error - pid->last_error);
   pid->last_error = pid->error;
   float proportional = pid->error;
   float integral = pid->error + integral;

   if (integral*pid->Ki > pid->integral_limit) {
     integral = pid->integral_limit;
   }

   if (integral*pid->Ki < -pid->integral_limit) {
     integral = -pid->integral_limit;
   }

   // BUG: Comment this out if this doesn't work
   // 
   // if (fabs(pid->error) > pid->integral_active_zone) {
   //   integral = 0;
   // }

   pid->power = (proportional*pid->Kp) + (integral*pid->Ki) + (derivative*pid->Kd);

   return pid->power;
 }
