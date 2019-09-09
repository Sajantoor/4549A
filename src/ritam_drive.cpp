#include "main.h"
#include "drive.h"
#include "motor_setup.h"
#include "ritam_drive.h"


typedef struct pid_terms{
  float Kp;
  float Kd;
  float Ki;
  float integral_limit;

  float error;
  float last_error;
  float power;
  float integral_active_zone;
}

pid_terms;


float power_limit(float allowed_speed, float actual_speed){
   if (actual_speed > allowed_speed){
       actual_speed = allowed_speed;
   }

   if (actual_speed < -allowed_speed){
       actual_speed = -allowed_speed;
   }

   return actual_speed;
 }

 void pid_init(pid_terms *pid, float Kp, float Ki, float Kd, float integral_limit, float integral_active_zone){
   pid->Kp = Kp;
   pid->Ki = Ki;
   pid->Kd = Kd;

   pid->integral_limit = integral_limit;
   pid->error = 0;
   pid->last_error = 0;
   pid->power = 0;
   pid->integral_active_zone = integral_active_zone;
 }

 float pid_cal(pid_terms *pid, float target, float sensor) {
   pid->error = target - sensor;
   float derivative = (pid->error - pid->last_error);
   pid->last_error = pid->error;
   float proportional = pid->error;
   float integral = pid->error + integral;

   if (integral*pid->Ki > pid->integral_limit){
     integral = pid->integral_limit;
   }
   if (integral*pid->Ki < -pid->integral_limit){
     integral = -pid->integral_limit;
   }

   if(fabs(pid->error) > pid->integral_active_zone){
     integral = 0;
   }

   pid->power = (proportional*pid->Kp) + (integral*pid->Ki) + (derivative*pid->Kd);

   return pid->power;
 }
