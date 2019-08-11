#ifndef RITAM_DRIVE_H_
#define RITAM_DRIVE_H_

#include "main.h"


float power_limit(float allowed_speed, float actual_speed);

 typedef struct pid_terms pid_terms;


 void pid_init(pid_terms *pid, float Kp, float Ki, float Kd, float integral_limit, float integral_active_zone);

 float pid_cal(pid_terms *pid, float target, float sensor);


#endif
