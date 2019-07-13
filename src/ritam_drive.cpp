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

}pid_terms;


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

 float pid_cal(pid_terms *pid, float target, float sensor){

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


//
// void position_face_point2(float target_x, float target_y,int timeout)
// {
//     pid_terms *drive;
//     vector error;
//     // float Kp = 50; //0.2
//     //
//     // float kd = 0;//70   3
//     // float ki = 0;//0.06   0
//     //float proportional, derivative, integral;
//     float final_power;
//     float encoder_avg;
//     // int last_error = 0;
//     // int integral_limit = 50;
//
//     float calculated_power;
//
//     int max_speed = 110; //90
//     // float max_error = 0.001f;
//     bool timer_turn = true;
//     unsigned int net_timer;
//
//     int failsafe = 2000;    //varible value
//     int initial_millis = pros::millis();
//
//
//     float direction_face;
//     //net_timer = pros::millis() + timeout; //just to initialize net_timer at first
//
//     float encoder_average;
//     float error_p;
//
//     pid_init(drive, 50, 0, 0, 50, 0.383972);
//
//       do {
//        error.x = target_x - position.x;		//wheel size is 4.17 inches
//        error.y = target_y - position.y;
//
//        direction_face = atan2f(error.x , error.y);
//
//        encoder_avg = orientation;
//        // error_p = direction_face - encoder_avg;
//        // derivative = (error_p - last_error)*kd;
//        // last_error = error_p;
//        // integral = (error_p + integral);
//        // proportional = error_p*Kp;
//
//        calculated_power = pid_cal(drive, direction_face, encoder_avg);
//
//        final_power = power_limit(max_speed, calculated_power);
//
//          pros::lcd::print(6, "final_power %f\n", final_power);
//         turn_set(final_power);
//
//         if (timer_turn == true){
//           net_timer = pros::millis() + timeout;
//         }
//
//         if (fabs(error_p) < 0.0174533)//0.0349066
//         {   //if less than 2 degrees // ticks_to_deg = 3.18627451      (2*3.18627451) * (pi/180)
//           timer_turn = false;
//         }
//
//           pros::delay(20); //20
//
//       }
//         while((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis()));
//        turn_set(0);		//set drive to 0 power
// }


// pid_terms drive;
//
// pid_init(&drive, 80, 0, 0.5, 50, 0.383972);
//
// final_power = pid_cal(&drive, target, encoder_avg);



void position_face_point3(float target_x, float target_y,int timeout)
{
    pid_terms turn;
    vector error;

    float final_power;
    float encoder_avg;

    float calculated_power;

    int max_speed = 110; //90
    bool timer_turn = true;
    unsigned int net_timer;

    int failsafe = 2000;    //varible value
    int initial_millis = pros::millis();

    float direction_face;

    float encoder_average;
    float error_p;

    pid_init(&turn, 50, 0, 0, 50, 0.383972);

      do {
       error.x = target_x - position.x;		//wheel size is 4.17 inches
       error.y = target_y - position.y;

       direction_face = atan2f(error.x , error.y);
       encoder_avg = orientation;
       error_p = direction_face - encoder_avg;

       calculated_power = pid_cal(&turn, direction_face, encoder_avg);

       final_power = power_limit(max_speed, calculated_power);

         pros::lcd::print(6, "final_power %f\n", final_power);
        turn_set(final_power);

        if (timer_turn == true){
          net_timer = pros::millis() + timeout;
        }

        if (fabs(error_p) < 0.0174533)//0.0349066
        {   //if less than 2 degrees // ticks_to_deg = 3.18627451      (2*3.18627451) * (pi/180)
          timer_turn = false;
        }

          pros::delay(20); //20

      }
        while((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis()));
       turn_set(0);		//set drive to 0 power
}





// pid_init(&turn, 80, 0, 0.5, 50, 0.383972);
//
// float final_power = pid_cal(&turn, target, encoder_avg);


 void position_drive2(float target_x, float target_y, int timeout)
 {
      vector error;
      pid_terms turn;
      pid_terms drive;

     float final_power_t;

     float final_power_d;

     float encoder_avg;

     int max_speed = 100; //90

     bool timer_drive = true;
     unsigned int net_timer;

     int failsafe = 2000;    //varible value
     int initial_millis = pros::millis();
     int direction_face;

     float magnitude;
     //net_timer = pros::millis() + timeout; //just to initialize net_timer at first

     float encoder_average;
    float error_t;
    float error_d;


   pid_init(&turn, 6, 0, 0, 50, 0.383972);
   pid_init(&drive, 4, 0, 0, 50, 0.383972);

         while((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis()))
         {
           //encoder_average = position.y;
           error.x = target_x - position.x;		//wheel size is 4.17 inches
           error.y = target_y - position.y;

           direction_face = atan2f(error.x , error.y);

           encoder_avg = orientation;
           error_t = direction_face - encoder_avg;


          final_power_t = pid_cal(&turn, direction_face, encoder_avg);

          final_power_t = power_limit(max_speed, final_power_t);

           pros::lcd::print(0, "final_power_t %f\n", final_power_t);
           pros::lcd::print(4, "error_t %f\n", error_t);



//----------------------------------------------DRIVE_PART OF THE LOOP--------------------------------------

           error.x = target_x - position.x;		//wheel size is 4.17 inches
           error.y = target_y - position.y;

           magnitude = sqrt(powf(error.x,2) + powf(error.y,2));

           error_d = magnitude;
           final_power_d = pid_cal(&drive, magnitude*2, magnitude);

           final_power_d = power_limit(max_speed, final_power_d);

           pros::lcd::print(1, "final_power_d %f\n", final_power_d);
           pros::lcd::print(2, "error_d %f\n", error_d);


           right_drive_set(final_power_d + final_power_t);
           left_drive_set(final_power_d - final_power_t);



            if (fabs(error_d) < 1){	//less than 1 inches

              break;
        		}

        		else if (timer_drive)
        		{
        			net_timer = pros::millis() + timeout;
        		}


              pros::delay(20); //20



     }
        drive_set(0);		//set drive to 0 power
 }
