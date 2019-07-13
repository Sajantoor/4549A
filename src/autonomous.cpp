#include "main.h"
#include "drive.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "stacker.h"
#include "ritam_drive.h"
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous()
{
  pros::lcd::initialize();
  beginning_orientation = 0;
  reset_drive_encoders();
  prev_inches_traveled_left = 0;
  prev_inches_traveled_right = 0;
  position = {0,0};

//math_test(0,0,0,70);
//position_drive(0,0,0,40,250);

   //drive_pid_encoder(10, 300);
  // turn_pid_encoder_average(90, 250);
  // turn_pid_encoder_average(-90, 250);
   //drive_pid_encoder(-5, 300);
  // turn_pid_encoder_average(90, 250);
  // turn_pid_encoder_average(-90, 250);
//position_turn(90,500,78);
// pros::delay(2000);
//position_turn(90,450, 80);
//position_face_point3(35,10, 300);
//position_drive(0,0,0,40,300);
position_drive(0,0,0,-40,300);
//
//pros::lcd::print(7, "position.x %f\n", position.x);//pros::lcd::print(6, "position.x %f\n", position.x);


//unsignedpros::lcd::print(6, "orientation %f", orientation);

}
