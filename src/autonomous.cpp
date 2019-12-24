#include "main.h"
#include "drive.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "all_used.h"
#include "lcd.h"
#include "lift.h"
#include "angler.h"

void autonomous() {
  printf("PositionX: %f || PositionY: %f || Orientation: %f \n \n", position.x, position.y, orientation);
  printf("position.x %f \n", position.x);
  printf("position.y %f \n", position.y);
  reset_position_full(0, 0, 0);
  beginning_orientation = 0;
  //position_turn(-90, 100, 100);
  //turn_pid_encoder_average(90, 100);
  //position_turn2(degToRad(-90), ccw, 0, 30, 3.7);
  // loader_left.move(127);
  // loader_right.move(127);

            // position_drive(0, 0, -30, -30, 0, -90, 1, 0);
            // turn_pid_encoder_average(-13, 100);
            // position_drive(-30, -30, -30, -10, 0, 90, 1, 0);
            // turn_pid_encoder_average(90, 1000);
            // turn_pid_encoder_average(0, 5000);
            // position_drive(-30, -10, 0, -10, 0, 90, 1, 0);
            // turn_pid_encoder_average(0, 100);

      // position_drive2(0, 20, 0, 100,6000);
      // position_drive2(0, 0, 0, 100,6000);
    //  // // position_turn(90, 500, 100);
    //  // // position_turn(0, 500, 100);
    //  pros::delay(5000);
    // position_drive2(0, 0, 0, -100);

  // turn_pid_encoder_average(90, 100);
  // turn_pid_encoder_average(0, 100);
  // position_drive(0, 30, 0, 0, 0, -100, 0.5, 0);
  // loader_left.move(0);
  // loader_right.move(0);
  // printf("position.x %f \n", position.x);
  // printf("position.y %f \n", position.y);
  // pros::delay(6000);
  // // reset_position_full(120, 40, 0);
  //position_drive(120, 40, 120, 10, -50, -70, 1, 0);

  // angler_pid(1580, 0);
  // pros::delay(2000);
  // loader_left.move(-127);
  // loader_right.move(-127);
  // pros::delay(1500);
  // loader_left.move(0);
  // loader_right.move(0);
  // angler_pid(3665, 0, 80, false);
  // pros::delay(1000);

  position_drive2(0,20,0,100,4000);
  position_drive2(-20,20,0,90,4000);
  position_drive2(-20,0,0,100,4000);
  position_drive2(0,0,0,90,4000);
  position_drive2(0,20,0,100,4000);
  position_drive2(-20,20,0,90,4000);
  position_drive2(-20,0,0,100,4000);
  position_drive2(0,0,0,90,4000);

 // position_drive2(0, 50, 0, 63,3000);
 // loader_left.move(0);
 // loader_right.move(0);
 // position_drive2(0, 5, 0, 63,3000);
 // position_turn(90, 500, 100);
 // reset_position_full(0, 5, 0);
 // position_drive2(20, 5, 0, 100,1000);
 // position_drive2(20, 50, 0, 100,3000);
 // loader_left.move(-63);
 // loader_right.move(-63);
 // pros::delay(1000);
 // angler_pid(500, 20000);
 // loader_left.move(0);
 // loader_right.move(0);
 // angler_pid(3665, 0, 80, false);
 // pros::delay(6000);
 // drive_line_up(-100,500);
  if(switcher == 1){
    angler_pid(1580, 0);
    pros::delay(2000);
    loader_left.move(-127);
    loader_right.move(-127);
    pros::delay(1500);
    loader_left.move(0);
    loader_right.move(0);
    angler_pid(3665, 0, 80, false);

    position_drive2(16, 0, 0, 127, 1500);
    position_drive2(0, 0, 0, 127, 1200);
  }              //

  if(switcher == 2){

    position_drive2(-16, 0, 0, 127, 1500);
    position_drive2(0, 0, 0, 127, 1200);
    angler_pid(1580, 0);
    pros::delay(2000);
    loader_left.move(-127);
    loader_right.move(-127);
    pros::delay(1500);
    loader_left.move(0);
    loader_right.move(0);
    angler_pid(3665, 0, 80, false);

  }

  if(switcher == 3){
    angler_pid(1580, 0);
    pros::delay(2000);
    loader_left.move(-127);
    loader_right.move(-127);
    pros::delay(1500);
    loader_left.move(0);
    loader_right.move(0);
    angler_pid(3665, 0, 80, false);

    loader_left.move(127);
    loader_right.move(127);
    position_drive2(0, 55, 0, 70,3000);
    loader_left.move(0);
    loader_right.move(0);
    position_drive2(0, 5, 0, 110,2500);
    position_turn(90,2000,100);
    reset_position_full(0, 5, 0);
    position_drive2(30, 5, 0, 100,1500);
    reset_position_full(0, 0, 0);
    position_drive2(0, 18, 0, 100, 1500);
    angler_pid(900, 20000);
    pros::delay(1800);
    loader_left.move(-90);
    loader_right.move(-90);
    pros::delay(1800);
    drive_line_up(-100, 1000);
  }

  if(switcher == 4){
    angler_pid(1580, 0);
    pros::delay(2000);
    loader_left.move(-127);
    loader_right.move(-127);
    pros::delay(1500);
    loader_left.move(0);
    loader_right.move(0);
    angler_pid(3665, 0, 80, false);

    reset_position_full(0, 0, 0);
    beginning_orientation = 0;

    loader_left.move(127);
    loader_right.move(127);
    position_drive2(0, 40, 0, 40,3000);
    loader_left.move(0);
    loader_right.move(0);
    position_drive2(0, 5, 0, 127,3000);
    position_turn(-90,1500,100);
    reset_position_full(0, 5, 0);
    position_drive2(-30, 5, 0, 100,1500);
    reset_position_full(0, 0, 0);
    position_drive2(0, 12, 0, 80, 1500);
    angler_pid(900, 20000);
    pros::delay(1200);
    loader_left.move(-90);
    loader_right.move(-90);
    pros::delay(1700);
    drive_line_up(-90, 1000);
  }
  printf("orientation %f \n", orientation);
  printf("position.x %f \n", position.x);
  printf("position.y %f \n", position.y);
}
