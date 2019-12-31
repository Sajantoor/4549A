#include "main.h"
#include "drive.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "all_used.h"
#include "lcd.h"
#include "lift.h"
#include "angler.h"

void autonomous() {
  // reset_position_full(0, 0, 0);
  basicMovement(20, 0, 0);
  basicMovement(0, 0, 0);
//   printf("PositionX: %f || PositionY: %f || Orientation: %f \n \n", position.x, position.y, orientation);
//   printf("position.x %f \n", position.x);
//   printf("position.y %f \n", position.y);

//   beginning_orientation = 0;
//
//   position_drive2(0,20,0,100,3000);
//   position_turn(90, 1000, 100);
//   position_drive2(10,20,90,100,3000);
//   // reset_position_full(0, 20, 0);
//   //position_drive2(0,0,90,100,3000);
//   // position_drive2(20,40,0,100,2000);
//   // position_drive2(20,20,0,100,2000);
//   // position_turn(-90, 1000, 100);
//
// //mini auto
//    // position_drive2(0,20,0,100,2000);
//    // position_drive2(20,0,0,100,2000);
//    // position_drive2(20,20,0,100,2000);
//    // position_drive2(20,0,0,100,2000);
//    // position_turn(90, 1000, 100);
//    // reset_position_full(20, 0, 0);
//    // position_drive2(20,5,0,100,2000);
//
//   // position_drive2(0,0,0,70,2000);
//   // position_drive2(0,20,0,100,2000);
//   // position_drive2(20,20,0,100,2000);
//   // position_drive2(20,0,0,100,2000);
//   // position_drive2(0,0,0,70,2000);
//
//   if(switcher == 1){
//     angler_pid(1580, 0);
//     pros::delay(2000);
//     loader_left.move(-127);
//     loader_right.move(-127);
//     pros::delay(1500);
//     loader_left.move(0);
//     loader_right.move(0);
//     angler_pid(3665, 0, 80, false);
//
//     position_drive2(16, 0, 0, 127, 1500);
//     position_drive2(0, 0, 0, 127, 1200);
//   }
//
//   if(switcher == 2){
//
//     position_drive2(-16, 0, 0, 127, 1500);
//     position_drive2(0, 0, 0, 127, 1200);
//     angler_pid(1580, 0);
//     pros::delay(2000);
//     loader_left.move(-127);
//     loader_right.move(-127);
//     pros::delay(1500);
//     loader_left.move(0);
//     loader_right.move(0);
//     angler_pid(3665, 0, 80, false);
//
//   }
//
//   if(switcher == 3){
//     angler_pid(1580, 0);
//     pros::delay(2000);
//     loader_left.move(-127);
//     loader_right.move(-127);
//     pros::delay(1500);
//     loader_left.move(0);
//     loader_right.move(0);
//     angler_pid(3665, 0, 80, false);
//
//     loader_left.move(127);
//     loader_right.move(127);
//     position_drive2(0, 55, 0, 70,3000);
//     loader_left.move(0);
//     loader_right.move(0);
//     position_drive2(0, 5, 0, 110,2500);
//     position_turn(90,2000,100);
//     reset_position_full(0, 5, 0);
//     position_drive2(30, 5, 0, 100,1500);
//     reset_position_full(0, 0, 0);
//     position_drive2(0, 18, 0, 100, 1500);
//     angler_pid(900, 20000);
//     pros::delay(1800);
//     loader_left.move(-90);
//     loader_right.move(-90);
//     pros::delay(1800);
//     drive_line_up(-100, 1000);
//   }
//
//   if(switcher == 4){
//     angler_pid(1580, 0);
//     pros::delay(2000);
//     loader_left.move(-127);
//     loader_right.move(-127);
//     pros::delay(1500);
//     loader_left.move(0);
//     loader_right.move(0);
//     angler_pid(3665, 0, 80, false);
//
//     reset_position_full(0, 0, 0);
//     beginning_orientation = 0;
//
//     loader_left.move(127);
//     loader_right.move(127);
//     position_drive2(0, 40, 0, 40,3000);
//     loader_left.move(0);
//     loader_right.move(0);
//     position_drive2(0, 5, 0, 127,3000);
//     position_turn(-90,1500,100);
//     reset_position_full(0, 5, 0);
//     position_drive2(-30, 5, 0, 100,1500);
//     reset_position_full(0, 0, 0);
//     position_drive2(0, 12, 0, 80, 1500);
//     angler_pid(900, 20000);
//     pros::delay(1200);
//     loader_left.move(-90);
//     loader_right.move(-90);
//     pros::delay(1700);
//     drive_line_up(-90, 1000);
//   }
//   printf("orientation %f \n", radToDeg(orientation));
//   printf("position.x %f \n", position.x);
//   printf("position.y %f \n", position.y);
}
