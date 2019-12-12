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

     position_drive2(-20, 0, 0, 100);
     // position_turn(90, 500, 100);
     // position_turn(0, 500, 100);
      position_drive2(0, 0, 0, 100);

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
  printf("orientation %f \n", orientation);
  printf("position.x %f \n", position.x);
  printf("position.y %f \n", position.y);

//RED two ZONE STACK
  if(switcher == 1){
    // angler_pid(2720, 0);
    // angler_pid(1640, 0);
    // angler_pid(2720, 0);
    // loader_left.move(-127);
    // loader_right.move(-127);
    // pros::delay(1500);
    // loader_left.move(0);
    // loader_right.move(0);
    // angler_pid(1640, 0);
    angler_pid(2000, 0);
    angler_pid(1580, 0);
    angler_pid(2000, 0);
    loader_left.move(-127);
    loader_right.move(-127);
    pros::delay(1500);
    loader_left.move(0);
    loader_right.move(0);
    angler_pid(1580, 0);

    reset_position_full(44, 10, 0);
    beginning_orientation = 0;
    loader_left.move(127);
    loader_right.move(127);
    position_drive(44, 10, 44, 60, 0, 127, 1, 0, 2000, 15);// pick up cubes
    position_drive(44, 50, 44, 30, 0, -100, 1, 0, 2000, 15);// pick up cubes
    turn_pid_encoder_average(-90,400);
    position_drive(44, 30, 30, 30, 0, 127, 1, 0, 2000, 15);// pick up cubes
    turn_pid_encoder_average(-140,1000);
    position_drive(30, 30, 17, 21, 0, 127, 1, 0, 2000, 15);// pick up cubes
    loader_left.move(0);
    loader_right.move(0);
    angler_pid(2530, 2000);
    angler_pid(1580, 0);
    pros::delay(1400);
    //position_drive(118, 20, 126, 16, 0, 100, 1, 10);// pick up cubes
}

  if(switcher == 2){
    // angler_pid(2000, 0);
    // angler_pid(1580, 0);
    // angler_pid(2000, 0);
    // loader_left.move(-127);
    // loader_right.move(-127);
    // pros::delay(1500);
    // loader_left.move(0);
    // loader_right.move(0);
    // angler_pid(1580, 0);

    reset_position_full(95, 10, 0);
    beginning_orientation = 0;
    loader_left.move(127);
    loader_right.move(127);
    position_drive(95, 10, 95, 45, 0, 127, 1, 0, 2000, 15);// pick up cubes
    //position_drive(95, 45, 120, 10, 0, -110, 0.5, 0, 3000, 30);// CURVE TURN
//    position_drive2(95, 45, 125, 10, 0, -127, 0.5, 0, 2350, 30);// CURVE TURN
    position_turn(12, 200, 127);
    position_drive(118, 10, 118, 40, 0, 127, 1, 0, 2000, 15);// pick up cubes
    turn_pid_encoder_average(150,1000);
    loader_left.move(75);
    loader_right.move(75);
    position_drive(118, 45, 144, 0, 0, 100, 1, 0, 2200, 15);// pick up cubes
    loader_left.move(-50);
    loader_right.move(-50);
    pros::delay(600);
    angler_pid(2580, 2500);
    pros::delay(1200);
    angler_pid(1580, 0);
    loader_left.move(0);
    loader_right.move(0);
    position_drive(144, 0, 110, 50, 0, -100, 1, 0, 2000, 15);// pick up cubes
  }

if(switcher == 5){

}

//SKILLZ
  if(switcher == 10){
    loader_left.move(127);
    loader_right.move(127);
    position_drive(120, 10, 120, 60, 0, 100, 1.5, 0, 100, 15);// pick up cubes
    position_drive(120, 60, 120, 12, -50, -100, 1.5, 0, 100, 15);
    position_turn(100, 100, 100);
    position_drive(120, 12, 122, 12, 0, 110, 1.5, 0, 100, 15); //drive to red one zone
    loader_left.move(0);
    loader_right.move(0);
    loader_left.move(-80);
    loader_right.move(-80);
    pros::delay(225);
    loader_left.move(80);
    loader_right.move(80);
    pros::delay(1120);
    loader_left.move(0);
    loader_right.move(0);
    pros::delay(200);
    angler_pid(2570, 0);
    pros::delay(2000);
    angler_pid(1780, 0);
    loader_left.move(-50);
    loader_right.move(-50);
    position_drive(130, 10, 120, 10, 0, -100, 1.8, 0, 100, 15);
    position_turn(0, 100, 100);
    //first stack done in red one zone
    loader_left.move(127);
    loader_right.move(127);
    position_drive(120, 10, 120, 135, 0, 100, 1.8, 0, 100, 15);// pick up second stack in blue zone
    position_turn(90, 100,100);
    position_drive(120, 135, 140, 132, 0, 100, 1.8, 0, 100, 15);// drive to blue one zone
    loader_left.move(0);
    loader_right.move(0);
    loader_left.move(-80);
    loader_right.move(-80);
    pros::delay(225);
    loader_left.move(80);
    loader_right.move(80);
    pros::delay(1120);
    loader_left.move(0);
    loader_right.move(0);
    pros::delay(200);
    angler_pid(2570, 0);
    pros::delay(2000);
    angler_pid(1780, 0);
    loader_left.move(-50);
    loader_right.move(-50);
    position_drive(140, 132, 120, 135, 0, -100, 1.8, 0, 100, 15);// stack done for second blue one zone stack
    position_turn(-90, 100, 100);
    drive_line_up(-90, 3000);
    reset_position_full(120, 135, 0);// line up with wall
  }

//TESTING
  if (switcher == 11) {
    reset_position_full(10, 24, 0);
    beginning_orientation = 0;
    printf("PositionX: %f || PositionY: %f || Orientation: %f \n \n", position.x, position.y, orientation);
    printf("position.x %f \n", position.x);
    printf("position.y %f \n", position.y);
    printf("orientation %f \n", radToDeg(orientation));

    //position_turn2(degToRad(90), cw, 0.15, 30, 2.5);
    //position_turn(90, 100);
    //pros::delay(5000);
    // position_drive(120, 40, 120, 10, -50, -100, 1.5, 0);

    /*
    position_drive(10, 24, 10, 50, 0, 100, 0.5, 0);
    position_drive(10, 50, 10, 10, -80, -100, 0.5, 5);
    */
  }
}
