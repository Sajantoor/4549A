#include "main.h"
#include "drive.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "all_used.h"
#include "lcd.h"
#include "lift.h"
#include "intake.h"
#include "angler.h"

void autonomous() {
  printf("PositionX: %f || PositionY: %f || Orientation: %f \n \n", position.x, position.y, orientation);
  printf("position.x %f \n", position.x);
  printf("position.y %f \n", position.y);
  reset_position_full(120, 10, 0);
  //position_turn(90, 100);
  //position_turn2(degToRad(90), cw, 0.17, 30, 2.5);
  // position_drive(120, 10, 120, 40, 0, 100, 0.5, 0);
  // printf("position.x %f \n", position.x);
  // printf("position.y %f \n", position.y);
  // pros::delay(6000);
  // // reset_position_full(120, 40, 0);
  // position_drive(120, 40, 120, 10, -50, -70, 0.5, 0);
  // printf("position.x %f \n", position.x);
  // printf("position.y %f \n", position.y);

//RED two ZONE STACK
  if(switcher == 1){
    reset_position_full(46, 10, 0);
    beginning_orientation = 0;
    angler_pid(2720, 0);
    pros::delay(1500);
    loader_left.move(-127);
    loader_right.move(-127);
    pros::delay(1500);
    loader_left.move(0);
    loader_right.move(0);
    angler_pid(1640, 0);
    loader_left.move(100);
    loader_right.move(100);
    position_drive(46, 10, 46, 60, 0, 100, 1, 5);// pick up cubes
    position_drive(46, 10, 46, 10, 0, 100, 1, 0);// pick up cubes
    drive_line_up(-50, 600);
    drive_line_up(50, 500);
    position_turn2(degToRad(-90), ccw, 0.17, 30, 2.5);
    position_drive(46, 12, 26, 12, 0, 100, 0.5, 5);
    loader_left.move(0);
    loader_right.move(0);
    loader_left.move(-80);
    loader_right.move(-80);
    pros::delay(225);
    loader_left.move(80);
    loader_right.move(80);
    pros::delay(500);
    loader_left.move(0);
    loader_right.move(0);
    pros::delay(200);
    angler_pid(2570, 0);
    pros::delay(2000);
    angler_pid(1640, 0);
    loader_left.move(-50);
    loader_right.move(-50);
}

//SKILLZ
  if(switcher == 10){
    loader_left.move(127);
    loader_right.move(127);
    position_drive(120, 10, 120, 60, 0, 100, 1.5, 0);// pick up cubes
    position_drive(120, 60, 120, 12, -50, -100, 1.5, 0);
    position_turn(100, 100);
    position_drive(120, 12, 122, 12, 0, 110, 1.5, 0); //drive to red one zone
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
    position_drive(130, 10, 120, 10, 0, -100, 1.8, 0);
    position_turn(0, 100);
    //first stack done in red one zone
    loader_left.move(127);
    loader_right.move(127);
    position_drive(120, 10, 120, 135, 0, 100, 1.8, 0);// pick up second stack in blue zone
    position_turn(90, 100);
    position_drive(120, 135, 140, 132, 0, 100, 1.8, 0);// drive to blue one zone
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
    position_drive(140, 132, 120, 135, 0, -100, 1.8, 0);// stack done for second blue one zone stack
    position_turn(-90, 100);
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
