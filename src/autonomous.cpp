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
  //drive_pid_encoder(20, 5000, 60);

reset_drive_encoders();
  // unlock mechanism

  // drive_pid_encoder(30, 1000, 127);
  // position_turn2(degToRad(90), cw, 0.17, 35, 6);
  // position_turn2(degToRad(0), ccw, 0.17, 35, 6);
  // drive_pid_encoder(-30, 1000, 127);


  //position_turn(45, 100);
  //position_turn2(degToRad(90), cw, 0.17, 30, 2.5);
  // position_drive(120, 10, 120, 40, 0, 100, 0.5, 0);
  // position_drive(120, 40, 120, 10, -50, -100, 0.5, 0);


//RED two ZONE STACK
  if (switcher == 1) {
    angler_pid(2720, 0);
    angler_pid(1640, 0);
    angler_pid(2720, 0);
    loader_left.move(-127);
    loader_right.move(-127);
    pros::delay(1500);
    loader_left.move(0);
    loader_right.move(0);
    angler_pid(1640, 0);

    loader_left.move(127);
    loader_right.move(127);
    drive_pid_encoder(17, 1000, 80);
    pros::delay(50);
    reset_position_full(23, 46, 0);
    position_turn2(degToRad(90), cw, 0.2, 35, 6);
    //position_turn(45,100);
    loader_left.move(-127);
    loader_right.move(-127);
    pros::delay(500);
    //position_turn2(degToRad(0), ccw, 0.17, 35, 6);
    reset_position_full(46, 35, 0.5*pi);
    position_turn(5,100);
    loader_left.move(75);
    loader_right.move(75);
    drive_pid_encoder(11, 1000, 60);
    drive_pid_encoder(15, 1000, 50);
    drive_pid_encoder(-35, 1000, 112);
    drive_line_up(-50, 400);
    drive_line_up(50, 230);
    reset_position_full(23, 12, 0);
    position_turn2(degToRad(-90), ccw, 0.17, 35, 6);
    drive_pid_encoder(25, 2000, 80);
    loader_left.move(-80);
    loader_right.move(-80);
    pros::delay(325);
    loader_left.move(0);
    loader_right.move(0);
    angler_pid(2790, 0);
    pros::delay(2000);
    angler_pid(1640, 0);
    pros::delay(50);
    loader_left.move(-127);
    loader_right.move(-127);
    drive_pid_encoder(-10, 1000, 80);
    // reset_position_full(46, 10, 0);
    // beginning_orientation = 0;
    // pros::delay(600);
    // loader_left.move(127);
    // loader_right.move(127);
    // drive_pid_encoder(50, 1000, 80);
    // drive_pid_encoder(-50, 1000, 90);
    // drive_line_up(-50, 400);
    // drive_line_up(50, 500);
    // position_turn2(degToRad(90), cw, 0.17, 35, 6);
}

if (switcher == 2) {
  angler_pid(2720, 0);
  angler_pid(1640, 0);
  angler_pid(2720, 0);
  loader_left.move(-127);
  loader_right.move(-127);
  pros::delay(1500);
  loader_left.move(0);
  loader_right.move(0);
  angler_pid(1640, 0);
  // drive_pid_encoder(10, 1000, 80);
  // pros::delay(500);
  // drive_pid_encoder(-10, 1000, 80);
}

if(switcher == 5){
  angler_pid(2720, 0);
  angler_pid(1640, 0);
  angler_pid(2720, 0);
  loader_left.move(-127);
  loader_right.move(-127);
  pros::delay(1500);
  loader_left.move(0);
  loader_right.move(0);
  angler_pid(1640, 0);

  loader_left.move(127);
  loader_right.move(127);
  drive_pid_encoder(17, 1000, 80);
  pros::delay(50);
  reset_position_full(23, 46, 0);
  position_turn2(degToRad(-90), ccw, 0.11, 30, 6);
  //position_turn(45,100);
  loader_left.move(-127);
  loader_right.move(-127);
  pros::delay(500);
  //position_turn2(degToRad(0), ccw, 0.17, 35, 6);
  reset_position_full(46, 35, -0.5*pi);
  position_turn(-5,100);
  loader_left.move(75);
  loader_right.move(75);
  drive_pid_encoder(11, 1000, 60);
  drive_pid_encoder(15, 1000, 50);
  drive_pid_encoder(-35, 1000, 85);
  drive_line_up(-50, 400);
  drive_line_up(50, 230);
  reset_position_full(23, 12, 0);
  position_turn2(degToRad(90), cw, 0.17, 35, 6);
  drive_pid_encoder(1, 1000, 80);
  loader_left.move(-80);
  loader_right.move(-80);
  pros::delay(325);
  loader_left.move(0);
  loader_right.move(0);
  angler_pid(2790, 0);
  pros::delay(2000);
  angler_pid(1640, 0);
  pros::delay(50);
  loader_left.move(-127);
  loader_right.move(-127);
  drive_pid_encoder(-10, 1000, 80);
  // reset_position_full(46, 10, 0);
  // beginning_orientation = 0;
  // pros::delay(600);
  // loader_left.move(127);
  // loader_right.move(127);
  // drive_pid_encoder(50, 1000, 80);
  // drive_pid_encoder(-50, 1000, 90);
  // drive_line_up(-50, 400);
  // drive_line_up(50, 500);
  // position_turn2(degToRad(90), cw, 0.17, 35, 6);
}

//SKILLZ
  if (switcher == 10) {
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
