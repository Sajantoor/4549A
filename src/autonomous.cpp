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
  //pros::lcd::initialize();
  full_position_reset();
  position_drive(0, 0, 0, 50, 0, 100, 1.2, 0);
  loader_left.move(127);
  loader_right.move(127);
  position_drive(0, 50, 0, 0, 0, -100, 1.2, 0);
  position_turn2(degToRad(90), cw, 0.2,30,2.5);
  position_drive(0, 0, 30, 0, 0, 100, 1.2, 0);
  angler_pid(1250);
  pros::delay(2000);
  angler_pid(2050);
  position_drive(30, 0, 0, 0, 0, -100, 1.2, 0);
  // //pros::delay(5000);
  // position_drive(0, 21, 0, 21, 50, 100, 1, 0);
  //position_turn2(degToRad(-90), ccw, 0.2,30,2.5);
  // position_drive(0, 25, 0, 0, 50, -100, 1, 0);
  // position_drive(0, 0, 20, 20, 0, 100, 1, 0);

  if (switcher == 11) {


    //position_drive(0,0,0,40,250);
    //position_turn(90,500,78);
    // pros::delay(2000);
    //position_turn2(0.5*pi, cw, 0.17, 35, 7.3);
    //position_face_point(35,10, 300);
    //position_drive(0,0,0,40,300);
    //position_face_point2(71, 20, cw, 0, 35, 0, 30);
    //pros::delay(5000);
    //lift_target = 200;
    // position_turn2(-0.5*pi, ccw, 0.17, 35, 7.3);
    // position_turn2(0, cw, 0.17, 35, 7.3);
    // pros::delay(5000);
    // position_drive(0, 20, 0, 0, 0, -100, 1, 100);

    //position_drive(0, 0, 0, 20, 0, 100, 1, 100);

    //position_drive(position.x, position.y, 0, 0, 0, 100, 1, 100);


    //position_turn2(degToRad(20), cw, 0.005, 35, 8);

    //position_drive(0, 0, 20, 20, 0, 100, 1, 100);
    //position_turn(90,100);
    //position_face_point(10,40,100);
    // pros::delay(10000);
    // position_drive(0,-20,0,0,100,90,0.05,100);
    //position_drive(0,25,0,-10,-100,0.05,300);
  }
}
