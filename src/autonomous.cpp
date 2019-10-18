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
  //
  if (switcher == 1){

  }

  //Red Back Auto 5 stack
  if (switcher == 2) {
    // lift(1210, 1000);
    // pros::delay(500);
    // lift(2600, 0);

    loader_left.move(127);
    loader_right.move(127);
    position_drive(0, 0, 0, 42, 0, 80, 1.2, 0, 8.2);
    position_drive(0, 40, 0, 4, 0, -90, 1.2, 0, 8.2);
    position_turn2(degToRad(93), cw, 0.2,30,2.5);
    position_drive(0, 0, 10, -5, 0, 90, 1.2, 0, 3);//10,-10
    loader_left.move(0);
    loader_right.move(0);
    loader_left.move(-80);
    loader_right.move(-80);
    pros::delay(500);
    loader_left.move(60);
    loader_right.move(60);
    pros::delay(1200);
    loader_left.move(0);
    loader_right.move(0);
    pros::delay(200);
    angler_pid(1265);
    pros::delay(2000);
    angler_pid(2050);
    loader_left.move(-50);
    loader_right.move(-50);
    position_drive(10, -5, 0, 0, 0, -90, 1.2, 0, 4);//10,-10

    //position_turn2(degToRad(90), cw, 0.2,30,2.5);
    // position_drive(0, 0, 30, 0, 0, 100, 1.2, 0);
    // angler_pid(1250);
    // pros::delay(2000);
    // angler_pid(2050);
    // position_drive(30, 0, 0, 0, 0, -100, 1.2, 0);


    // //pros::delay(5000);
    // position_drive(0, 21, 0, 21, 50, 100, 1, 0);
    //position_turn2(degToRad(-90), ccw, 0.2,30,2.5);
    // position_drive(0, 25, 0, 0, 50, -100, 1, 0);
    // position_drive(0, 0, 20, 20, 0, 100, 1, 0);
  }
}
