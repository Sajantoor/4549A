#include "main.h"
#include "drive.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "all_used.h"
#include "lcd.h"
#include "lift.h"
#include "angler.h"
#include "intake.h"

float timerAuto;

void autonomous() {
  printf("PositionX: %f || PositionY: %f || Orientation: %f \n \n", position.x, position.y, orientation);
  printf("position.x %f \n", position.x);
  printf("position.y %f \n", position.y);
  reset_position_full(0, 0, 0);
  beginning_orientation = 0;
  float initial_time;
  gyro.reset();

  initial_time = pros::millis();
  autoIntakeFunc(127);
  position_drive2(0, 0, 0, 30, 0, 60, 0.1, 0, 3000, 50,40, true);// pick up cubes
  position_drive2(0, 30, 0, 0, 0, -60, 0.1, 0, 3000, 50,40);// pick up cubes

  // FRONT AUTO STARTING
  // loader_right.move(127);
  // loader_left.move(127);
  // position_drive2(0, 0, 0, 13, 0, 90, 0.1, 0, 3000, 50,40);
  // pros::delay(1000);
  // loader_right.move(-90);
  // loader_left.move(-90);
  // pros::delay(500);
  // loader_right.move(0);
  // loader_left.move(0);
  // lift(2500, 20000);
  // position_drive2(0, 13, 0, 30, 0, 45, 0.1, 0, 3000, 50,40);// pick up cubes
  // loader_left.move(-127);
  // loader_right.move(-40);
  // pros::delay(800);
  // position_drive2(0, 30, 0, 27, 0, 45, 0.1, 0, 3000, 50,40);
  // lift(2100, 20000);
  // loader_right.move(127);
  // loader_left.move(127);
  // position_drive2(0, 27, 0, 30, 0, 60, 0.1, 0, 3000, 50,40);// pick up cubes
  // // position_drive2(0, 10, 0, 32, 0, 60, 0.1, 0, 3000, 50,40);// pick up cubes
  // lift(1000, 2000);
  // pros::delay(500);
  // lift(500, 2000);
  // pros::delay(500);
  // lift(0, 2000);
  // pros::delay(500);
//RED FRONT AUTO
  if(switcher == 1){
    initial_time = pros::millis();
    lift(1950, 20000);
    pros::delay(1000);
    lift(0, 0);
    pros::delay(1000);
    position_drive(0,15,0,false,127,1100, 0, 127, 10);//pick up first set of cubes
    position_turn(90,400,100);
    position_drive(8,15,90,false,70,1300,80,80,5);//pick up first set of cubes
    position_turn(35,600,100);
    // angler_pid(1100, 20000);
    position_drive(28,38,35,false,80,2500, 0, 100, 20);//pick up first set of cubes
    // angler_pid(870, 500, 127, false);
    position_drive(0,40,-90,false,100,3500, 0, 100, 25);//pick up first set of cubes
    //angler_pid(1000, 3000);
    angler_pid(2500, true, 100, false, 2000);
    pros::delay(1000);
    position_drive(-22,3,-150,false,127,2500, 0, 100, 26);//pick up first set of cubes
    loader_left.move(0);
    loader_right.move(0);
    angler_pid(1189, true, 127, true);
    pros::delay(2300);
    position_drive(-16,14,-150,false,127,3800);//pick up first set of cube
    timerAuto = pros::millis() - initial_time;
    controller.print(0, 0, "Timer Auto: %f", timerAuto);
  }

//RED BACK AUTO
  if(switcher == 2){
    initial_time = pros::millis();
    pros::delay(2000);
    lift(1950, 20000);
    pros::delay(1500);
    lift(0, 0);
    pros::delay(1000);
    loader_left.move(127);
    loader_right.move(127);
    position_drive2(0, 0, 0, 40, 0, 90, 0.1, 0, 2300, 50,40);// pick up cubes
    position_drive2(0, 40, -24, 4, 0, -90, 0.1, 0, 2300, 50,40);// pick up cubes
    position_turn(0, 3000, 127);
    position_drive2(-24, 4, -23, 38, 0, 90, 0.1, 0, 2300, 50,40);// pick up cubes
    position_turn(130, 1500, 110);
    position_drive2(-25, 38, 5, 6, 0, 90, 0.1, 0, 3000, 50,40);// pick up cubes
    loader_left.move(0);
    loader_right.move(0);
    angler_pid(-4500, true, 127, true);
    drive_line_up(-100,1000);
  //   position_drive2(0, 0, 0, 23, 0, 90, 0.1, 0, 2300, 50,40);// pick up cubes
  //   position_drive2(0, 23, 25, 9, 0, -110, 1, 0, 2000, 50,40);// pick up cubes
  //   position_turn(5, 1000, 90);
  //   position_drive2(25, 9, 25, 38, 0, 90, 0.5, 0, 2600, 50,20);// pick up cubes
  //   position_drive2(23, 42, 26.5, 23, 0, -110, 0.05, 0, 1800, 50,40);// pick up cubes
  //   position_turn(129, 1500, 110);
    // loader_left.move(0);
    // loader_right.move(0);
    //   angler_pid(2200, true, 127, false, 0, true);
  //   position_drive2(26.5, 23, 40, 5, 0, 110, 0.05, 0, 950, 50,40);// pick up cubes
  //   angler_pid(1020, true, 127, true);
  //   pros::delay(2000);
  // angler_pid(3400, true, 127, false, 2000);
  // drive_line_up(-100, 500);
  }

//1 POINT UNLOCK AUTO RED RIGHT
  if(switcher == 3){
    position_drive(16, 0, 0,false, 127, 1500);
    position_drive(0, 0, 0,false, 127, 1200);

    lift(1950, 20000);
    loader_left.move(-127);
    loader_right.move(-127);
    pros::delay(1500);
    lift(0, 0);
    pros::delay(1000);
  }

  //1 POINT UNLOCK AUTO RED LEFT
  if(switcher == 4){
    position_drive2(0, 0, 0, 15, 0, 110, 0.1, 0, 2300, 50,40);// pick up cubes
    position_drive2(0, 15, 0, 0, 0, -110  , 0.1, 0, 2300, 50,40);// pick up cubes

    lift(1950, 20000);
    loader_left.move(-127);
    loader_right.move(-127);
    pros::delay(1500);
    lift(0, 0);
    pros::delay(1000);
  }

  //BLUE FRONT AUTO
  if(switcher == 5){
    initial_time = pros::millis();
    lift(1950, 20000);
    pros::delay(1000);
    lift(0, 0);
    pros::delay(600);
    position_drive(0,15,0,false,127,1100, 0, 127, 10);//pick up first set of cubes
    position_turn(-45,400,100);
    // position_drive(-7,15,-90,false,60,1300,80,80,5);//pick up first set of cubes
    // position_turn(-40,600,100);
    // angler_pid(1100, 20000);
    position_drive(-30,40,-45,false,80,2500, 0, 100, 20);//pick up first set of cubes
    // angler_pid(870, 500, 127, false);
    position_drive(0,40,90,false,100,3000, 0, 110, 25);//pick up first set of cubes
    //angler_pid(1000, 3000);
    angler_pid(2500, true, 100, false, 2000);
    pros::delay(1000);
    position_drive(21,0,150,false,127,3000, 0, 100, 26);//pick up first set of cubes
    loader_left.move(0);
    loader_right.move(0);
    angler_pid(1189, true, 127, true);
    pros::delay(2300);
    position_drive(16,14,-150,false,127,3800);//pick up first set of cube
    timerAuto = pros::millis() - initial_time;
    controller.print(0, 0, "Timer Auto: %f", timerAuto);
  }

  //BLUE BACK AUTO
    if(switcher == 6){
      initial_time = pros::millis();
      lift(1950, 20000);
      loader_left.move(-127);
      loader_right.move(-127);
      pros::delay(1500);
      lift(0, 0);
      pros::delay(1000);
      loader_left.move(127);
      loader_right.move(127);
      position_drive2(0, 0, 0, 23, 0, 90, 0.1, 0, 2300, 50,40);// pick up cubes
      position_drive2(0, 18, -25, 9, 0, -110, 1, 0, 2000, 50,40);// pick up cubes
      position_turn(0, 1000, 9);
      position_drive2(-25, 9, -25, 38, 0, 100, 0.5, 0, 2600, 50,20);// pick up cubes
      position_drive2(-23, 42, -26.5, 23, 0, -110, 0.05, 0, 1800, 50,40);// pick up cubes
      position_turn(-129, 1500, 110);
      loader_left.move(0);
      loader_right.move(0);
        angler_pid(2200, true, 127, false, 0, true);
      position_drive2(-26.5, 23, -40, 5, 0, 110, 0.05, 0, 950, 50,40);// pick up cubes
      angler_pid(1020, true, 127, true);
      pros::delay(2000);
    angler_pid(3400, true, 127, false, 2000);
    drive_line_up(-100, 500);
    }

  //1 POINT UNLOCK AUTO BLUE RIGHT
    if(switcher == 7){
      position_drive(16, 0, 0,false, 127, 1500);
      position_drive(0, 0, 0, false,127, 1200);

      angler_pid(1580, 0);
      pros::delay(2000);
      loader_left.move(-127);
      loader_right.move(-127);
      pros::delay(1500);
      loader_left.move(0);
      loader_right.move(0);
      angler_pid(3665, 0, 80, false);
    }

    //1 POINT UNLOCK AUTO BLUE LEFT
    if(switcher == 8){
      position_drive(-16, 0, 0,false,127, 1500);
      position_drive(0, 0, 0, false,127, 1200);

      angler_pid(1580, 0);
      pros::delay(2000);
      loader_left.move(-127);
      loader_right.move(-127);
      pros::delay(1500);
      loader_left.move(0);
      loader_right.move(0);
      angler_pid(3665, 0, 80, false);
    }


    //SKILLS
    if(switcher == 9) {
      loader_left.move(127);
      loader_right.move(127);
      position_drive2(0, 0, 0, 113, 0, 80, 0.05, 0, 11000, 50,40);// pick up cubes
      pros::delay(1500);
      position_turn(30, 1500, 127);
      position_drive2(0, 112, 10, 122, 127, 100, 0.5, 0, 3500, 2, 127);// pick up cubes
      loader_left.move(0);
      loader_right.move(0);
      angler_pid(1020, true, 127, true);
      pros::delay(6000);
      position_drive2(position.x, position.y, -5, 100, 0, -60, 0.5, 0, 15000, 5, 100);// pick up cubes
      drive_line_up(-80, 800);
      position_turn(0, 1500, 127);
      angler_pid(3530, true, 100, false, 2000);
      pros::delay(3000);
      lift(1905, 20000);
      pros::delay(3000);
      drive_line_up(80, 1100);
      reset_position_full(1, 120, 0);
    }
  printf("orientation %f \n", radToDeg(orientation));
  printf("position.x %f \n", position.x);
  printf("position.y %f \n", position.y);
}
