#include "main.h"
#include "drive.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "all_used.h"
#include "lcd.h"
#include "lift.h"
#include "angler.h"

float timerAuto;

void autonomous() {
  printf("PositionX: %f || PositionY: %f || Orientation: %f \n \n", position.x, position.y, orientation);
  printf("position.x %f \n", position.x);
  printf("position.y %f \n", position.y);
  reset_position_full(0, 0, 0);
  beginning_orientation = 0;
  float initial_time;

  if(switcher == 0){
    lift(1950, 20000);
    pros::delay(1000);
    lift(0, 0);
  }

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
            // initial_time = pros::millis();
            // lift(1950, 20000);
            // pros::delay(1000);
            // lift(0, 0);
            // loader_left.move(100);
            // loader_right.move(100);
            // position_drive(0,56,0,false,65,4000, 80, 127, 30);//pick up first set of cubes
            // position_drive(0,12,0,false,127,4500, 127, 100 , 25);//come back facing the scoring zone
            // loader_left.move(0);
            // loader_right.move(0);
            // position_turn(90, 600, 110);
            // position_drive(0,-20,90,false,127,1500, 0, 0, 0);//strafe to line up with wall
            // reset_position_full(0,0,0);
            // loader_left.move(-20);
            // loader_right.move(-20);
            // position_drive(0,9,0,false,127,1000, 0, 0, 0);//strafe to line up with wall
            // loader_left.move(0);
            // loader_right.move(0);
            // angler_pid(1189, true, 127, true);
            // pros::delay(1600);
            // position_drive(0,-5,0,false,127,1000, 0, 0, 0);//strafe to line up with wall
            // angler_pid(3000, true, 100, false, 2000);
            // timerAuto = pros::millis() - initial_time;
            // controller.print(0, 0, "Timer Auto: %d", timerAuto/1000);
            //
    //better autonomoua
    initial_time = pros::millis();
    lift(1950, 20000);
    pros::delay(1000);
    lift(0, 0);
    position_drive(0,44.4,0,false,100,4000, 127, 0, 30, 10, 80);//pick up first set of cubes
    position_drive(0,23,0,false,127,4000);//pick up first set of cubes
    position_drive(26.5,10,0,false,127,4000, 0, 0, 0, 15, 100);//pick up first set of cubes
    position_drive(26.5,57,0,false,110,4000, 0, 0, 0, 25, 70);//pick up first set of cubes
    position_drive(26.5,23,0,false,127,4000);//pick up first set of cubes
    position_turn(135, 1500, 110);
    position_drive(33,12.1,135,false,127,4000);//pick up first set of cubes
    loader_left.move(0);
    loader_right.move(0);
    angler_pid(1189, true, 127, true);
    pros::delay(1600);
    position_drive(26.5,23,0,false,127,1000);//strafe to line up with wall
    angler_pid(3000, true, 100, false, 2000);
    timerAuto = pros::millis() - initial_time;
  }

//1 POINT UNLOCK AUTO RED RIGHT
  if(switcher == 3){
    position_drive(16, 0, 0,false, 127, 1500);
    position_drive(0, 0, 0,false, 127, 1200);

    angler_pid(1580, 0);
    pros::delay(2000);
    loader_left.move(-127);
    loader_right.move(-127);
    pros::delay(1500);
    loader_left.move(0);
    loader_right.move(0);
    angler_pid(3665, 0, 80, false);
  }

  //1 POINT UNLOCK AUTO RED LEFT
  if(switcher == 4){
    position_drive(-16, 0, 0, false,127, 1500);
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
      pros::delay(1000);
      lift(0, 0);
      loader_left.move(100);
      loader_right.move(100);
      position_drive(0,56,0,false,65,4000, 80, 127, 30);//pick up first set of cubes
      position_drive(0,12,0,false,127,4500, 127, 100 , 25);//come back facing the scoring zone
      loader_left.move(0);
      loader_right.move(0);
      position_turn(-90, 600, 110);
      position_drive(0,-20,-90,false,127,1500, 0, 0, 0);//strafe to line up with wall
      reset_position_full(0,0,0);
      loader_left.move(-20);
      loader_right.move(-20);
      position_drive(0,9,0,false,127,1000, 0, 0, 0);//strafe to line up with wall
      loader_left.move(0);
      loader_right.move(0);
      angler_pid(1189, true, 127, true);
      pros::delay(1600);
      position_drive(0,-5,0,false,127,1000, 0, 0, 0);//strafe to line up with wall
      angler_pid(3000, true, 100, false, 2000);
      timerAuto = pros::millis() - initial_time;
      controller.print(0, 0, "Timer Auto: %d", timerAuto/1000);

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
      position_drive(0, 117.5, 0, false, 127, 9000, 127, 0, 120, 20, 70);
      position_turn(45, 600, 127);
      position_drive(12.1, 128.6, 45, false, 127, 9000, 0, 0, 0, 10, 70);
      angler_pid(1189, true, 127, true);
      pros::delay(1600);
      position_drive(0,117,0,false,127,1000, 0, 0, 0);//strafe to line up with wall
      angler_pid(3000, true, 100, false, 2000);
      position_turn(-90, 600, 127);
      position_drive(25, 117, -90, false, 127, 9000, 0, 0, 0, 10, 70);
      reset_position_full(23, 117, 0);
      position_drive(-37, 117, 0, false, 127, 9000, 0, 0, 0, 10, 70);
    }
  printf("orientation %f \n", radToDeg(orientation));
  printf("position.x %f \n", position.x);
  printf("position.y %f \n", position.y);
}
