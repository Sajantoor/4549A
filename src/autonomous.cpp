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

  printf("switcher %f", switcher);
  // if no auto unlock
  if (switcher == 0) {
    lift(1950, 20000);
    pros::delay(1000);
    lift(0, 0);
  }



 //position_turn(15, 5000, 127);
  //position_drive2(0, 0, 0, 20, 0, 50, 0.5, 0, 11000, 50,40);// pick up cubes
  // position_turn(0, 1500, 127);
  // intakeSpeed = 127;
  // position_drive2(0, 40, 0, 65, 0, 100, 0.5, 0, 11000, 100,40);// pick up cubes
  //position_drive2(0, 0, 0, 25, 0, 127, 0.5, 0, 9000, 30);// pick up cubes  //position_turn(91, 1000, 127);

  // the auto we worked on in class on friday, working
            //   initial_time = pros::millis();
            //   lift(1950, 20000);
            //   pros::delay(1000);
            //   lift(0, 0);
            //   pros::delay(1000);
            //   loader_left.move(127);
            //   loader_right.move(127);
            //   position_drive2(0, 0, 0, 44.4, 0, 110, 0.1, 0, 2300, 50,40);// pick up cubes
            //   position_drive2(0, 44.4, 25, 9, 0, -110, 1, 0, 2000, 50,40);// pick up cubes
            //   position_turn(-5, 400, 80);
            //   controller.print(0, 0, "driving straight");
            //   position_drive2(25, 9, 23, 50, 0, 127, 0.5, 0, 2600, 50,20);// pick up cubes
            //   position_drive2(23, 50, 26.5, 23, 0, -110, 0.05, 0, 1800, 50,40);// pick up cubes
            //   position_turn(120, 1500, 110);
            //   loader_left.move(0);
            //   loader_right.move(0);
            //     angler_pid(2200, true, 127, false, 0, true);
            //   position_drive2(26.5, 23, 42.5, 5, 0, 110, 0.05, 0, 950, 50,40);// pick up cubes
            //   angler_pid(1020, true, 127, true);
            //   pros::delay(2800);
            // angler_pid(3400, true, 127, false, 2000);
            // drive_line_up(-100, 1000);


    // end of auto

  // position_drive(33,12.1,135,false,127,4000);//pick up first set of cubes
  // loader_left.move(0);
  // loader_right.move(0);
  // angler_pid(1189, true, 127, true);
  // pros::delay(1600);
  // position_drive(26.5,23,0,false,127,1000);//strafe to line up with wall
  // angler_pid(3000, true, 100, false, 2000);
  // timerAuto = pros::millis() - initial_time;

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
    lift(1950, 20000);
    loader_left.move(-127);
    loader_right.move(-127);
    pros::delay(1500);
    lift(0, 0);
    pros::delay(1000);
    loader_left.move(127);
    loader_right.move(127);
    position_drive2(0, 0, 0, 44.4, 0, 110, 0.1, 0, 2300, 50,40);// pick up cubes
    position_drive2(0, 44.4, 25, 9, 0, -110, 1, 0, 2000, 50,40);// pick up cubes
    position_turn(-5, 400, 80);
    position_drive2(25, 9, 23, 50, 0, 127, 0.5, 0, 2600, 50,20);// pick up cubes
    position_drive2(23, 50, 26.5, 23, 0, -110, 0.05, 0, 1800, 50,40);// pick up cubes
    position_turn(117, 1500, 110);
    loader_left.move(0);
    loader_right.move(0);
      angler_pid(2200, true, 127, false, 0, true);
    position_drive2(26.5, 23, 43.5, 5, 0, 110, 0.05, 0, 950, 50,40);// pick up cubes
    angler_pid(1020, true, 127, true);
    pros::delay(2000);
  angler_pid(3400, true, 127, false, 2000);
  drive_line_up(-100, 1000);
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
      loader_left.move(-127);
      loader_right.move(-127);
      pros::delay(1600);
      lift(0, 1000);
      pros::delay(1000);
      loader_left.move(127);
      loader_right.move(127);
      position_drive2(0, 0, 0, 44.4, 0, 110, 0.1, 0, 2300, 50,40);// pick up cubes
      position_drive2(0, 44.4, -24, 3, 0, -110, 1, 0, 2000, 50,40);// pick up cubes
      position_turn(-5, 400, 80);
      position_drive2(-24, 3, -26, 50, 0, 127, 0.5, 0, 2600, 50,20);// pick up cubes
      position_drive2(-26, 50, -26.5, 23, 0, -110, 0.05, 0, 1800, 50,40);// pick up cubes
      position_turn(-135, 1500, 110);
      loader_left.move(0);
      loader_right.move(0);
        angler_pid(2200, true, 127, false, 0, true);
      position_drive2(-26.5, 23, -37, 3, 0, 110, 0.05, 0, 950, 50,40);// pick up cubes
      angler_pid(1020, true, 127, true);
      pros::delay(2000);
    angler_pid(3400, true, 127, false, 2000);
    drive_line_up(-100, 1000);
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
