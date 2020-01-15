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
  loader_left.move(127);
  loader_right.move(127);
  position_drive(0,15,0,false,127,1200);//pick up first set of cubes
  position_turn(90,400,100);
  position_drive(8,15,90,false,70,1500);//pick up first set of cubes
  loader_left.move(100);
  loader_right.move(100);
  position_turn(45,500,100);
  // angler_pid(1100, 20000);
  position_drive(33,43,45,false,60,2500);//pick up first set of cubes
  // angler_pid(870, 500, 127, false);
  loader_left.move(110);
  loader_right.move(110);
  position_drive(0,40,-90,false,80,3800);//pick up first set of cubes
  loader_left.move(100);
  loader_right.move(100);
  //angler_pid(1000, 3000);
  position_drive(-22,3,-150,false,100,3700);//pick up first set of cubes
  loader_left.move(0);
  loader_right.move(0);
  angler_pid(2600, 4500);
  pros::delay(3000);
  position_drive(-16,14,-150,false,100,3800);//pick up first set of cube
  //position_drive(15,20,90,false,100,3000);//come back and line up with the 5th cube
  // position_drive(15,40,10,true,100,3000);//pick up the 5th cube
  // position_drive(-5,25,-90,true,100,3000);//line up with the wall and face scoring
  // position_drive(-10,0,0,true,100,3000);//drive towards the scoring zone
  // position_drive(-10,-10,-90,false,100,3000);//drive towards the scoring zone
   timerAuto = pros::millis() - initial_time;


  // // position_drive(0,20,0,110,3000);//pick up first set of cubes
  // // position_drive(20,0,0,110,3000);//pick up first set of cubes
  // // position_drive(20,20,0,110,3000);//pick up first set of cubes
  // timerAuto = (pros::millis() - initial_time)/1000;
  // printf("timerAuto %f \n", timerAuto);

  //position_drive(0,10,0,110,3000);//pick up first set of cubes
  // position_drive(0,5,90,110,500);//pick up first set of cubes
  // position_drive(20,5,90,110,500);//pick up first set of cubes
  // angler_pid(2425, 20000);
  // angler_pid(817, 0, 80, false);
   //position_drive(0,20,0,100,3000);
  // position_turn(90, 1000, 100);
  // position_drive(10,20,90,100,3000);
  // reset_position_full(0, 20, 0);
  //position_drive(0,0,90,100,3000);
  // position_drive(20,40,0,100,2000);
  // position_drive(20,20,0,100,2000);
  // position_turn(-90, 1000, 100);

//mini auto
   // position_drive(0,20,0,100,2000);
   // position_drive(20,0,0,100,2000);
   // position_drive(20,20,0,100,2000);
   // position_drive(20,0,0,100,2000);
   // position_turn(90, 1000, 100);
   // reset_position_full(20, 0, 0);
   // position_drive(20,5,0,100,2000);

  // position_drive(0,0,0,70,2000);
  // position_drive(0,20,0,100,2000);
  // position_drive(20,20,0,100,2000);
  // position_drive(20,0,0,100,2000);
  // position_drive(0,0,0,70,2000);

//RED FRONT AUTO
  if(switcher == 1){
    loader_left.move(127);
    loader_right.move(127);
    position_drive(0,20,0,false,100,3000);//pick up first set of cubes
    position_drive(15,20,90,true,100,3000);//come back and line up with the 5th cube
    position_drive(15,40,10,true,100,3000);//pick up the 5th cube
    position_drive(-5,25,-90,true,100,3000);//line up with the wall and face scoring
    position_drive(-10,0,0,true,100,3000);//drive towards the scoring zone
    position_drive(-10,-10,-90,false,100,3000);//drive towards the scoring zone



    //second possible autonomous
    // loader_left.move(127);
    // loader_right.move(127);
    // angler_pid(3100, 5000);
    // lift(1780, 5000);
    // position_drive(0,40,0,false,100,3000);//drive towards tower
    // lift(900, 0);//pick up cubes
    // pros::delay(1000);
    // position_drive(0,35,0,false,100,3000);//back up to pick up final tower cube
    // position_drive(0,43,0,false,100,3000);//pick up final tower cube
    // position_drive(0,30,90,false,100,3000);//come back and line up with 5th cubes
    // position_drive(15,30,90,false,100,3000);//pick up 5th cube
    // position_drive(25,43,180,false,100,3000);//line up with wall and face scoring zone
    // position_drive(25,10,180,false,100,3000);//drive towards the scoring zone
    // angler_pid(900, 20000);//Score
    // pros::delay(1800);
    // loader_left.move(-90);
    // loader_right.move(-90);
    // pros::delay(1800);
    // position_drive(25,20,90,false,127,3000);//drive back
  }

//RED BACK AUTO
  if(switcher == 2){
    initial_time = pros::millis();
    loader_left.move(127);
    loader_right.move(127);
    position_drive(0,53,0,false,50,3000);//pick up first set of cubes
    loader_left.move(40);
    loader_right.move(40);
    // position_drive(0,8,0,false,115,3000);//pick up first set of cubes
    // reset_position_full(0, 8, 0);
    // position_drive(25,5,0,false,90,3000);//strafe to the second set
    // position_drive(25,45,0,false,100,3000);//pick up second set of cubes
    // loader_left.move(0);
    // loader_right.move(0);
    position_drive(-2,10,0,false,127,4500);//come back facing the scoring zone
    loader_left.move(0);
    loader_right.move(0);
    position_turn(90, 3000, 110);
    loader_left.move(-50);
    loader_right.move(-50);
    pros::delay(300);
    loader_left.move(0);
    loader_right.move(0);
    position_drive(0,-20,90,false,127,1500);//strafe to line up with wall
    reset_position_full(0,0,0);
    position_drive(0,9,0,false,127,1000);//strafe to line up with wall
    angler_pid(2625, 20000);
    pros::delay(3000);
    position_drive(0,-5,0,false,127,1000);//strafe to line up with wall
    timerAuto = pros::millis() - initial_time;

    // second possible autonomous
    // loader_left.move(127);
    // loader_right.move(127);
    // position_drive(0,50,0,100,3000);//pick up first set of cubes
    // position_drive(0,50,0,100,3000);//

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
      position_drive(0,45,0,false,100,3000);//pick up first set of cubes
      position_drive(0,25,90,false,100,3000);//come back and line up with the 5th cube
      position_drive(15,25,90,false,100,3000);//pick up the 5th cube
      position_drive(27,25,180,false,100,3000);//line up with the wall and face scoring
      position_drive(27,10,180,false,100,3000);//drive towards the scoring zone
      angler_pid(900, 20000);//Score
      pros::delay(1800);
      loader_left.move(-90);
      loader_right.move(-90);
      pros::delay(1800);
      position_drive(27,20,90,false,127,3000);//drive back

    }

  //BLUE BACK AUTO
    if(switcher == 6){
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
      position_drive(0,50,0,false,100,3000);//pick up first set of cubes
      position_drive(20,5,0,false,110,3000);//strafe to the second set
      position_drive(20,50,0,false,100,3000);//pick up second set of cubes
      loader_left.move(0);
      loader_right.move(0);
      position_drive(20,5,90,false,110,3000);//come back facing the scoring zone
      position_drive(20,-5,90,false,100,500);//strafe to line up with wall
      position_drive(25,position.y,90,false,127,3000);//drive to scoring zone
      angler_pid(900, 20000);//Score
      pros::delay(1800);
      loader_left.move(-90);
      loader_right.move(-90);
      pros::delay(1800);
      position_drive(15,position.y,90,false,127,3000);
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
      position_drive(0,125,0,false,100,3000);//pick up first set of cubes
      position_drive(15,125,0,false,100,3000);//
      angler_pid(900, 20000);//Score
      pros::delay(1800);
      loader_left.move(-90);
      loader_right.move(-90);
      pros::delay(1800);
      loader_left.move(127);
      loader_right.move(127);
      position_drive(15,115,0,false,100,3000);//
      position_drive(-5,115,180,false,100,3000);//
      position_drive(-5,140,180,false,100,3000);//
      position_drive(-5,80,180,false,100,3000);//
      position_drive(-5,85,180,false,100,3000);//
      lift(1780, 5000);
      loader_left.move(-90);
      loader_right.move(-90);
      lift(900, 0);
      position_drive(-5,120,-90,false,100,3000);//
      loader_left.move(127);
      loader_right.move(127);
      position_drive(-20,120,-90,false,100,3000);//

    }
  printf("orientation %f \n", radToDeg(orientation));
  printf("position.x %f \n", position.x);
  printf("position.y %f \n", position.y);
}
