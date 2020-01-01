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

  // position_drive2(0,20,0,100,3000);
  // position_turn(90, 1000, 100);
  // position_drive2(10,20,90,100,3000);
  // reset_position_full(0, 20, 0);
  //position_drive2(0,0,90,100,3000);
  // position_drive2(20,40,0,100,2000);
  // position_drive2(20,20,0,100,2000);
  // position_turn(-90, 1000, 100);

//mini auto
   // position_drive2(0,20,0,100,2000);
   // position_drive2(20,0,0,100,2000);
   // position_drive2(20,20,0,100,2000);
   // position_drive2(20,0,0,100,2000);
   // position_turn(90, 1000, 100);
   // reset_position_full(20, 0, 0);
   // position_drive2(20,5,0,100,2000);

  // position_drive2(0,0,0,70,2000);
  // position_drive2(0,20,0,100,2000);
  // position_drive2(20,20,0,100,2000);
  // position_drive2(20,0,0,100,2000);
  // position_drive2(0,0,0,70,2000);

//RED FRONT AUTO
  if(switcher == 1){
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
    position_drive2(0,45,0,100,3000);//pick up first set of cubes
    position_drive2(0,25,90,100,3000);//come back and line up with the 5th cube
    position_drive2(15,25,90,100,3000);//pick up the 5th cube
    position_drive2(27,25,180,100,3000);//line up with the wall and face scoring
    position_drive2(27,10,180,100,3000);//drive towards the scoring zone
    angler_pid(900, 20000);//Score
    pros::delay(1800);
    loader_left.move(-90);
    loader_right.move(-90);
    pros::delay(1800);
    position_drive2(27,20,90,127,3000);//drive back

  }

//RED BACK AUTO
  if(switcher == 2){
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
    position_drive2(0,50,0,100,3000);//pick up first set of cubes
    position_drive2(20,5,0,110,3000);//strafe to the second set
    position_drive2(20,50,0,100,3000);//pick up second set of cubes
    loader_left.move(0);
    loader_right.move(0);
    position_drive2(20,5,90,110,3000);//come back facing the scoring zone
    position_drive2(20,-5,90,100,500);//strafe to line up with wall
    position_drive2(25,position.y,90,127,3000);//drive to scoring zone
    angler_pid(900, 20000);//Score
    pros::delay(1800);
    loader_left.move(-90);
    loader_right.move(-90);
    pros::delay(1800);
    position_drive2(15,position.y,90,127,3000);
  }

//1 POINT UNLOCK AUTO RED RIGHT
  if(switcher == 3){
    position_drive2(16, 0, 0, 127, 1500);
    position_drive2(0, 0, 0, 127, 1200);

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
    position_drive2(-16, 0, 0, 127, 1500);
    position_drive2(0, 0, 0, 127, 1200);

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
      position_drive2(0,45,0,100,3000);//pick up first set of cubes
      position_drive2(0,25,90,100,3000);//come back and line up with the 5th cube
      position_drive2(15,25,90,100,3000);//pick up the 5th cube
      position_drive2(27,25,180,100,3000);//line up with the wall and face scoring
      position_drive2(27,10,180,100,3000);//drive towards the scoring zone
      angler_pid(900, 20000);//Score
      pros::delay(1800);
      loader_left.move(-90);
      loader_right.move(-90);
      pros::delay(1800);
      position_drive2(27,20,90,127,3000);//drive back

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
      position_drive2(0,50,0,100,3000);//pick up first set of cubes
      position_drive2(20,5,0,110,3000);//strafe to the second set
      position_drive2(20,50,0,100,3000);//pick up second set of cubes
      loader_left.move(0);
      loader_right.move(0);
      position_drive2(20,5,90,110,3000);//come back facing the scoring zone
      position_drive2(20,-5,90,100,500);//strafe to line up with wall
      position_drive2(25,position.y,90,127,3000);//drive to scoring zone
      angler_pid(900, 20000);//Score
      pros::delay(1800);
      loader_left.move(-90);
      loader_right.move(-90);
      pros::delay(1800);
      position_drive2(15,position.y,90,127,3000);
    }

  //1 POINT UNLOCK AUTO BLUE RIGHT
    if(switcher == 7){
      position_drive2(16, 0, 0, 127, 1500);
      position_drive2(0, 0, 0, 127, 1200);

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
      position_drive2(-16, 0, 0, 127, 1500);
      position_drive2(0, 0, 0, 127, 1200);

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

    }
  printf("orientation %f \n", radToDeg(orientation));
  printf("position.x %f \n", position.x);
  printf("position.y %f \n", position.y);
}
