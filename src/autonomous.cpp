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

void unlock() {
  lift(2500, 20000);
  pros::delay(1000);
  lift(0, 1000);
  pros::delay(600);
  loader_left.move(127);
  loader_right.move(127);
}

void autonomous() {
  printf("PositionX: %f || PositionY: %f || Orientation: %f \n \n", position.x, position.y, orientation);
  printf("position.x %f \n", position.x);
  printf("position.y %f \n", position.y);
  reset_position_full(0, 0, 0);
  beginning_orientation = 0;
  int initial_time = pros::millis();
  gyro.reset();
  inertial.reset();
  pros::delay(2000);
  position_turn(90, 20000, 80); // turn
  // if(switcher == 0){
  //   unlock();
  // }

//RED FRONT AUTO
  if (switcher == 1) {
    unlock();
    position_drive2(0, 0, 0, 22, 0, 90, 0.1, 0, 1000, 50, 40, false); // go straight
    position_turn(90, 900, 127); // turn
    position_drive2(0, 22, 35, 22, 0, 127, 0.1, 0, 1500, 50, 90, false); // drive and pick up line
    position_drive2(35, 22, 25, 22, 0, -127, 0.1, 5, 1500, 50, 90, false);
    position_turn(115, 2000, 127);
    position_drive2(35, 22, 45, 18, 0, 127, 0.1, 0, 1500, 50, 40, false);
    position_drive2(45, 18, 9, 21, 0, -127, 0.1, 0, 1500, 50, 40, false);
    position_turn(240, 2000, 127);
    drive_line_up(70,450);
    // position_drive2(-9, 21, -7, 115, 0, 100, 0.1, 0, 1500, 50, 40, false);
    angler_pid(-4500, true, 127, true);
    pros::delay(2500);
    drive_line_up(-127, 400);
    // drive_line_up(60, 600);
    // autoIntakeFunc(0);
    // angler_pid(-4500, true, 127, false);
    // pros::delay(3000);
    // drive_line_up(-127, 600);
    timerAuto = pros::millis() - initial_time;
    controller.print(0, 0, "Timer Auto: %f", (timerAuto/10));
  }

  //RED BACK AUTO
  if(switcher == 2){
    unlock();
    position_drive2(0, 0, 0, 27, 0, 127, 0.1, 0, 1300, 50, 90, false);
    position_drive2(0, 20, 0, 25, 0, 50, 0.1, 0, 1000, 50, 90, false);
    // autoIntakeFunc(0);
    // loader_right.move(127);
    // loader_left.move(127); // wait till cube gets in tray fully
    // lift(750, 1000);
    // pros::delay(500);
    // position_drive2(0, 25, 0, 45, 0, 50, 0.1, 0, 1000, 50, 80, false);
    // lift(0, 1000);
    // pros::delay(1000);
    // autoIntakeFunc(127);
    // autoIntakeFunc(0);
    // loader_right.move(0);
    // loader_left.move(0);
    loader_left.move(0);
    loader_right.move(0);
    position_drive2(0, 25, 25, 5, 0, -127, 0.1, 0, 1500, 40, 127, false); // go back
    position_turn(0, 1000, 127);
    loader_left.move(127);
    loader_right.move(127);
    position_drive2(25, 0, 25, 45, 0, 80, 0.05, 0, 2700, 50, 95, false); // pick up line of cubes
    position_drive2(25, 45, 25, 18, 0, -127, 0.1, 0, 2000, 50, 127, false); // go back
    loader_left.move(0);
    loader_right.move(0);
    position_turn(128, 1000, 110); // turn to stack
    position_drive2(25, 20, 31, 10, 0, 127, 0.05, 0, 1000, 50, 80, false); // go back
    angler_pid(-4500, true, 127, true);
    pros::delay(2500);
    drive_line_up(-127, 700);

    timerAuto = pros::millis() - initial_time;
    controller.print(0, 0, "Timer Auto: %f", timerAuto);
  }

//1 POINT UNLOCK AUTO RED RIGHT
  if(switcher == 3){
    position_drive(16, 0, 0,false, 127, 1500);
    position_drive(0, 0, 0,false, 127, 1200);

unlock();
  }

  //1 POINT UNLOCK AUTO RED LEFT
  if(switcher == 4){
    position_drive2(0, 0, 0, 15, 0, 110, 0.1, 0, 2300, 50,40);// pick up cubes
    position_drive2(0, 15, 0, 0, 0, -110  , 0.1, 0, 2300, 50,40);// pick up cubes

unlock();
  }

  //BLUE FRONT AUTO
  if(switcher == 7){
    unlock();
    position_drive2(0, 0, 0, 22, 0, 90, 0.1, 0, 1000, 50, 40, false); // go straight
    position_turn(-90, 900, 127); // turn
    position_drive2(0, 22, -35, 22, 0, 127, 0.1, 0, 1500, 50, 90, false); // drive and pick up line
    position_drive2(-35, 22, -25, 22, 0, -127, 0.1, 5, 1500, 50, 90, false);
    position_turn(-115, 2000, 127);
    position_drive2(-35, 22, -45, 18, 0, 127, 0.1, 0, 1500, 50, 40, false);
    position_drive2(-45, 18, -9, 21, 0, -127, 0.1, 0, 1500, 50, 40, false);
    position_turn(-240, 2000, 127);
    drive_line_up(70,450);
    loader_left.move(0);
    loader_right.move(0);
    // position_drive2(-9, 21, -7, 115, 0, 100, 0.1, 0, 1500, 50, 40, false);
    angler_pid(-4500, true, 127, true);
    pros::delay(2500);
    drive_line_up(-127, 400);
    // drive_line_up(60, 600);
    // autoIntakeFunc(0);
    // angler_pid(-4500, true, 127, false);
    // pros::delay(3000);
    // drive_line_up(-127, 600);
    timerAuto = pros::millis() - initial_time;
    controller.print(0, 0, "Timer Auto: %f", (timerAuto/10));
  }

  //BLUE BACK AUTO
    if(switcher == 8){
      unlock();
      position_drive2(0, 0, 0, 27, 0, 127, 0.1, 0, 1300, 50, 90, false);
      position_drive2(0, 20, 0, 25, 0, 50, 0.1, 0, 1000, 50, 90, false);
      // autoIntakeFunc(0);
      // loader_right.move(127);
      // loader_left.move(127); // wait till cube gets in tray fully
      // lift(750, 1000);
      // pros::delay(500);
      // position_drive2(0, 25, 0, 45, 0, 50, 0.1, 0, 1000, 50, 80, false);
      // lift(0, 1000);
      // pros::delay(1000);
      // autoIntakeFunc(127);
      // autoIntakeFunc(0);
      // loader_right.move(0);
      // loader_left.move(0);
      position_drive2(0, 25, -27, 3, 0, -127, 0.1, 0, 2000, 40, 127, false); // go back
      position_turn(0, 1000, 127);
      loader_left.move(127);
      loader_right.move(127);
      position_drive2(-25, 0, -24, 45, 0, 80, 0.05, 0, 2500, 50, 95, false); // pick up line of cubes
      position_drive2(-25, 48, -25, 18, 0, -127, 0.1, 0, 1300, 50, 127, false); // go back
      loader_left.move(0);
      loader_right.move(0);
      position_turn(-120, 3000, 110); // turn to stack
      position_drive2(-25, 18, -40, 7, 0, 127, 0.05, 0, 1200, 50, 80, false); // go back
      angler_pid(-4500, true, 127, true);
      pros::delay(2500);
      drive_line_up(-127, 700);

      timerAuto = pros::millis() - initial_time;
      controller.print(0, 0, "Timer Auto: %f", timerAuto);
    }

  //1 POINT UNLOCK AUTO BLUE RIGHT
    if(switcher == 9){
      position_drive(16, 0, 0,false, 127, 1500);
      position_drive(0, 0, 0, false,127, 1200);

unlock();
    }

    //1 POINT UNLOCK AUTO BLUE LEFT
    if(switcher == 10){
      position_drive(-16, 0, 0,false,127, 1500);
      position_drive(0, 0, 0, false,127, 1200);
unlock();
    }


    //SKILLS
    if(switcher == 13) {
      // SKILLS -----------------
      lift(2500, 20000);
      pros::delay(1500);
      lift(0, 1000);
      pros::delay(1500);
      loader_left.move(127);
      loader_right.move(127);
      autoIntakeFunc(127);
      position_drive2(0, 0, 0, 45, 0, 75, 0.05, 0, 3000, 50, 40, false); // drive forward to tower
      pros::delay(500);
      autoIntakeFunc(0);
      sensor_outtake();
      lift(2500, 20000);
      pros::delay(1500);
      position_turn(-25, 1000, 100);
      // position_drive2(0, 45, -5, 55, 0, 127, 0.05, 0, 2500, 30, 40, false); // go forward
      loader_right.move(-120);
      loader_left.move(-120);
      pros::delay(1500);
      position_drive2(-5, 55, 0, 35, 0, -127, 0.05, 8, 3000, 60, 40, false);
      position_turn(0, 1000, 127);
      loader_right.move(0);
      loader_left.move(0);
      lift(0, 4000);
      pros::delay(800);
      autoIntakeFunc(127);
      position_drive2(0, 42, 0, 117, 0, 65, 0.05, 0, 3000, 200, 40, false); // pick up rest of cubes
      position_drive2(0, 117, 0, 98, 0, -110, 0.05, 0, 3000, 20, 40, false); // pick up rest of cubes
      autoIntakeFunc(0);
      sensor_outtake();
      lift(2800, 20000);
      pros::delay(1000);
      position_turn(79, 1500, 60); // turn to stack
      drive_line_up(50, 400);
      loader_right.move(-127);
      loader_left.move(-127);
      pros::delay(1000);
      drive_line_up(-50, 400);
      lift(0, 4000);
      loader_right.move(0);
      loader_left.move(0);
      position_turn(25, 1400, 127); // turn to stack
      position_drive2(0, 87, 18.5, 119.5, 0, 80, 0.05, 0, 1800, 200, 40, false); // pick up rest of cubes
      angler_pid(-4500, true, 115, false);
      pros::delay(3500);
      angler.move(127);
      drive_line_up(-90,700);
      pros::delay(3000);
      angler.move(0);
      position_turn(0, 700, 100); // turn to line up
      lift(2500, 20000);
      pros::delay(1000);
      drive_line_up(50,4000); // line up
      reset_position_full(0, 0, 0);
      timerAuto = pros::millis() - initial_time;
      controller.print(0, 0, "Timer Auto: %f", (timerAuto/10));
      position_drive2(0, 0, 0, -10, 0, -100, 0.1, 0, 2500, 50, 40, false);
      position_turn(-90, 1000, 100); // face tower
      lift(0, 4000);
      loader_right.move(127);
      loader_left.move(127);
      position_drive2(0, -10, -33, -10, 0, 100, 0.1, 0, 2000, 50, 40, false); // go forward and pick up cube
      position_drive2(-33, -10, -20, -10, 0, -127, 0.1, 7, 2000, 50, 40, false); // go back
      autoIntakeFunc(0);
      sensor_outtake();
      lift(2800, 20000);
      pros::delay(1000);
      position_drive2(-20, -10, -39, -10, 0, 127, 0.1, 5, 2000, 50, 40, false); // go forward and outtake
      loader_right.move(-127);
      loader_left.move(-127);
      pros::delay(1000);
      position_drive2(-39, -10, -20, -10, 0, -127, 0.1, 12, 2000, 50, 40, false); // go back
      position_turn(-180, 1000, 100); // turn to face cubes
      lift(0, 4000);
      drive_line_up(-50,1800);
      reset_position_full(0, 0, 0);
      autoIntakeFunc(127);
      position_drive2(0, 0, 1.5, 113, 0, 60, 0.05, 0, 9000, 200, 40, false); // get all the cubes in line
      pros::delay(1300);
      position_drive2(1.5, 113, 1.5, 100, 0, -127, 0.05, 5, 2000, 60, 40, false); // go back
      // autoIntakeFunc(0);
      // sensor_outtake();
      // lift(2800, 20000);
      // pros::delay(2000);
      // position_turn(90, 1000, 100); // turn to tower
      // position_drive2(1.5, 105, 7, 105, 0, 127, 0.05, 0, 1500, 200, 40, false); // go forward to tower
      // loader_right.move(-120);
      // loader_left.move(-120);
      // pros::delay(1500);
      position_drive2(9, 108, -21.8, 102, 0, -100, 0.05, 0, 1500, 200, 40, false); // go back
      // lift(0, 2000);
    position_turn(-48, 1000, 100); // turn to stack
      loader_right.move(0);
      loader_left.move(0);
      drive_line_up(90, 1000);
      position_drive2(-21.8, 102, -30, 117, 0, 100, 0.05, 0, 1000, 200, 40, false); // go to stack zone
      angler_pid(-4500, true, 127, false);
      pros::delay(3000);
      angler_pid(0, true, 127, false, 2000);
      drive_line_up(-90, 500);
      timerAuto = pros::millis() - initial_time;
      controller.print(0, 0, "Timer Auto: %f", (timerAuto/10));
      // position_turn(110, 20000, 100); // turn to stack
      // loader_right.move(127);
      // loader_left.move(127);
      // position_drive2(position.x, position.y, 34, 61, 0, 127, 0.05, 0, 4000, 200, 40, true); // go to stack zone
      // autoIntakeFunc(0);
      // sensor_outtake();
      // lift(2800, 20000);
      // pros::delay(1000);
      // loader_right.move(-120);
      // loader_left.move(-120);
      // timerAuto = pros::millis() - initial_time;
      // controller.print(0, 0, "Timer Auto: %f", (timerAuto/10));
      // SKILLS --------------

    }
  printf("orientation %f \n", radToDeg(orientation));
  printf("position.x %f \n", position.x);
  printf("position.y %f \n", position.y);
}
