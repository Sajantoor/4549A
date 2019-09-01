#include "main.h"
#include "drive.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "ritam_drive.h"
#include "all_used.h"

void autonomous()
{
  pros::lcd::initialize();
  full_position_reset();

//position_drive(0,0,0,40,250);
//position_turn(90,500,78);
// pros::delay(2000);
//position_turn2(0.5*pi, cw, 0.17, 35, 7.3);
//position_face_point(35,10, 300);
//position_drive(0,0,0,40,300);
//position_face_point2(71, 20, cw, 0, 35, 0, 30);

position_drive(0, 0, 0, 20, 0, 100, 1, 100);
// position_turn2(-0.5*pi, ccw, 0.17, 35, 7.3);
// position_turn2(0, cw, 0.17, 35, 7.3);
position_drive(0, 20, 0, 0, 0, -100, 1, 100);

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
