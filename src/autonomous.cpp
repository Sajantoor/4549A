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

  if (switcher == 11) {
    beginning_orientation = 0;
    reset_position_full(position, 10,24, 0);
    printf("PositionX: %f || PositionY: %f || Orientation: %f \n \n", position.x, position.y, orientation);
    /*
    position_drive(10, 24, 10, 50, 0, 100, 0.5, 0);
    position_drive(10, 50, 10, 10, -80, -100, 0.5, 5);
    */
  }
}
