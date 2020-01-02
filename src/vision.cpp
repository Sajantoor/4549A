#include "main.h"
#include "drive.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "all_used.h"
#include "lift.h"
#include "pid.h"
#include "vision.h"

float calcArea(float width, float height) {
  return width * height;
}

void vision_tracking(void*ignore) {
  // init
  // color signiatures
  pros::vision_signature_s_t PURPLE_CUBE = pros::Vision::signature_from_utility(1, 643, 1735, 1188, 12159, 15089, 13624, 3.000, 0);
  pros::vision_signature_s_t ORANGE_CUBE = pros::Vision::signature_from_utility(2, 9263, 11967, 10614, -3327, -2175, -2750, 3.000, 0);
  pros::vision_signature_s_t GREEN_CUBE = pros::Vision::signature_from_utility(3, -10151, -7167, -8658, -5641, -2415, -4028, 3.000, 0);
  // zero point on sensor is the middle
  vision_sensor.set_zero_point(pros::E_VISION_ZERO_CENTER);
  while(true) {
    pros::vision_object_s_t obj = vision_sensor.get_by_sig(0, 2);
    // no object detected
    // object detection
    if (obj.x_middle_coord != -32084) {
      int x = obj.x_middle_coord;
      int y = obj.y_middle_coord;
      int direction;
      float size = calcArea(obj.width, obj.height);

      if (x > 0) {
        direction = 1;
      } else {
        direction = -1;
      }

      if (fabs(x) > 50) {
        turn_set(30 * direction);
      } else if (size > 52080) {
        set_drive(0, 0);
      } else {
        set_drive(30, 30);
      }



      // position_drive2(20, 20, 20 * direction, 30, 5000);
      //  position_drive2(float ending_point_x, float ending_point_y, float target_angle, float max_power, unsigned int timeout)


      // printf("x: %i \n y: %i \n \n", x, y);
      printf("size: %f \n \n", size);
    } else {
      turn_set(30);
    }
  }
}
