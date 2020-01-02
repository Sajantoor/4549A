#include "main.h"
#include "drive.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "all_used.h"
#include "lift.h"
#include "pid.h"
#include "vision.h"

const int PURPLE = 1;
const int ORANGE = 2;
const int GREEN = 3;
const int DETECTION_THRESHOLD = 0; // area of smallest possible acceptable cube
const int MAX_CUBE_SIZE = 45264; // stop before cube gets this big
// -32084 is the error value for x
const int ERROR_X = -32084;


int falsePositiveCheck[3];
int targetSelection(float x, float width, float height, int i) {
  // no object detected
  if (x != ERROR_X && (falsePositiveCheck[i] > 100)) {
    if (width == 0 || height == 0) {
      return 0;
    }

  } else if (x != ERROR_X) {
    falsePositiveCheck[i] = falsePositiveCheck[i] + 1;
  } else {
    falsePositiveCheck[i] = 0;
    return 0;
  }

  return width * height;
}

void vision_tracking(void*ignore) {
  // init
  // color signiatures
  pros::vision_signature_s_t PURPLE_CUBE = pros::Vision::signature_from_utility(PURPLE, 643, 1735, 1188, 12159, 15089, 13624, 3.000, 0);
  pros::vision_signature_s_t ORANGE_CUBE = pros::Vision::signature_from_utility(ORANGE, 9263, 11967, 10614, -3327, -2175, -2750, 3.000, 0);
  pros::vision_signature_s_t GREEN_CUBE = pros::Vision::signature_from_utility(GREEN, -10151, -7167, -8658, -5641, -2415, -4028, 3.000, 0);

  int cubeColor = 0;
  int largestCube = 0;
  bool target = false;
  int detectionValues[3];
  float largestSize;
  // zero point on sensor is the middle
  vision_sensor.set_zero_point(pros::E_VISION_ZERO_CENTER);
  while(true) {
    if (!target) {
      // detect all colours of cubes
      pros::vision_object_s_t purpleDetection = vision_sensor.get_by_sig(0, PURPLE);
      pros::vision_object_s_t orangeDetection = vision_sensor.get_by_sig(0, ORANGE);
      pros::vision_object_s_t greenDetection = vision_sensor.get_by_sig(0, GREEN);

      detectionValues[0] = targetSelection(purpleDetection.x_middle_coord, purpleDetection.width, purpleDetection.height, 0);
      detectionValues[1] = targetSelection(orangeDetection.x_middle_coord, orangeDetection.width, orangeDetection.height, 1);
      detectionValues[2] = targetSelection(greenDetection.x_middle_coord, greenDetection.width, greenDetection.height, 2);

      for (size_t i = 0; i < 3; i++) {
        if (detectionValues[i] > largestCube) {
          largestCube = detectionValues[i];
          cubeColor = i + 1;
        }
      }

      if (largestCube > DETECTION_THRESHOLD) {
        printf("target selected: %i \n \n", cubeColor);
        target = true;
      } else {
        largestCube = 0;
        cubeColor = 0;
      }

      turn_set(30);
    } else {
      pros::vision_object_s_t cube = vision_sensor.get_by_sig(0, cubeColor);
      int direction;
      // motion part
      float x = cube.x_middle_coord;
      float width = cube.width;
      float sizeValue = cube.width * cube.height;
      if (sizeValue > largestSize) {
        largestSize = sizeValue;
      }

      // printf("size value: %f \n \n", largestSize);
      if (width != 0) {
        if (x > 0) {
          direction = 1;
        } else {
          direction = -1;
        }

        if (fabs(x) > 50) {
          turn_set(30 * direction);
        } else if (sizeValue >= MAX_CUBE_SIZE) {
          set_drive(0, 0);
        } else if (sizeValue < MAX_CUBE_SIZE) {
          set_drive(30, 30);
        } /* else if (sizeValue > MAX_CUBE_SIZE) {
          set_drive(-30, -30);
        } */ else {
          set_drive(0, 0);
        }

      } else {
        printf("target lost! \n \n");
        set_drive(0, 0);
        cubeColor = 0;
        largestCube = 0;
        target = false;
      }
    }

    pros::delay(20);
  }
}

//BUG: if multiple cubes of the same colour exist of if the cube is moved back behind other coloured cubes. should switch to other cubes based on size_t
// FIX: Needs to check for other cubes sizes while driving
