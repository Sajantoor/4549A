#include "main.h"
#include "drive.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "all_used.h"
#include "lift.h"
#include "pid.h"
#include "vision.h"

// signature IDs
const int PURPLE = 1;
const int ORANGE = 2;
const int GREEN = 3;

const int DETECTION_THRESHOLD = 0; // area of smallest possible acceptable cube
const int MAX_CUBE_SIZE = 45264; // stop before cube gets this big
const int ERROR_X = -32084; // -32084 is the error value for x
const int FALSE_POSITIVE_THRESHOLD = 100; // threshold to confirm something has been detected

int falsePositiveCheck[3]; // false positive detections for each tracking color, to confirm something is detected a threshold must be reached
int cubeColor = 0; // ID of targeted cube color
int largestCube = 0; // largest cube of all tracked colors
bool target = false; // used to check if a color is being tracked or not

// return area of the cube using width and height
float sizeCheck(float x, float width, float height, int id) {
  // no object detected & reached false positive threshold
  if (x != ERROR_X && (falsePositiveCheck[id] > FALSE_POSITIVE_THRESHOLD)) {
    // area of cube in vision sensor units => not actual area
    return width * height;

  // object detection, increment false positive check
  } else if (x != ERROR_X) {
    falsePositiveCheck[id] = falsePositiveCheck[id] + 1;
  } else {
    // no object detected
    falsePositiveCheck[id] = 0;
    return 0;
  }
}

// ENHANCE: arguments could be taken to detect a certain color or something => ignore certain colors
float targetSelection() {
  float detectionValues[3]; // area of all detection colors
  // detecting values
  pros::vision_object_s_t purpleDetection = vision_sensor.get_by_sig(0, PURPLE);
  pros::vision_object_s_t orangeDetection = vision_sensor.get_by_sig(0, ORANGE);
  pros::vision_object_s_t greenDetection = vision_sensor.get_by_sig(0, GREEN);
  // check size of all detections
  detectionValues[0] = sizeCheck(purpleDetection.x_middle_coord, purpleDetection.width, purpleDetection.height, 0);
  detectionValues[1] = sizeCheck(orangeDetection.x_middle_coord, orangeDetection.width, orangeDetection.height, 1);
  detectionValues[2] = sizeCheck(greenDetection.x_middle_coord, greenDetection.width, greenDetection.height, 2);
  // gets the largest cube from the array and the color id
  for (size_t i = 0; i < 3; i++) {
    if (detectionValues[i] > largestCube) {
      largestCube = detectionValues[i];
      cubeColor = i + 1;
    }
  }
  // if the cube is bigger than smallest possible cube, target is selected
  if (largestCube > DETECTION_THRESHOLD) {
    printf("target selected: %i \n \n", cubeColor);
    target = true;
  } else {
    largestCube = 0;
    cubeColor = 0;
  }

  return largestCube;
}

void vision_tracking(void*ignore) {
  // init
  // color signiatures
  pros::vision_signature_s_t PURPLE_CUBE = pros::Vision::signature_from_utility(PURPLE, 643, 1735, 1188, 12159, 15089, 13624, 3.000, 0);
  pros::vision_signature_s_t ORANGE_CUBE = pros::Vision::signature_from_utility(ORANGE, 9263, 11967, 10614, -3327, -2175, -2750, 3.000, 0);
  pros::vision_signature_s_t GREEN_CUBE = pros::Vision::signature_from_utility(GREEN, -10151, -7167, -8658, -5641, -2415, -4028, 3.000, 0);

  float largestSize;
  // zero point on sensor is the middle
  vision_sensor.set_zero_point(pros::E_VISION_ZERO_CENTER);
  while(true) {
    if (!target) {
      // detect all colours of cubes
      targetSelection();
      // turns until cube detected
      turn_set(30);
    } else {
      // targeted cube
      pros::vision_object_s_t cube = vision_sensor.get_by_sig(0, cubeColor);
      int direction;  // direction of turning
      float x = cube.x_middle_coord;  // x is used to calculate turning, x direction
      float sizeValue = cube.width * cube.height; // y is used to calculate how far forward or backward to move, y direction

      // used for testing
      // if (sizeValue > largestSize) {
      //   largestSize = sizeValue;
      // }

      // printf("size value: %f \n \n", largestSize);

      // checks if cube still exists
      if (cube.width != 0) {
        // direction
        if (x > 0) {
          direction = 1;
        } else {
          direction = -1;
        }
        // basic movement
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
        // lost target
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
