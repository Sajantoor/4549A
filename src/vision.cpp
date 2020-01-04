#include "main.h"
#include "drive.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "all_used.h"
#include "lift.h"
#include "pid.h"
#include "vision.h"

// signature IDs
const int GREEN = 1;
const int ORANGE = 2;
const int PURPLE = 3;

const int DETECTION_THRESHOLD = 10000; // area of smallest possible acceptable cube
const int FALSE_POSITIVE_THRESHOLD = 50; // threshold to confirm something has been detected
const int DEEP_VISION_THRESHOLD = 1000; // apply deep detection algorithm if size is above this value and multiple objects detected
const int MIN_DEEP_VISION_THRESHOLD = 1000; // min size to be considered to be considered apart of deep vision
const int MAX_CUBE_SIZE = 45264; // stop before cube gets this big
const int ERROR_X = -32084; // -32084 is the error value for x

struct data {
  int width; // effective width of the cube
  int height; // effective height of the cube
  int size; // area, width * height
  int x;
  int id;
  bool deepVisionCheck;
};

data deepVisionData;
data currentCube;

int falsePositiveCheck[3]; // false positive detections for each tracking color, to confirm something is detected a threshold must be reached
int cubeColor = 0; // ID of targeted cube color
int largestCube = 0; // largest cube of all tracked colors
// bool target = false; // used to check if a color is being tracked or not
int targetedCube = 0; // cube currently being targeted (by ID)


void clearData(data * x) {
  x-> width = 0;
  x-> height = 0;
  x-> size = 0;
  x-> x = 0;
  x-> deepVisionCheck = 0;
}

void telemetry() {
//  pros::vision_object_s_t orangeDetection = vision_sensor.get_by_sig(0, ORANGE);
  // printf("left_coord: %i \n\n top_coord: %i \n\n", orangeDetection.left_coord, orangeDetection.top_coord);
  printf("width: %i \n\n height: %i \n \n", currentCube.width, currentCube.height);
  printf("size: %i \n \n", currentCube.size);
//  printf("size: %i \n \n", orangeDetection.width * orangeDetection.height);
  // printf("x: %i \n\n y: %i \n\n", orangeDetection.x_middle_coord, orangeDetection.y_middle_coord);
  printf("x: %i \n\n", currentCube.x);
  printf("object count: %i \n\n", vision_sensor.get_object_count());
}
// return area of the cube using width and height
float sizeCheck(float x, float width, float height, int id) {
  // no object detected & reached false positive threshold
  if (x != ERROR_X && (falsePositiveCheck[id] > FALSE_POSITIVE_THRESHOLD)) {
    // area of cube in vision sensor units => not actual area
    return width * height;

  // object detection, increment false positive check
  } else if (x != ERROR_X) {
    falsePositiveCheck[id] = falsePositiveCheck[id] + 1;
    if (falsePositiveCheck[id] > 10) {
      set_drive(0, 0);
    }
  } else {
    // no object detected
    falsePositiveCheck[id] = 0;
    return 0;
  }
}

float deepVision(int id) {
  int numObjects = vision_sensor.get_object_count();
  float smallestLeft; // smallest left coordinate
  float largestLeft; // largest
  float largestLeftWidth; // width of largest to calculate the right coordinate
  float largestTop; // largest top coordinate
  float smallestTop; //smallest
  float smallestTopHeight; // height of smallest top to calculate bottom coordinate

  if (numObjects > 5) numObjects = 5;

  pros::vision_object_s_t partArray[numObjects];

  for (size_t i = 0; i < numObjects; i++) {
    partArray[i] = vision_sensor.get_by_sig(i, id);

    if (partArray[i].width * partArray[i].height < MIN_DEEP_VISION_THRESHOLD) {
      // printf("Deep vision cancelled \n \n");
      break;
    }

    if (partArray[i].left_coord > largestLeft) {
      largestLeft = partArray[i].left_coord;
      largestLeftWidth = partArray[i].width;
    }
    // not else if here because what if smallest left is the first value, will cause bugs
    if (partArray[i].left_coord < smallestLeft) {
      smallestLeft = partArray[i].left_coord;
    }

    if (partArray[i].top_coord > largestTop) {
      largestTop = partArray[i].top_coord;
    }

    if (partArray[i].top_coord < smallestTop) {
      smallestTop = partArray[i].top_coord;
      smallestTopHeight = partArray[i].height;
    }
  }

  deepVisionData.deepVisionCheck = true;
  deepVisionData.width = (largestLeft + largestLeftWidth) - smallestLeft;
  deepVisionData.height = largestTop - (smallestTop - smallestTopHeight);
  deepVisionData.size = deepVisionData.width * deepVisionData.height;
  deepVisionData.x = smallestLeft + (deepVisionData.width / 2);

  if (deepVisionData.size < 0) {
    return 0;
  }

  return deepVisionData.size;
}

// ignore the targeted cube
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
    if ((vision_sensor.get_object_count() > 1) && (detectionValues[i] > DEEP_VISION_THRESHOLD)) {
      detectionValues[i] = deepVision(i + 1);
    }

    if (detectionValues[i] > largestCube) {
      largestCube = detectionValues[i];
      cubeColor = i + 1;
    }
  }
  // if the cube is bigger than smallest possible cube, target is selected
  if (largestCube > DETECTION_THRESHOLD) {
    if (largestCube == deepVisionData.size) {
      currentCube = deepVisionData;
    } else {
      // clearData(&deepVisionData);
    }
  } else {
    largestCube = 0;
    cubeColor = 0;
  }

  return largestCube;
}

void vision_tracking(void*ignore) {
  // init
  vision_sensor.set_exposure(150);
  // color signiatures
  pros::vision_signature_s_t GREEN_CUBE = pros::Vision::signature_from_utility(GREEN, -9567, -7177, -8372, -3137, -993, -2066, 4.300, 0);
  pros::vision_signature_s_t ORANGE_CUBE = pros::Vision::signature_from_utility(ORANGE, 7125, 8759, 7942, -2631, -1647, -2138, 2.500, 0);
  pros::vision_signature_s_t PURPLE_CUBE = pros::Vision::signature_from_utility(PURPLE, -143, 1049, 454, 8523, 12759, 10640, 3.200, 0);

  // float largestSize;
  // // zero point on sensor is the middle
  vision_sensor.set_zero_point(pros::E_VISION_ZERO_CENTER);
  while(true) {
    pros::vision_object_s_t orangeDetection = vision_sensor.get_by_sig(0, ORANGE);

    if (!targetedCube) { // any cube 1 - 3 would be true, 0 is false
      // detect all colours of cubes
      targetSelection();
      targetedCube = cubeColor;

      if (cubeColor != 0) {
        printf("target selected: %i \n \n", cubeColor);
      }

      // turns until cube detected
      turn_set(30);
    } else {
      // targeted cube
      pros::vision_object_s_t cube = vision_sensor.get_by_sig(0, targetedCube);

      if (currentCube.deepVisionCheck) {
        deepVision(targetedCube);
        currentCube = deepVisionData;
        printf("deep vision target selected: %i \n \n", cubeColor);
      } else {
        currentCube.width = cube.width;
        currentCube.height = cube.height;
        currentCube.size = cube.width * cube.height;
        currentCube.x = cube.x_middle_coord;
      }

      int direction;  // direction of turning

      // compares targeted cubes to other cubes
      if ((currentCube.size < targetSelection()) && (cubeColor != targetedCube && cubeColor != 0)) {
        pros::vision_object_s_t cube = vision_sensor.get_by_sig(0, cubeColor);
        currentCube.width = cube.width;
        currentCube.height = cube.height;
        currentCube.size = currentCube.width * currentCube.height;
        currentCube.x = cube.x_middle_coord;
        printf("switched to a closer cube %i \n \n", cubeColor);
      }

      // checks if cube still exists
      if (currentCube.size != 0) {
        // direction
        if (currentCube.x > 0) {
          direction = 1;
        } else {
          direction = -1;
        }
        // basic movement
        if (fabs(currentCube.x) > 100) {
          turn_set(20 * direction);
        } else if (currentCube.size >= MAX_CUBE_SIZE) {
          set_drive(0, 0);
        } else if (currentCube.size < MAX_CUBE_SIZE) {
          set_drive(30, 30);
        } /* else if (currentCube.size > MAX_CUBE_SIZE) {
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
        targetedCube = 0;
        clearData(&currentCube);
      }
    }

    pros::delay(20);
  }
}
