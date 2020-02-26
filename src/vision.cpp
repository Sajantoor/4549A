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

const int DETECTION_THRESHOLD = 500; // area of smallest possible acceptable cube
const int FALSE_POSITIVE_THRESHOLD = 50; // threshold to confirm something has been detected
const int DEEP_VISION_THRESHOLD = 1000; // apply deep detection algorithm if size is above this value and multiple objects detected
const int MIN_DEEP_VISION_THRESHOLD = 500; // min size to be considered to be considered apart of deep vision
const int MAX_SIZE = 45264; // stop before cube gets this big
const int ERROR_X = -32084; // -32084 is the error value for x

data deepVisionData;
data currentCube;

int falsePositiveCheck[3]; // false positive detections for each tracking color, to confirm something is detected a threshold must be reached
int cubeColor = 0; // ID of targeted cube color
int targetedCube = 0; // cube currently being targeted (by ID)


void clearData(data * x) {
  x-> width = 0;
  x-> height = 0;
  x-> size = 0;
  x-> x = 0;
  x -> y = 0;
  x-> deepVisionCheck = 0;
}
// return area of the cube using width and height
int sizeCheck(float x, float width, float height, int id) {
  // no object detected & reached false positive threshold
  if (x != ERROR_X && (falsePositiveCheck[id] > FALSE_POSITIVE_THRESHOLD)) {
    // area of cube in vision sensor units => not actual area
    return width * height;

  // object detection, increment false positive check
  } else if (x != ERROR_X) {
    falsePositiveCheck[id] = falsePositiveCheck[id] + 1;
    return 0;
  } else {
    // no object detected
    falsePositiveCheck[id] = 0;
    return 0;
  }
}

int deepVision(int id) {
  int numObjects = vision_sensor.get_object_count();
  float smallestLeft; // smallest left coordinate
  float largestLeft; // largest
  float largestLeftWidth; // width of largest to calculate the right coordinate
  float largestTop; // largest top coordinate
  float smallestTop; //smallest
  float smallestTopHeight; // height of smallest top to calculate bottom coordinate

  if (numObjects > 3) numObjects = 3;

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
  deepVisionData.y = largestTop - (deepVisionData.height /2);

  if (deepVisionData.size < 0) {
    return 0;
  }

  return deepVisionData.size;
}

// ignore the targeted cube
int targetSelection() {
  int detectionValues[3]; // area of all detection colors
  int closestCube = 0;
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
    // if ((vision_sensor.get_object_count() > 1) && (detectionValues[i] > DEEP_VISION_THRESHOLD)) {
    //   detectionValues[i] = deepVision(i + 1);
    // }

    if (detectionValues[i] > closestCube) {
      closestCube = detectionValues[i];
      cubeColor = i + 1;
    }
  }
  // if the cube is bigger than smallest possible cube, target is selected
  if (closestCube > DETECTION_THRESHOLD) {
    if (closestCube == deepVisionData.size) {
      currentCube = deepVisionData;
    } else {
      clearData(&deepVisionData);
    }
  } else {
    closestCube = 0;
    cubeColor = 0;
    return 0;
  }

  return closestCube;
}

void vision_tracking(void*ignore) {
  // color signatures from VEX Vision Utility
  pros::vision_signature_s_t PURPLE_CUBE = pros::Vision::signature_from_utility(PURPLE, 2039, 3121, 2580, 5781, 7533, 6658, 2.500, 0);
  pros::vision_signature_s_t ORANGE_CUBE = pros::Vision::signature_from_utility(ORANGE, 7153, 9985, 8570, -2527, -2005, -2266, 2.500, 0);
  pros::vision_signature_s_t GREEN_CUBE = pros::Vision::signature_from_utility(GREEN, -6671, -5117, -5894, -4369, -2823, -3596, 2.500, 0);

  // zero point on sensor is the middle
  vision_sensor.set_zero_point(pros::E_VISION_ZERO_CENTER);
  while(true) {
    if (targetedCube == 0) { // any cube 1 - 3 would be true, 0 is false
      // detect all colours of cubes
      int closestCube = targetSelection();
      if (cubeColor != 0) {
        targetedCube = cubeColor;
      }

    } else {
      // targeted cube
      pros::vision_object_s_t trackingCube = vision_sensor.get_by_sig(0, targetedCube);

      if (currentCube.deepVisionCheck) {
        // deepVision(targetedCube);
        currentCube = deepVisionData;
      } else {
        currentCube.width = trackingCube.width;
        currentCube.height = trackingCube.height;
        currentCube.size = trackingCube.width * trackingCube.height;
        currentCube.x = trackingCube.x_middle_coord;
        currentCube.y = trackingCube.y_middle_coord;
     }
      if ((currentCube.size < targetSelection()) && (cubeColor != targetedCube && cubeColor != 0)) {
        pros::vision_object_s_t cube = vision_sensor.get_by_sig(0, cubeColor);
        currentCube.width = cube.width;
        currentCube.height = cube.height;
        currentCube.size = currentCube.width * currentCube.height;
        currentCube.x = cube.x_middle_coord;
      }

      // checks if cube still exists
      if (!currentCube.size > 0) {
        clearData(&currentCube);
        cubeColor = 0;
        targetedCube = 0;
      } else {
        // printf("x: %i \n \n", currentCube.x);
        // printf("size: %i \n \n", currentCube.size);
        // printf("color: %i \n \n", cubeColor);
      }
    }

    pros::delay(20);
  }
}
