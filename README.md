# 4549A
> VEX Robotics Code For the Envertronics Team: 4549A

 #### Table of Contents
 
 * [Initialize](#Initialize)
 * [Motors and Sensors](#motors-and-sensors)
 * [Opcontrol](#Opcontrol)
 * [PID](#PID)
 * [Lift PID Task](#Lift-PID-Task)
 * [Angler PID Task](#angler-pid)
 * [LCD Display](#LCD-Display)
 * [Tracking Task](#Tracking-Task)
 * [Turn PIDs](#Turn-PIDs)
 * [Drive PID](#Drive-PID)
 * [Vision Sensor Cube Tracking](#Vision-Sensor-Cube-Tracking)
 * [Motor Sensor Init](#Motor-Sensor-Init)
 * [Autonomous](#Autonomous)
 
 
 ## Initialize
 > The initalize file is used to define tasks for future use. 

An example of a task definition: 
 ```cpp
 pros::task_t tracking_task = pros::c::task_create(tracking_update, (void*)NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "TRACKING TASK");
 ```

[View Initialize](../master/src/initialize.cpp)
 
 ## Motors and Sensors
 > Motors and sensors are defined in their own file.
 
 An example of a motor definition: 
 ```cpp
 pros::Motor drive_left(DRIVE_LEFT, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
 ```
 
 [View motors and sensors](../master/src/motor_setup.cpp)
 
 ## Opcontrol
 > This is where all the driver related code is. It's designed to be as simple as possible for the driver and requires as few inputs as possible from the driver allowing them to focus on more important aspects of the game.
 
 ```cpp
if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
  angler_pid(1250);
  pros::delay(20);
  angler_pid(2050);
}
 ```
[View opcontrol](../master/src/opcontrol.cpp)

 
This code allows the driver to, at a button press, stack cubes with the angler and the angler returns to its original position. This is a task which allows the driver to do other things such as drive or intake, if they desired. (The values here are potentiometer values.)
 
 ## PID 
 > This is our non redundant, PID function. PID values such as kp, kd, etc. are entered using a constructor inside a struct. These values are used with a target and sensor values to calculate what power the motor should be at that given time. 
 
 ```cpp
typedef struct pid_values {
  float Kp, Kd, Ki, integral_limit, error, last_error, power, integral_active_zone, calc_power;
  float integral, derivative, proportional, target;
  int max_power;
  // constructor
  pid_values(float Kp, float Kd, float Ki, float integral_limit, float integral_active_zone, int max_power):
    Kp(Kp), Kd(Kd), Ki(Ki), integral_limit(integral_limit), integral_active_zone(integral_active_zone), 
    max_power(max_power), error(0), last_error(0), power(0), calc_power(0) {
  }
} pid_values;
```
[View the struct](../master/include/pid.h)


This is the struct with its constructor. It's designed to be as flexible as possible and non redundant. Inside our function we can manipulate different aspects of this struct such as the derivative value if we so desired using the "dot (.)" operator. `<constructor id>.derivative`

 
 ```cpp
 pid_values angler_pid(0.31, 0.07, 0, 30, 500, 70);
 ```
 
This piece of code creates a struct with the id of "angler_pid" using the pid_values constructor (above). These arguments are passed according to the constructor parameters (kp is the first argument within the constuctor, therefore 0.31 will be the kp).
 

```cpp
float pid_calc(pid_values *pid, float target, float sensor) {
   pid->target = target;
   pid->error = pid->target - sensor;
   float derivative = (pid->error - pid->last_error);
   pid->last_error = pid->error;
   float proportional = pid->error;
   float integral = pid->error + integral;

   if (integral*pid->Ki > pid->integral_limit) {
     integral = pid->integral_limit;
   }

   if (integral*pid->Ki < -pid->integral_limit) {
     integral = -pid->integral_limit;
   }

   if (fabs(pid->error) > pid->integral_active_zone) {
     integral = 0;
   }

   pid->calc_power = (proportional*pid->Kp) + (integral*pid->Ki) + (derivative*pid->Kd);
   // calculates power then returns power as max power
   pid->power = power_limit(pid->max_power, pid->calc_power);
   return pid->power;
}
```


This function runs the PID calculations using pointers. It takes in the arguments of the struct (which is called "pid" because of the pointer), target and the sensor we are using, sometimes this is encoders, other times it's potentiometer values. The motor is given the return value of `pid.power`, which is the final power calculated. 

This function is called like this: 
```cpp
float final_power = pid_calc(&lift_pid, height, position);
motor.move(final_power);
```

One of the functions called in the "pid_calc" function is "power_limit". It performs a simple check and returns with a float value. It takes in 2 arguments, "allowed_speed" (max speed) and "actual_speed". It checks whether or not the calculated PID speed is greater than the max speed allowed and returns a value accordingly. 

```cpp
float power_limit(float allowed_speed, float actual_speed) {
   if (actual_speed > allowed_speed) {
       actual_speed = allowed_speed;
   }

   else if (actual_speed < -allowed_speed) {
       actual_speed = -allowed_speed;
   }
   
   return actual_speed;
}
```

[View PID](../master/src/pid.cpp)

## Lift PID Task
> The Lift PID task uses the same PID calculations but it has to overcome another problem. In order for the lift to move, clearance has to be made via the angler moving, therefore the task only runs once the angler has moved to a certain threshold. Another problem is we need it to have a delay in order to drop the cubes into the tower and allowing the driver to position themselves, otherwise the angler drops instantly after the position has been reached due to gravity and the weight of the lift. 

```cpp
  while (true) {
    while ((potentiometer_angler.get_value() < angler_threshold) && liftBool) {
      ...

       if ((fabs(lift_pid.error) < 10) && (currentTime > delayTime)) {
         liftBool = false; // exit out of the loop
         delayTime = 0 // reset delay time
       }
     }
  }
  ```

[View Lift](../master/src/lift.cpp)


## Angler PID Task
> The angler task uses the PID calculations as well as vector arrays to queue up and switch between different targets. The vector arrays solves the problem where the target is changed while the task is running and it goes to that changed target instead of switching to the new target after the current target has been reached. 

```cpp 

      // max torque value is used to calculate how many cubes are in the angler
      if (pros::c::motor_get_torque(11) > maxTorque) {
	maxTorque = pros::c::motor_get_torque(11);
      }

      // slightly reduces the target of a 7 stack to improve accuracy
      if ((maxTorque > SEVEN_STACK_TORQUE) && torqueCheck) {
	currentTarget = currentTarget + 100;
	torqueCheck = false;
      }

      // run motors faster depending on the amount of torque applied on the motor
      // based on the number of cubes, the max power needs to be greater
      if (pros::c::motor_get_torque(11) > 0.7) {
	angler_pid.max_power = 100;
      } else {
	angler_pid.max_power = 80;
      }

      // slows down near the end of the stack
      if (fabs(angler_pid.error) < 600) {
	if (angler_pid.max_power < 40) {
	  angler_pid.max_power = 40;
	} else {
	  // slow down faster for 7 stack or greater
	  if (maxTorque > SEVEN_STACK_TORQUE) {
	    angler_pid.max_power = angler_pid.max_power - 25;
	  } else {
	    angler_pid.max_power = angler_pid.max_power - 15;
	  }
	}
      }
 ``` 
 > Using torque values from the motor, we solved the problem where we weren't able to detect the number of cubes in the angler. Those torque values allow us to stack automatically with an increased degree of accuracy. 

[View Angler](../master/src/angler.cpp)


## LCD Display
> The LCD display is custom made and best tailored for our use. It allows us to easily switch between autos and allows us to see values such as sensor values easily. 

![LCD Start Page](../assets/2019-10-17-072731_480x272_pros_capture.png) 
![LCD Red Auto Selecter](../assets/2019-10-17-072819_480x272_pros_capture.png) 
![LCD Values Page](../assets/2019-10-17-072758_480x272_pros_capture.png) 

[View LCD](../master/src/lcd.cpp)

## Tracking Task
> Another name of this tracking System is Absolute Positioning System (APS). The APS System that keeps track of the absolute position (i.e. Cartesian Coordinates) and the current orientation during Programming Skills and Match Autonomous. The input for this system tracking system are 3 Encoders and the output are the absolute Position and the Orientation.

> This is the major part of calculations done to calculate the X and Y coordinates of the bot and the current Orientation.
```cpp
    if (change_in_angle == 0) {
     local_offset = {inches_traveled_back - prev_inches_traveled_back , inches_traveled_right - prev_inches_traveled_right};
    } else {
      local_offset = { 2 * sin(change_in_angle / 2) * (((inches_traveled_back - prev_inches_traveled_back) / change_in_angle) + 1.266666f),
      2 * sin(change_in_angle/2) * ((inches_traveled_right - prev_inches_traveled_right) / change_in_angle + distance_between_centre)};
    }
    
    float average_orientation = orientation + (change_in_angle/2);
    float rotation_amount = orientation + (change_in_angle)/2;

    polar offset_polar = vector_to_polar(local_offset);
    offset_polar.theta += rotation_amount;
    vector global_offset = polar_to_vector(offset_polar);

    position.x += global_offset.x;
    position.y += global_offset.y;
```
[View Tracking Task](../master/src/drive.cpp)

## Turn PIDs
> These are the many Driving and Turning Pids with input taken from the Tracking Task, for ex. `position.x`, `position.y` and `orientation`. I have 4 sets of turn functions, so I have a turn function to turn a specific degree and another turn function to turn to a specific angle but it is done in a different way. This is the same with my turn fuction to turn a specific coordinate. 

> For turning to a specific angle, I have one turn function that uses normal PID Calculations 
```cpp
      encoder_avg = orientation;
      error = degToRad(target) - encoder_avg;
      derivative = (error - last_error)*kd;
      last_error = error;
      integral = error + integral;
      proportional = error*kp;
      
      if (fabs(error) > (degToRad(22))) integral = 0;//22
      if (integral > integral_limit) integral = integral_limit;
      if (-integral < -integral_limit) integral = -integral_limit;

      final_power = proportional + derivative + (integral * ki);
      turn_set(final_power);
```
> The other turn function that is used to turn to a specific angle is made to use more of the Tracking Values and is alot more accurate. Another difference is that the second turn function has a paramater that we can set to turn either clockwise or anticlockwise, which is why I have a switch statement.
```cpp
  switch (turnDir) {
	case cw:
    	target_angle = orientation + flmod(target_angle - orientation, pi * 2);
    	endFull = orientation * (1 - ratio_full) + target_angle * ratio_full;
      	set_drive(-80,80);

	while (orientation < endFull) {
		pros::delay(10);
	}

      	set_drive(-coast_power, coast_power);

	while (orientation < target_angle - degToRad(stop_offset_deg)) {
		pros::delay(10);
	}

      set_drive(20,-20);
      pros::delay(100);
      set_drive(0,0);
      
      break;
```
> This is for the Counter Clock wise Switch Statement
```cpp
case ccw:
	target_angle = orientation - flmod(orientation - target_angle, pi * 2);
	endFull = orientation * (1 - ratio_full) + target_angle * ratio_full;
	set_drive(80, -80);
	while (orientation > endFull) {
	  pros::delay(10);
	}

	set_drive(coast_power, -coast_power);
	while (orientation > target_angle + degToRad(stop_offset_deg)) {
		pros::delay(10);
	}

	set_drive(-20,20);
	pros::delay(150);
	set_drive(0, 0);

	break;
```
This difference is the same for my turn function that turns the bot to turn towards a certain coordinate.

[View Turning](../master/src/drive.cpp)

## Drive PID
> I made a drive pid that takes in account the X, Y and Orientation to drive to any coordinate with correction. This function uses many concepts from Math like Algebra, Trignometry and even Calculus to find calculations for corrections and supllying power to the motors.
```cpp
if (max_error) {
	err_angle = orientation - line_angle;
	err_x = positionErr.x + positionErr.y * tan(err_angle);
	correctA = atan2(ending_point_x - position.x, ending_point_y - position.y);
	if (max_speed < 0) correctA += pi;
	correction = fabs(err_x) > max_error ? 8.2 * (nearestangle(correctA, orientation) - orientation) * 				sgn(max_speed) : 0;
}
```
> As you can see in the code I use some variables from my Tracking Function. `position.x`, `position.y` and `orientation`. Another important part of my code is the part where I supply power to the motors and put in the correction variable.
```cpp
finalpower = round(-127.0 / 17 * positionErr.y) * sgn(max_speed); //17.9

limit_to_val_set(finalpower, abs(max_speed));
		if (finalpower * sgn(max_speed) < 35) //30
finalpower = 35 * sgn(max_speed);
		int delta = finalpower - last;
		limit_to_val_set(delta, 5);
		finalpower = last += delta;

switch (sgn(correction)) {
	case 0:
		left_drive_set(finalpower);
		right_drive_set(finalpower);
		break;
	case 1:
		left_drive_set(finalpower);
		right_drive_set(finalpower * exp(-correction));
		break;
	case -1:
		left_drive_set(finalpower * exp(correction));
		right_drive_set(finalpower);
		break;
}
```
[View Drive PID](../master/src/drive.cpp#L666)

### Vision Sensor Cube Tracking
> Vision sensor is used to recognize, track and follow cubes. It uses 3 signatures, one for each cube colour and then uses drive functions to follow the cubes. 

```cpp
struct data {
  int width; // effective width of the cube
  int height; // effective height of the cube
  int size; // area, width * height
  int x; // x value
  int y; // y value
  int id; // id for the cube color
  bool deepVisionCheck; // see if the deep vision function is used
};
```

> This code is used for to manage data for the cube using a struct, this data is manipulated in the following functions. 

```cpp
int targetSelection() {
  int detectionValues[3]; // y value of all detection colors
  int closestCube = 0;
  // detecting values
  pros::vision_object_s_t purpleDetection = vision_sensor.get_by_sig(0, PURPLE);
  pros::vision_object_s_t orangeDetection = vision_sensor.get_by_sig(0, ORANGE);
  pros::vision_object_s_t greenDetection = vision_sensor.get_by_sig(0, GREEN);

  // check size of all detections
  detectionValues[0] = greenDetection.y_middle_coord;
  detectionValues[1] = orangeDetection.y_middle_coord;
  detectionValues[2] = purpleDetection.y_middle_coord;
  // gets the closest cube from the array and the color id
  for (size_t i = 0; i < 3; i++) {
    if ((vision_sensor.get_object_count() > 1) && (detectionValues[i] > DEEP_VISION_THRESHOLD)) {
      detectionValues[i] = deepVision(i + 1);
    }

    if (detectionValues[i] > closestCube) {
      closestCube = detectionValues[i];
      cubeColor = i + 1;
    }
  }
  // if the cube is bigger than smallest possible cube, target is selected
  if (closestCube > DETECTION_THRESHOLD) {
    if (closestCube == deepVisionData.y) {
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
``` 
> This code selects a target cube colour to be the closest cube from the for loop and that cubes's coordinates are used in the motion and tracking part of the code. It also has to meet specfic thresholds to see if the cube is close enough to track for example `DETECTION_THRESHOLD`

```cpp
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

  return deepVisionData.y;
}
``` 
> The deep vision function is used for cubes that are darker, when the vision sensor is lacking light. When multiple smaller cubes of the same colour are detected. This uses a geometry to piece together a bigger cube. 

[View vision sensor code](../master/src/vision.cpp)

## Motor Sensor Init
> This is very all the motors, sensors and ports for the motors are defined. 
```cpp
#define DRIVE_LEFT 9
#define pot_port_arm 1
extern pros::Motor drive_left;
pros::Motor drive_left(DRIVE_LEFT, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::ADIPort potentiometer_arm (pot_port_arm, pros::E_ADI_ANALOG_IN);
```
## Autonomous 
> In autonomous, all the PIDs and function are used to make routines for match's autos and Programing Skills. This is also where the LCD is used to help select auto.

```cpp
if (switcher == 1){
 //routines for red auto
}

if (switcher == 2){
 //routines for blue auto
}
``` 
The switcher value is what I change in the LCD to change autos.

[View Autonomous](../master/src/autonomous.cpp)
