# 4549A
> VEX Robotics Code For the Envertronics Team: 4549A

 #### Table of Contents
 
 * [Initialize](#Initialize)
 * [Motors and Sensors](#Motors_and_Sensors)
 * [Opcontrol](#Opcontrol)
 * [PID](#PID)
 * [Angler PID Task](#Angler-PID)
 * [Lift PID Task](#Lif-PID-Task)
 * [LCD Display](#LCD-Display)
 
 
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
 > This is our non redundant, PID function. PID values such as kp, ki, etc. are entered using a constructor inside a struct. These values are used with a target and sensor values to calculate what power the motor should be at that given time. 
 
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

## Angler PID Task
> The angler task uses the PID calculations as well as vector arrays to queue up and switch between different targets. The vector arrays solves the problem where the target is changed while the task is running and it goes to that changed target instead of switching to the new target after the current target has been reached. 

[View Angler](../master/src/angler.cpp)

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

## LCD Display
> The LCD display is custom made and best tailored for our use. It allows us to easily switch between autos and allows us to see values such as sensor values easily. 

< Include screenshot of LCD here > 

[View LCD](../master/src/lcd.cpp)


 
 
