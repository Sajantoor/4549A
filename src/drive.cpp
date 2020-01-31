#include "main.h"
#include "drive.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "math.h"
#include "all_used.h"
#include "lift.h"
#include "pid.h"

using namespace pros::literals;

// globals
float correction_turn;
float degrees_flag;
float prev_correction_turn;
float correction_drive;
float prev_correction_drive;
float drive_distance_correction;
float degrees_to_rad_left;
float degrees_to_rad_right;
float orientation;
float beginning_orientation;
float prev_inches_traveled_left;
float prev_inches_traveled_right;
float prev_inches_traveled_back;
vector position;
sVel velocity;

pros::task_t tracking_task;

template <typename T> int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}


//-------------------------------TRACKING VALUES------------------------------------------------------------------------------------------------------------------

polar vector_to_polar(vector v) {
  return{sqrt(powf(v.x,2) + powf(v.y,2)), atan2f(v.x,v.y)};
}

vector polar_to_vector(polar p) {
  return{p.r * sin(p.theta),p.r * cos(p.theta)};
}

void vectorToPolar(vector& vector, polar& polar) {
	if (vector.x || vector.y) {
		polar.r = sqrt(vector.x * vector.x + vector.y * vector.y);
		polar.theta = atan2(vector.y, vector.x);
	} else polar.r = polar.theta = 0;
}

void polarToVector(polar& polar, vector& vector) {
	if (polar.r) {
		vector.x = polar.r * cos(polar.theta);
		vector.y = polar.r * sin(polar.theta);
	} else vector.x = vector.y = 0;
}

void tracking_update(void*ignore) {
  while(true) {
    //gets the ticks from the each of encoders
    float degrees_encoder_left = (left_encoder.get_value());
    float degrees_encoder_right = (right_encoder.get_value());
    float degrees_encoder_back = (back_encoder.get_value());

    //converts each of the encoders ticks to degrees and returns value in radians
    float degrees_to_rad_left = (pi/180) * degrees_encoder_left;
    float degrees_to_rad_right = (pi/180) * degrees_encoder_right;
    float degrees_to_rad_back = (pi/180) * degrees_encoder_back;

    //the radius of the tracking wheels
    const float wheel_radius = 1.3845055;

    //Finds how much each tracking wheel traveled in inches
    float inches_traveled_left = degrees_to_rad_left * wheel_radius; //gives back values in inches
    float inches_traveled_right = degrees_to_rad_right * wheel_radius; //gives back values in inches
    float inches_traveled_back = degrees_to_rad_back * wheel_radius; //gives back values in inches


    const float distance_between_centre = 4.60091565;//1.59437
    const float distance_between_backwheel_center = 4.913425;//2.5

    //Returns the orientation of the bot in radians
    float new_absolute_orientation = beginning_orientation + ((inches_traveled_left - inches_traveled_right)/(2*distance_between_centre));

    //Returns how much it has rotated from its previous point in radians
    float change_in_angle = new_absolute_orientation - orientation;

    // The change in position from previous reset
    vector local_offset;

    //if it did not rotate then it calculates the offset normally
    if (change_in_angle == 0) {
     local_offset = {inches_traveled_back - prev_inches_traveled_back , inches_traveled_right - prev_inches_traveled_right};
    }
    //otherwise calculate the inches traveled according to how much it turned
    else {
      local_offset = { 2 * sin(change_in_angle / 2) * (((inches_traveled_back - prev_inches_traveled_back) / change_in_angle) + distance_between_backwheel_center),
      2 * sin(change_in_angle/2) * ((inches_traveled_right - prev_inches_traveled_right) / change_in_angle + distance_between_centre)};
    }

    float average_orientation = orientation + (change_in_angle/2);
    float rotation_amount = orientation + (change_in_angle)/2;

    //converts local_offset to polar coordinates so that we can rotate by the rotation_amount
    polar offset_polar = vector_to_polar(local_offset);
    offset_polar.theta += rotation_amount;
    vector global_offset = polar_to_vector(offset_polar);
    //converts back to vector coordinates

    //updates the position.x and position.y
    position.x += global_offset.x;
    position.y += global_offset.y;

    //updates orienation values and the inches traveled by the tracking wheels
    orientation = new_absolute_orientation; //gives back value in radians
    prev_inches_traveled_left = inches_traveled_left;
    prev_inches_traveled_right = inches_traveled_right;
    prev_inches_traveled_back = inches_traveled_back;
    pros::delay(10);
  }
}

//tracks the current velocity of the bot
void tracking_velocity(void*ignore) {
  while(true) {
  	unsigned long curTime = pros::millis();
  	long passed = curTime - velocity.lstChecked;

  	if (passed > 40) {
  		float posA = orientation;
  		float posY = position.y;
  		float posX = position.x;
  		velocity.a = ((posA - velocity.lstPosA) * 1000.0) / passed;
  		velocity.y = ((posY - velocity.lstPosY) * 1000.0) / passed;
  		velocity.x = ((posX - velocity.lstPosX) * 1000.0) / passed;
  		velocity.lstPosA = posA;
  		velocity.lstPosY = posY;
  		velocity.lstPosX = posX;
  		velocity.lstChecked = curTime;
  	 }

    pros::delay(10);
  }
}
//------------------------------------------------------------------------------------------------------------------

void drive_line_up (int speed, int run_time_drive) {
  drive_set(speed);
  pros::delay(run_time_drive);
  drive_set(0);
}

void intake_run(int speed_intake, int run_time_intake){
  loader_right.move(speed_intake);
  loader_left.move(speed_intake);
  pros::delay(run_time_intake);
  loader_right.move(0);
  loader_left.move(0);
}

void turn_pid_encoder_average(double target, unsigned int timeout) {
  int ticks_to_deg = 3;
  pid_values turn_pid(60.4, 0, 0, 30, degToRad(10), 110);//59.9
  drive_distance_correction = 0;

  degrees_flag = target*ticks_to_deg;
  int failsafe = 1500;    //2000
  int initial_millis = pros::millis();
  unsigned int net_timer;
  bool timer_turn = true;
  net_timer = pros::millis() + timeout;
  float final_power;
  bool turnBool = true;

//  while(orientation < degToRad(target)) {
  while(turnBool && (pros::millis() < net_timer) && ((initial_millis + failsafe) > pros::millis())) {
    final_power = pid_calc(&turn_pid, degToRad(target), orientation);
    turn_set(final_power);
    printf("finalpower %f \n\n", final_power);
    printf("error %f \n\n", turn_pid.error);

    if (fabs(turn_pid.error) < degToRad(0.5)) turnBool = false;
    if (timer_turn == true) net_timer = pros::millis() + timeout;

    pros::delay(10); //20
  }

  if (final_power > 0) {
    set_drive(20,-20);
    pros::delay(150);
    drive_set(0);
  } else if (final_power < 0) {
    set_drive(-20,20);
    pros::delay(150);
    drive_set(0);
  } else {
    set_drive(0,0);
  }

  correction_drive = (correction_turn + (left_encoder.get_value() - right_encoder.get_value())/2 - degrees_flag);
  prev_correction_turn = correction_turn;
  correction_turn = ((left_encoder.get_value() - right_encoder.get_value())/2 - degrees_flag + prev_correction_turn);///ticks_to_deg;
  turn_pid.error = 0;
  printf("correction turn %f\n", correction_turn);
  printf("encoder avg %d\n", (left_encoder.get_value() + right_encoder.get_value())/2);
}

void drive_pid_encoder(float target, unsigned int timeout, int max_speed) {
  reset_drive_encoders();

  if (pros::competition::is_autonomous()) {
    float Kp_C = 0;
    int failsafe = 2500;    //varible value
    int initial_millis = pros::millis();
    pid_values drive_pid(0.4, 1, 0, 30, (12/(2.75*pi)), max_speed);

    double error_c;
    int direction;
    float left;
    float right;
    int encoder_average;
    int average_left;
    int average_right;
    bool timer_drive = true;			//used to exit out of piD loop after robot reaches error
    float net_timer = pros::millis() + timeout; //just to initialize net_timer at first

      if (target > 0) direction = 1;
      else if (target < 0) direction = -1;

      while(pros::competition::is_autonomous() && (pros::millis() < net_timer) && ((initial_millis + failsafe) > pros::millis())) {
        encoder_average = (right_encoder.get_value() + left_encoder.get_value()) / 2;
        drive_pid.error = ((target)/(2.75*pi) * 360) - drive_distance_correction - encoder_average;

        float final_power = pid_calc(&drive_pid, ((target)/(2.75*pi) * 360) - drive_distance_correction, encoder_average);
        error_c = (left_encoder.get_value() - right_encoder.get_value()) + correction_drive;

        left_drive_set(final_power - error_c*Kp_C);
        right_drive_set(final_power + error_c*Kp_C);

     		if (fabs(drive_pid.error) < (1/(2.75*pi) * 360)){	//less than 1 inches
     			timer_drive = false;		//start timer to to exit piD loop
           drive_pid.integral = 0;
     		}

     		else if (timer_drive){
     			net_timer = pros::millis() + timeout;
     		}

        printf("error %f \n \n", drive_pid.error);

    	pros::delay(20);
    }
  }

  drive_set(0);		//set drive to 0 power

//  printf("encoder avg: %d\n", (left_encoder.get_value() + right_encoder.get_value())/2);
//  pros::lcd::print(7, "encoder avg %d", (left_encoder.get_value() + right_encoder.get_value())/2);


  correction_turn = ((left_encoder.get_value() - right_encoder.get_value())/2 + correction_drive);///ticks_to_deg;
  prev_correction_drive = correction_drive;
  correction_drive = (left_encoder.get_value() - right_encoder.get_value()) + prev_correction_drive;

  drive_distance_correction = ((left_encoder.get_value() + right_encoder.get_value())/2) - ((target)/(4*pi) * 360) + drive_distance_correction;

  printf("correction drive %f\n", correction_drive);
  printf("correction_turn = %1f\n", correction_turn);

  //pros::lcd::print(0, "correction drive %f", correction_drive);
}

//-------------------------------------POSITION PIDS--------------------------------------------------------------------


void position_turn(float target, int timeout, int max_speed) {
    float kp = 160;//75.6
    float kd = 0;
    float ki = 0;
    float proportional, derivative, integral;

    float error;
    float final_power;
    float encoder_avg;
    int last_error = 0;
    int integral_limit = 50;

    //int max_speed = 100;
    bool timer_turn = true;
    unsigned int net_timer;

    int failsafe = timeout;
    int initial_millis = pros::millis();
    net_timer = initial_millis + timeout; //just to initialize net_timer at first

    printf("orientation %f\n", orientation);
    printf("timer %i\n", pros::millis());


    do {
      //do a basic pid loop on orientation with the target as the ending angle
      encoder_avg = orientation;
      error = degToRad(target) - encoder_avg;
      derivative = (error - last_error)*kd;
      last_error = error;
      integral = error + integral;
      proportional = error*kp;

      //pros::lcd::print(0, "orientation %f\n", orientation);
      //pros::lcd::print(1, "error %f\n", error);
      printf("target %f\n", degToRad(target));
      printf("error %f\n", radToDeg(error));
      printf("aoijfeosigo \n\n");

      //integral limeter
      if (fabs(error) > (degToRad(22))) integral = 0;//22
      if (integral > integral_limit) integral = integral_limit;
      if (-integral < -integral_limit) integral = -integral_limit;

      final_power = proportional + derivative + (integral * ki);
      turn_set(final_power);

      if (timer_turn == true) {
        net_timer = pros::millis() + timeout;
      }

      if (fabs(error) < degToRad(1)){
        timer_turn = false;
      }
      pros::delay(20);
    } while(radToDeg(error) < 1 && (pros::millis() < net_timer) && ((initial_millis + failsafe) > pros::millis()));

    HarshStop();

  printf("target %f\n", degToRad(target));
  printf("Degrees Turned from: %f to %f\n", error, orientation);
  printf("Degrees Turned from:%f to %f\n", radToDeg(error), radToDeg(orientation));
 }

//turning to a angle in a different way
void position_turn2(float target_angle, tTurnDir turnDir, float ratio_full, int coast_power, float stop_offset_deg) {
  printf("Turning to %f\n", radToDeg(target_angle));

  //the actual orientation that the bot will end at, calculated based on ratio_full and the starting and ending orientations
  float endFull;

  //decides which way to turn depened on which ever direction is shortest and fastest
  if (turnDir == ch) {
   	if (fmod(target_angle - orientation, pi * 2) > pi) {
      turnDir = ccw;
    } else {
      turnDir = cw;
    }
  }

// the bot keep going at full power until after the full ratio and then it goes at coastPower until stopOffsetDeg
  switch (turnDir) {
	   case cw:
     //calculates the target angle
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

//GOING COUNTER CLOCKWISE

  // This is the code for going counter clockwise
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

      default:
      break;
    }
}

//function to turn to a specific point
void position_face_point2(float target_x, float target_y, tTurnDir turnDir, float ratio_full, float coast_power, float offset, float stopOffsetDeg) {

  //the actual orientation that the bot will end at, calculated based on ratio_full and the starting and ending orientations
  float endFull, target;

  //decides which way to turn depened on which ever direction is shortest and fastest
  	if (turnDir == ch) {
  		if (fmod(atan2(target_x - position.x, target_y - position.y) + offset - orientation, pi * 2) > pi) {
        turnDir = ccw;
      } else {
        turnDir = cw;
      }
    }

    switch (turnDir) {
    	case cw:
        //calculation of the target dependent on our current position
      	target = orientation + flmod(atan2(target_x - position.x, target_y - position.y) + offset - orientation, pi * 2);
        endFull = orientation * (1 - ratio_full) + target * ratio_full;

    		set_drive(-80, 80);
    		while (orientation < endFull) {
    			pros::delay(10);
    		}

    		set_drive(-coast_power, coast_power);

    		while (orientation < nearestangle(atan2(target_x - position.x, target_y - position.y) + offset, target) - degToRad(stopOffsetDeg) /*&& (velSafety? NOT_SAFETY(power, turnToTargetNewAlg) : 1 )*/) {
    			pros::delay(10);
    		}

    		set_drive(20, -20);
    		pros::delay(150);
    		set_drive(0, 0);

    		break;

        //For going counter clockwise
  	case ccw:
      target = orientation - fmod(orientation - atan2(target_x - position.x, target_y - position.y) - offset, pi * 2);
      endFull = orientation * (1 - ratio_full) + (target) * ratio_full;
  	//	if (LOGS) writeDebugStreamLine("%f %f", radToDeg(target), radToDeg(endFull));

  		set_drive(80, -80);

  		while (orientation > endFull /*&& (velSafety? NOT_SAFETY(power, turnToTargetNewAlg) : 1 )*/) {
  			pros::delay(10);
  		}

  		set_drive(coast_power, -coast_power);

      while (orientation > nearestangle(atan2(target_x - position.x, target_y - position.y) + offset, target) + degToRad(stopOffsetDeg)) {
  			pros::delay(10);
  		}
  		//if (LOGS) writeDebugStreamLine("Turn done: %d",  orientation);

  			set_drive(-20, 20);
  			pros::delay(150);
  			//if (LOGS) writeDebugStreamLine("Break done: %d",  orientation);

  		set_drive(0, 0);

      printf("done \n");
  		break;

      default:
      break;
  	}
}

//same principle as position_face_point2 but using pid loops
void position_face_point(float target_x, float target_y,int timeout) {
    vector error;
    float Kp = 77; //0.2

    float kd = 0.5;//70   3
    float ki = 0;//0.06   0
    float proportional, derivative, integral;
    float final_power;
    float encoder_avg;
    int last_error = 0;
    int integral_limit = 50;

    int max_speed = 110; //90
    float max_error = 0.001f;
    bool timer_turn = true;
    unsigned int net_timer;

    int failsafe = 2000;
    int initial_millis = pros::millis();

    float direction_face;

    float encoder_average;
    float error_p;

    while(pros::competition::is_autonomous() && (pros::millis() < net_timer) && ((initial_millis + failsafe) > pros::millis())) {
      error.x = target_x - position.x;
      error.y = target_y - position.y;

      //what direction to face (target)
      direction_face = atan2f(error.y , error.x);

      encoder_avg = orientation;
      error_p = direction_face - encoder_avg;
      derivative = (error_p - last_error)*kd;
      last_error = error_p;
      integral = (error_p + integral);
      proportional = error_p*Kp;

      // start integral when target is less than 22 degrees
      if (fabs(error_p) > degToRad(22)){ integral = 0; }

      if (integral > integral_limit){ integral = integral_limit; }

      if (-integral < -integral_limit){ integral = -integral_limit; }


      final_power = proportional + derivative + (integral * ki);

      //pros::lcd::print(6, "final_power %f\n", final_power);

      turn_set(final_power);

      printf("direction_face %f\n", direction_face);
      printf(" \n");
      printf("error_p %f\n", error_p);
      printf(" \n");

      if (timer_turn == true) {
        net_timer = pros::millis() + timeout;
      }

      if (fabs(error_p) < degToRad(1)) {
        timer_turn = false;
      }
      pros::delay(20);
  }

    HarshStop();
    printf("target %f\n", radToDeg(direction_face));
    printf("target radians %f\n", direction_face);
    printf("Degrees Turned from: %f to %f\n", error_p, orientation);
    printf("Degrees Turned from:%f to %f\n", radToDeg(error_p), radToDeg(orientation));
}

void position_drive(float ending_point_x, float ending_point_y, float target_angle, bool cool_turn, float max_power, unsigned int timeout, bool pickUp_cube, float initial_intake, float final_intake, float transition_point, float end_speed_transition, float end_speed) {
    vector positionErr;
    vector rotated_motorPower;
    vector rotation_vector;
    vector delta_main_line;
    polar positionErrPolar;
    polar rotated_motorPowerPolar;

    pid_values turn_pid(300, 3, 6, 30, 500, 127);//300
    pid_values xDir_pid(28, 0, 0, 30, 500, 127);//28
    pid_values yDir_pid(11, 8, 0, 30, 500, 127);//12,8

    if(cool_turn) {
      turn_pid.Kp = 85;
      xDir_pid.Kp = 40;
      yDir_pid.Kp = 19;
    }

    //timeout on the code so that if it ever gets stuck in the while loop it exits after a certain amount of time
    unsigned int net_timer;
    int initial_millis = pros::millis();
    net_timer = initial_millis + timeout; //just to initialize net_timer at first
    int last_y = 0;
    int last_x = 0;

    bool timer_drive = true;

    float powf_of_X_Y;
    float magnitude_of_X_Y;
    float drive_left_power;
    float drive_left_b_power;
    float drive_right_power;
    float drive_right_b_power;
    float largestVal = 0;
    float intakeSpeed = initial_intake;
    printf("Moving to %f %f \n", ending_point_x, ending_point_y);

    do {
      largestVal = 0;
      // intake speed transition
      // transition point is the magnitude x, y error away intakes transition speeds
      if ((magnitude_of_X_Y < transition_point) && (transition_point != 0)) {
        intakeSpeed = final_intake;
      }
      //runs pid loops on the position.x and position.y and orienation
      float final_power_turn = pid_calc(&turn_pid, degToRad(target_angle), orientation);
      float final_power_xDir = pid_calc(&xDir_pid, ending_point_x, position.x);
      float final_power_yDir = pid_calc(&yDir_pid, ending_point_y, position.y);

      //assigns the final_power_strafe and throttle to a vector
      rotated_motorPower.y = final_power_yDir;
      rotated_motorPower.x = final_power_xDir;

      //The vector then is rotated so the frame of reference is robot centric besides field centric
      vectorToPolar(rotated_motorPower, rotated_motorPowerPolar);
      rotated_motorPowerPolar.theta += orientation;
      polarToVector(rotated_motorPowerPolar, rotated_motorPower);
			printf("position.x %f \n", position.x);
		  printf("position.y %f \n", position.y);

      if(pickUp_cube && light_sensor_intake.get_value() < 1900){
        loader_left.move(0);
        loader_right.move(0);
      }
      //applying slew rate on the motors so they dont burn out and there arent sudden movements
      if ((magnitude_of_X_Y < end_speed_transition) && (end_speed_transition != 0)) {
        limit_to_val_set(rotated_motorPower.y, abs(end_speed));
        int delta_y = rotated_motorPower.y - last_y;
        limit_to_val_set(delta_y, 3);
        rotated_motorPower.y = last_y += delta_y;

        limit_to_val_set(rotated_motorPower.x, abs(end_speed));
        int delta_x = rotated_motorPower.x - last_x;
        limit_to_val_set(delta_x, 5);
        rotated_motorPower.x = last_x += delta_x;
      }
      else {
        limit_to_val_set(rotated_motorPower.y, abs(max_power));
        int delta_y = rotated_motorPower.y - last_y;
        limit_to_val_set(delta_y, 3);
        rotated_motorPower.y = last_y += delta_y;

        limit_to_val_set(rotated_motorPower.x, abs(max_power));
        int delta_x = rotated_motorPower.x - last_x;
        limit_to_val_set(delta_x, 5);
        rotated_motorPower.x = last_x += delta_x;
      }
      drive_left_power = rotated_motorPower.y + final_power_turn + rotated_motorPower.x;
      drive_left_b_power = rotated_motorPower.y + final_power_turn - rotated_motorPower.x;
      drive_right_power = rotated_motorPower.y - final_power_turn - rotated_motorPower.x;
      drive_right_b_power = rotated_motorPower.y - final_power_turn + rotated_motorPower.x;


      float motor_power_array [4] = {drive_left_power, drive_left_b_power, drive_right_power, drive_right_b_power};

      //assigns largestVal to the highest value of the motor powers
      for (size_t i = 0; i < 4; i++) {
        if (abs(motor_power_array[i]) > largestVal) {
          largestVal = abs(motor_power_array[i]);
        }
      }
      printf("largestVal %f \n", largestVal);
      //Scales down all the motor_power if the largestVal is over 127, this is to make sure the motors arent getting power over 127
        if (largestVal > 127) {
          motor_power_array[0] = (motor_power_array[0] * 127) / abs(largestVal);
          motor_power_array[1] = (motor_power_array[1] * 127) / abs(largestVal);
          motor_power_array[2] = (motor_power_array[2] * 127) / abs(largestVal);
          motor_power_array[3] = (motor_power_array[3] * 127) / abs(largestVal);
        }

      //applies power to the motors in mecanum formation
      drive_left.move(motor_power_array [0]);
      drive_left_b.move(motor_power_array [1]);
      drive_right.move(motor_power_array [2]);
      drive_right_b.move(motor_power_array [3]);
      loader_right.move(intakeSpeed);
      loader_left.move(intakeSpeed);

      //this gets the magnitude of the error using the error from throttle and strafe
      powf_of_X_Y = powf(yDir_pid.error, 2) + powf(xDir_pid.error, 2);
      magnitude_of_X_Y = sqrtf(powf_of_X_Y);

			printf("magnitude_of_X_Y %f \n", magnitude_of_X_Y);
      pros::delay(10);

    } while ((magnitude_of_X_Y > 1 || abs(radToDeg(turn_pid.error)) > 1) && (pros::millis() < net_timer));

    //applies harsh stop depending on how fast the robot was moving
    HarshStop();
    loader_left.move(0);
    loader_right.move(0);
    printf("driving done\n");
}


          //   void sweep_turn(float x, float y, float end_angle, float arc_radius, tTurnDir turnDir, float max_speed) {
          //     vector Vector;
          //     polar Polar;
          //
          //     if(turnDir == ch){
          //       Vector.x = position.x - x;
          //       Vector.y = position.y - y;
          //       vectorToPolar(Vector, Polar);
          //       Polar.theta += end_angle;
          //       polarToVector(Polar, Vector);
          //
          //       turnDir = Vector.x > 0 ? cw : ccw;
          //     }
          //
          //     float yOrigin, xOrigin;
          //     float linearV, angularV, angularVLast = 0;
          //     float localR, localA;
          //
          //     const float kR = 15.0;
          //     const float kA = 5.0;
          //     const float kB = 60.0;
          //     const float kP = 30.0;
          //     const float kD = 2000.0;
          //
          //     switch (turnDir){
          //
          //       case cw:
          //       Vector.y = 0;
          //       Vector.x = arc_radius;
          //       vectorToPolar(Vector, Polar);
          //       Polar.theta -= end_angle;
          //       polarToVector(Polar, Vector);
          //       yOrigin = y + Vector.y;
          //       xOrigin = x + Vector.x;
          //
          //       localA = atan2(position.x - xOrigin, position.y - yOrigin);
          //       end_angle = nearestangle(end_angle, max_speed > 0 ? orientation : (orientation + pi));
          //
          //       do
          //       {
          //       float aGlobal = orientation;
          //       if (max_speed < 0)
          //       aGlobal += pi;
          //       angularV = velocity.a;
          //       float _y = position.y - yOrigin;
          //       float _x = position.x - xOrigin;
          //       localR = sqrt(_y * _y + _x * _x);
          //       localA = nearestangle(atan2(_x, _y), localA);
          //       linearV = velocity.x * sin(localA + pi / 2) + velocity.y * cos(localA + pi / 2);
          //
          //       float target = MAX(linearV, 15) / localR + kR * log(localR / arc_radius) + kA * (nearestangle(localA + pi / 2, aGlobal) - aGlobal);
          //       int turn = round(kB * target + kP * (target - angularV) + kD * (angularVLast - angularV) / 40);
          //       angularVLast = angularV;
          //
          //       if (turn < 0) {
          //         turn = 0;
          //       }
          //       else if (turn > 150) {
          //         turn = 150;
          //       }
          //
          //       if (max_speed > 0) {
          //         set_drive(max_speed, max_speed - turn);
          //       }
          //       else {
          //         set_drive(max_speed + turn, max_speed);
          //       }
          //       pros::delay(10);
          //     } while ((max_speed > 0 ? orientation : (orientation + pi)) - end_angle);
          //
          //       case ccw:
          //       Vector.y = 0;
          //       Vector.x = arc_radius;
          //       vectorToPolar(Vector, Polar);
          //       Polar.theta += end_angle;
          //       polarToVector(Polar, Vector);
          //       yOrigin = y + Vector.y;
          //       xOrigin = x + Vector.x;
          //
          //       localA = atan2(position.x - xOrigin, position.y - yOrigin);
          //       end_angle = nearestangle(end_angle, max_speed > 0 ? orientation : (orientation + pi));
          //
          //       do
          //       {
          //       float aGlobal = orientation;
          //       if (max_speed < 0)
          //       aGlobal += pi;
          //       angularV = velocity.a;
          //       float _y = position.y - yOrigin;
          //       float _x = position.x - xOrigin;
          //       localR = sqrt(_y * _y + _x * _x);
          //       localA = nearestangle(atan2(_x, _y), localA);
          //       linearV = velocity.x * sin(localA - pi / 2) + velocity.y * cos(localA - pi / 2);
          //
          //       float target = -MAX(linearV, 15) / localR + kR * log(localR / arc_radius) + kA * (nearestangle(localA - pi / 2, aGlobal) - aGlobal);
          //       int turn = round(kB * target + kP * (target - angularV) + kD * (angularVLast - angularV) / 40);
          //       angularVLast = angularV;
          //
          //       if (turn < 0)
          //       turn = 0;
          //       else if (turn > -150)
          //       turn = -150;
          //
          //       if (max_speed > 0)
          //       set_drive(max_speed + turn, max_speed);
          //       else
          //       set_drive(max_speed, max_speed - turn);
          //       pros::delay(10);
          //     } while ((max_speed > 0 ? orientation : (orientation + pi)) - end_angle);
          //
          //       default:
          //       break;
          //     }
          //     set_drive(0,0);
          // }
