#include "main.h"
#include "drive.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "math.h"
#include "all_used.h"
#include "lift.h"
#include "pid.h"


using namespace pros::literals;

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
    // float ticks_to_degs = 3.18627451;
    //float pi = 3.1415926535;
    float degrees_encoder_left = (left_encoder.get_value());//* ticks_to_degs
    float degrees_encoder_right = (right_encoder.get_value());//* ticks_to_degs
    float degrees_encoder_back = (back_encoder.get_value());//* ticks_to_degs

    float degrees_to_rad_left = (pi/180) * degrees_encoder_left; //gives back values in radians
    float degrees_to_rad_right = (pi/180) * degrees_encoder_right; //gives back values in radians
    float degrees_to_rad_back = (pi/180) * degrees_encoder_back; //gives back values in radians

    const float wheel_radius = 1.3845055; //1.4545

    float inches_traveled_left = degrees_to_rad_left * wheel_radius; //gives back values in inches
    float inches_traveled_right = degrees_to_rad_right * wheel_radius; //gives back values in inches
    float inches_traveled_back = degrees_to_rad_back * wheel_radius; //gives back values in inches

    const float distance_between_centre = 5.3365377;//5.49380807
    const float distance_between_backwheel_center = 4.15;//-2.0254878
    //CORDINATES facing the enemies side is 𝜃r = 0

    //beginning_orientation = 0;

    float new_absolute_orientation = beginning_orientation + ((inches_traveled_left - inches_traveled_right)/(2*distance_between_centre)); // gives back values in radians and gives us the orientation of the bot

    float change_in_angle = new_absolute_orientation - orientation; // gives back value in radians and also how much it has rotated from its previous point

    vector local_offset;

    if (change_in_angle == 0) {
     local_offset = {inches_traveled_back - prev_inches_traveled_back , inches_traveled_right - prev_inches_traveled_right};
    } else {
      local_offset = { 2 * sin(change_in_angle / 2) * (((inches_traveled_back - prev_inches_traveled_back) / change_in_angle) + distance_between_backwheel_center),//1.266666  3.54331
      2 * sin(change_in_angle/2) * ((inches_traveled_right - prev_inches_traveled_right) / change_in_angle + distance_between_centre)};
    }

    // printf("local_offset %f, %f \n \n", local_offset.x, local_offset.y);
    // printf("change_in_angle %f \n \n", change_in_angle);
    // printf("inches_traveled_left %f \n \n", inches_traveled_left);
    // printf("inches_traveled_right %f \n \n", inches_traveled_right);
    // printf("inches_traveled_back %f \n \n", inches_traveled_back);

    float average_orientation = orientation + (change_in_angle/2);
    float rotation_amount = orientation + (change_in_angle)/2;

    polar offset_polar = vector_to_polar(local_offset);
    offset_polar.theta += rotation_amount;
    vector global_offset = polar_to_vector(offset_polar);

    position.x += global_offset.x;
    position.y += global_offset.y;


    orientation = new_absolute_orientation; //gives back value in radians
    prev_inches_traveled_left = inches_traveled_left;
    prev_inches_traveled_right = inches_traveled_right;
    prev_inches_traveled_back = inches_traveled_back;
    pros::delay(10);
  }
}

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
  int failsafe = 2500;    //2000
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
    float kp = 55.5;//75.6
    float kd = 0.1;
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

    int failsafe = 1500;
    int initial_millis = pros::millis();
    printf("orientation %f\n", orientation);
    printf("timer %i\n", pros::millis());


    while(pros::competition::is_autonomous() && (pros::millis() < net_timer) && ((initial_millis + failsafe) > pros::millis())) {
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


      if (fabs(error) > (degToRad(22))) integral = 0;//22
      if (integral > integral_limit) integral = integral_limit;
      if (-integral < -integral_limit) integral = -integral_limit;

      final_power = proportional + derivative + (integral * ki);
      turn_set(final_power);

      //pros::lcd::print(2, "final_power %f\n", final_power);
      printf("position.x %f\n", position.x);
      printf(" \n");
      printf("position.y %f\n", position.y);
      printf(" \n");
      printf("finalpower %f\n", final_power);
      printf(" \n");

      if (timer_turn == true) {
        net_timer = pros::millis() + timeout;
      }

      if (fabs(error) < degToRad(1)){
        timer_turn = false;
      }
      pros::delay(20);
    }

  if (final_power > 0) {
    set_drive(20,-20);
    pros::delay(100);
    drive_set(0);
  } else if (final_power < 0) {
    set_drive(-20,20);
    pros::delay(100);
    drive_set(0);
  } else {
    set_drive(0,0);
  }

  printf("target %f\n", degToRad(target));
  printf("Degrees Turned from: %f to %f\n", error, orientation);
  printf("Degrees Turned from:%f to %f\n", radToDeg(error), radToDeg(orientation));
 }

void position_turn2(float target_angle, tTurnDir turnDir, float ratio_full, int coast_power, float stop_offset_deg) {
  printf("Turning to %f\n", radToDeg(target_angle));
  float endFull;

  if (turnDir == ch) {
   	if (fmod(target_angle - orientation, pi * 2) > pi) {
      turnDir = ccw;
    } else {
      turnDir = cw;
    }
  }

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

      printf("done \n");
      printf("Moving to %f at %f \n", target_angle, radToDeg(orientation));
      printf("ENTERED SECOND STAGE \n");
      printf(" \n");
      printf("position.x %f\n", position.x);
      printf(" \n");
      printf("position.y %f\n", position.y);
      printf(" \n");
      printf("target_angle %f\n", target_angle);
      printf(" \n");
      printf("endFull %f\n", endFull);
      printf(" \n");
      printf("orientation %f\n", orientation);
      printf(" \n");
      printf("Moving to %f at %f \n", target_angle, radToDeg(orientation));
      printf(" \n");
      printf("Turning to %f\n", radToDeg(target_angle));
      //pros::lcd::print(6,"orientation %f\n", orientation);
      printf("DONE \n");
      printf("--------------------------------------------------------------------------------------\n");
      printf(" \n");
      //pros::lcd::print(7," Moving to %f at %f \n", target_angle,radToDeg(orientation));
      break;

//GOING COUNTER CLOCKWISE

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

      printf("Moving to %f at %f \n", target_angle, radToDeg(orientation));
      printf("FINISHED TURNING \n");
      printf(" \n");
      printf("position.x %f\n", position.x);
      printf(" \n");
      printf("position.y %f\n", position.y);
      printf(" \n");
      printf("orientation %f\n", orientation);
      printf(" \n");
      printf("target_angle %f\n", target_angle);
      printf(" \n");
      printf("endFull %f\n", endFull);
      printf(" \n");
      printf("Moving to %f at %f \n", target_angle, radToDeg(orientation));
      printf(" \n");
      printf("Turning to %f\n", radToDeg(target_angle));
      //pros::lcd::print(6,"orientation %f\n", radToDeg(orientation));
      printf("--------------------------------------------------------------------------------------\n");
      printf(" \n");
      printf("done \n");

      break;

      default:
      break;
    }
}

void position_face_point2(float target_x, float target_y, tTurnDir turnDir, float ratio_full, float coast_power, float offset, float stopOffsetDeg) {
	float endFull, target;
  	if (turnDir == ch) {
  		if (fmod(atan2(target_x - position.x, target_y - position.y) + offset - orientation, pi * 2) > pi) {
        turnDir = ccw;
      } else {
        turnDir = cw;
      }
    }

    switch (turnDir) {
    	case cw:
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

        printf("Moving to %f at %f \n", target, radToDeg(orientation));
        printf("ENTERED SECOND STAGE \n");
        printf(" \n");
        printf("position.x %f\n", position.x);
        printf(" \n");
        printf("position.y %f\n", position.y);
        printf(" \n");
        printf("orientation %f\n", orientation);
        printf(" \n");
        printf("target_x %f\n", target_x);
        printf(" \n");
        printf("target_y %f\n", target_y);
        printf(" \n");
        printf("target %f\n", target);
        printf(" \n");
        printf("endFull %f\n", endFull);
        printf(" \n");
        printf("Moving to %f at %f \n", target, radToDeg(orientation));
        printf(" \n");
        printf("Turning to %f\n", radToDeg(target));
        //pros::lcd::print(6,"orientation %f\n", radToDeg(orientation));
        printf("--------------------------------------------------------------------------------------\n");
        printf(" \n");
        printf("done \n");

    		break;


  	case ccw:
      target = orientation - fmod(orientation - atan2(target_x - position.x, target_y - position.y) - offset, pi * 2);
      endFull = orientation * (1 - ratio_full) + (target) * ratio_full;
  	//	if (LOGS) writeDebugStreamLine("%f %f", radToDeg(target), radToDeg(endFull));

  		set_drive(80, -80);

  		while (orientation > endFull /*&& (velSafety? NOT_SAFETY(power, turnToTargetNewAlg) : 1 )*/) {
        printf("FIRST STAGE \n");
        printf(" \n");
        printf("position.x %f\n", position.x);
        printf(" \n");
        printf("position.y %f\n", position.y);
        printf(" \n");
        printf("orientation %f\n", orientation);
        printf(" \n");
        printf("target_x %f\n", target_x);
        printf(" \n");
        printf("target_y %f\n", target_y);
        printf(" \n");
        printf("target %f\n", target);
        printf(" \n");
        printf("endFull %f\n", endFull);
        printf(" \n");
        printf("Moving to %f at %f \n", target, orientation);
        printf(" \n");
        printf("--------------------------------------------------------------------------------------\n");
        printf(" \n");
  			pros::delay(10);
  		}

  		set_drive(coast_power, -coast_power);

      while (orientation > nearestangle(atan2(target_x - position.x, target_y - position.y) + offset, target) + degToRad(stopOffsetDeg)) {
        printf("SECOND STAGE \n");
        printf(" \n");
        printf("position.x %f\n", position.x);
        printf(" \n");
        printf("position.y %f\n", position.y);
        printf(" \n");
        printf("orientation %f\n", orientation);
        printf(" \n");
        printf("target_x %f\n", target_x);
        printf(" \n");
        printf("target_y %f\n", target_y);
        printf(" \n");
        printf("target %f\n", target);
        printf(" \n");
        printf("endFull %f\n", endFull);
        printf(" \n");
        printf("Moving to %f at %f \n", target, orientation);
        printf(" \n");
        printf("--------------------------------------------------------------------------------------\n");
        printf(" \n");
  			pros::delay(10);
  		}
  		//if (LOGS) writeDebugStreamLine("Turn done: %d",  orientation);

  			set_drive(-20, 20);
  			pros::delay(150);
  			//if (LOGS) writeDebugStreamLine("Break done: %d",  orientation);

  		set_drive(0, 0);

      printf("Moving to %f at %f \n", target, radToDeg(orientation));
      printf("ENTERED SECOND STAGE \n");
      printf(" \n");
      printf("position.x %f\n", position.x);
      printf(" \n");
      printf("position.y %f\n", position.y);
      printf(" \n");
      printf("orientation %f\n", orientation);
      printf(" \n");
      printf("target_angle %f\n", target);
      printf(" \n");
      printf("target_x %f\n", target_x);
      printf(" \n");
      printf("target_y %f\n", target_y);
      printf(" \n");
      printf("endFull %f\n", endFull);
      printf(" \n");
      printf("Moving to %f at %f \n", target, radToDeg(orientation));
      printf(" \n");
      printf("Turning to %f\n", radToDeg(target));
      //pros::lcd::print(6,"orientation %f\n", radToDeg(orientation));
      printf("--------------------------------------------------------------------------------------\n");
      printf(" \n");
      printf("done \n");
  		break;

      default:
      break;
  	}
}

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

   if (final_power > 0) {
     set_drive(20,-20);
     pros::delay(95);
     drive_set(0);
   }  else if (final_power < 0) {
     set_drive(-20,20);
     pros::delay(95);
     drive_set(0);
   } else {
     set_drive(0,0);
   }

   turn_set(0);
    printf("target %f\n", radToDeg(direction_face));
    printf("target radians %f\n", direction_face);
    printf("Degrees Turned from: %f to %f\n", error_p, orientation);
    printf("Degrees Turned from:%f to %f\n", radToDeg(error_p), radToDeg(orientation));
}

void position_drive(float starting_point_x, float starting_point_y, float ending_point_x, float ending_point_y, int startpower, float max_speed, float max_error, int early_stop, float timeout, float look_ahead_distance) {
    vector error;
    vector positionErr;
    vector rotation_vector;
    vector delta_main_line;
    vector rotated_main_line;
    vector look_ahead_point;
    polar look_ahead_point_polar;
    polar positionErrPolar;

    unsigned int net_timer;
    int initial_millis = pros::millis();
    net_timer = initial_millis + timeout; //just to initialize net_timer at first
    float failsafe = 2000;

    float magnPosvector;
    float angle_main_line;
    //float line_ahead_point = 0.5;
    float line_point_angle;
    float line_angle;
    float correction = 0;
    float finalpower;
    float err_angle;
    float err_x;
    float correctA;
    int last = startpower;
    float line_length;
    float sin_line;
    float cos_line;
    float velocity_line;
    printf("Moving to %f %f from %f %f at %f \n", ending_point_x, ending_point_y, starting_point_x, starting_point_y, max_speed);
    delta_main_line.x = ending_point_x - starting_point_x;
    delta_main_line.y = ending_point_y - starting_point_y;

    do {
      look_ahead_point.x = 0;
      look_ahead_point.y = positionErr.y + look_ahead_distance;
      vectorToPolar(look_ahead_point, look_ahead_point_polar);
      look_ahead_point_polar.theta -= angle_main_line;
      polarToVector(look_ahead_point_polar, look_ahead_point);
      look_ahead_point.x += ending_point_x;
      look_ahead_point.y += ending_point_y;

      positionErr.x = position.x - ending_point_x;
      positionErr.y = position.y - ending_point_y;
      angle_main_line = atan2f(delta_main_line.x, delta_main_line.y);
      line_angle = nearestangle(angle_main_line - (max_speed < 0 ? pi : 0), orientation);
      line_length = powf(positionErr.x , 2) + powf(positionErr.y , 2);
      magnPosvector = sqrt(line_length);
      sin_line = sin(angle_main_line);
      cos_line = cos(angle_main_line);
      vectorToPolar(positionErr, positionErrPolar);
      positionErrPolar.theta += angle_main_line;
      polarToVector(positionErrPolar, positionErr);

      if (max_error) {
  			err_angle = orientation - line_angle;
  			err_x = positionErr.x + positionErr.y * tan(err_angle);
  			correctA = atan2(look_ahead_point.x - position.x, look_ahead_point.y - position.y);
  			if (max_speed < 0)
  				correctA += pi;
  			correction = fabs(err_x) > max_error ? 6 * (nearestangle(correctA, orientation) - orientation) * sgn(max_speed) : 0; //5.7
        printf(" \n");//5.3
      }

    //------------------------------------------------------------math--------------------------------------------------------

      finalpower = round(-127.0 / 30 * positionErr.y) * sgn(max_speed); //17

      limit_to_val_set(finalpower, abs(max_speed));
			if (finalpower * sgn(max_speed) < 30) //30
      finalpower = 30 * sgn(max_speed);
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

        // printf("back_encoder %d\n", back_encoder.get_value());
        // printf(" \n");
        // printf("left_encoder %d\n", left_encoder.get_value());
        // printf(" \n");
        // printf("right_encoder %d\n", right_encoder.get_value());
        // printf(" \n");
        printf("position.x %f\n", position.x);
        printf(" \n");
        printf("position.y %f\n", position.y);
        printf(" \n");
        printf("positionErr.x %f\n", positionErr.x);
        printf(" \n");
        printf("positionErr.y %f\n", positionErr.y);
        printf(" \n");
        // printf("final_power %f\n", finalpower);
        // printf(" \n");
        printf("err_angle %f\n", err_angle);
        printf(" \n");
        printf("err_x %f\n", err_x);
        printf(" \n");
        printf("look_ahead_point.x %f\n", look_ahead_point.x);
        printf(" \n");
        printf("look_ahead_point.y %f\n", look_ahead_point.y);
        printf(" \n");
        printf("orientation %f\n", orientation);
        printf(" \n");
        printf("correctA %f\n", correctA);
        printf(" \n");
        printf("correction %f\n", correction);
        printf(" \n");
        // printf("max_error %f\n", max_error);
        // printf(" \n");
        // printf("orientation %f\n", orientation);
        // printf(" \n");
        // printf("sgn(max_speed) %d\n", sgn(max_speed));
        // printf(" \n");
        // printf("tan(err_angle) %f \n", tan(err_angle));
        // printf(" \n");
        // printf("exp(correction) %f \n", exp(correction));
        // printf(" \n");
        // printf("last finalpower %d \n", last);
        // printf(" \n");
        // printf("delta %d \n", delta);
        // printf(" \n");
        // printf("magnPosvector %f\n", magnPosvector);
        // printf(" \n");
        // printf("line_length %f\n", line_length);
        // printf(" \n");
        // printf("positionErrPolar %f\n", positionErrPolar.theta);
        // printf(" \n");
        // printf("Line Angle %f\n", radToDeg(line_angle));
        // printf(" \n");
        // printf("angle_main_line %f\n", radToDeg(angle_main_line));
        // printf(" \n");
        // printf("Moving to %f , %f from %f , %f at %f \n", ending_point_x, ending_point_y, starting_point_x, starting_point_y, max_speed);
        // printf(" \n");
        // printf("Moved to %f %f from %f %f at %f  || %f.x , %f.y , %f\n", ending_point_x, ending_point_y, starting_point_x, starting_point_y, max_speed, position.x, position.y, radToDeg(orientation));
        // printf(" \n");
        // printf("--------------------------------------------------------------------------------------\n");
        // printf(" \n");

        pros::delay(10);

      } while (positionErr.y < -early_stop && (pros::millis() < net_timer) && ((initial_millis + failsafe) > pros::millis()));

      drive_set(25 * sgn(max_speed));

    do {
      positionErr.x = position.x - ending_point_x;
      positionErr.y = position.y - ending_point_y;

      vectorToPolar(positionErr, positionErrPolar);
      positionErrPolar.theta += angle_main_line;
      polarToVector(positionErrPolar, positionErr);

      velocity_line = sin_line * velocity.x + cos_line * velocity.y;
      //printf("driving done velocity\n");
      // printf("position.x %f\n", position.x);
      // printf(" \n");
      // printf("position.y %f\n", position.y);
      // printf(" \n");

      pros::delay(5);
    } while((positionErr.y < -early_stop - (velocity_line * 0.098)) && (pros::millis() < net_timer));

    printf("driving done\n");
    printf("velocity_line %f \n", velocity_line);

    if (max_speed < 0) {
      drive_set(20);
      pros::delay(50);
      drive_set(0);
      printf("driving done back\n");

    } else if (max_speed > 0) {
      drive_set(-20);
      pros::delay(50);
      drive_set(0);
      printf("driving done forward\n");

    } else {
      drive_set(0);
    }
    // drive_set(0);
    printf("driving done\n");
}

void position_drive2(float starting_point_x, float starting_point_y, float ending_point_x, float ending_point_y, float target_angle, float max_power) {
    vector positionErr;
    vector rotation_vector;
    vector delta_main_line;
    polar positionErrPolar;

    pid_values turn_pid(0, 0, 0, 30, 500, max_power);
    pid_values strafe_pid(0, 0, 0, 30, 500, max_power);
    pid_values throttle_pid(0, 0, 0, 30, 500, max_power);

    unsigned int net_timer;
    int initial_millis = pros::millis();
    float failsafe = 2000;

    float err_angle;
    //float line_ahead_point = 0.5;
    int last_error_turn = 0;
    int last_error_throttle = 0;
    int last_error_strafe = 0;
    float max_speed = 100;
    printf("Moving to %f %f from %f %f \n", ending_point_x, ending_point_y, starting_point_x, starting_point_y);
    delta_main_line.x = ending_point_x - starting_point_x;
    delta_main_line.y = ending_point_y - starting_point_y;

    do {
      positionErr.x = position.x - ending_point_x;
      positionErr.y = position.y - ending_point_y;
      err_angle = target_angle - orientation;
      vectorToPolar(positionErr, positionErrPolar);
      positionErrPolar.theta -= err_angle;
      polarToVector(positionErrPolar, positionErr);

      int final_power_turn = pid_calc(&turn_pid, degToRad(target_angle), orientation);
      int final_power_strafe = pid_calc(&strafe_pid, ending_point_y, position.y);
      strafe_pid.error = positionErr.y;
      int final_power_throttle = pid_calc(&throttle_pid, ending_point_x, position.x);
      throttle_pid.error = positionErr.x;

      drive_left.move(final_power_turn + final_power_throttle + final_power_strafe);
      drive_left_b.move(final_power_turn + final_power_throttle - final_power_strafe);
      drive_right.move(final_power_turn - final_power_throttle + final_power_strafe);
      drive_right_b.move(final_power_turn - final_power_throttle - final_power_strafe);

      pros::delay(10);

    } while (positionErr.y < 0 && positionErr.x < 0 && fabs(turn_pid.error) < target_angle && (pros::millis() < net_timer) && ((initial_millis + failsafe) > pros::millis()));

    printf("driving done\n");
    drive_set(0);

    printf("driving done\n");
}

 void math_test(float starting_point_x, float starting_point_y, float ending_point_x, float ending_point_y) {

// //main line is the line created from the starting point to the ending point
// //offset line is the line created from the starting point and the current position
//
//  	while(true) {
//     float magn_main_line;
//     float cross_product;
//     float offset_distance;
//     float line_point = 0.5; //how forward x is from P
//     float right_angle = 1.5708;
//
//     float dot_product;
//     float angle_to_turn;
//     float angle_offset_tri;
//     float magn_offset_line;
//     vector main_line_vector;
//     vector offset_line_vector;
//     vector unit_vector;
//     float current_position_x = position.x;
//     float current_position_y = position.y;
//     float current_position_x_prime =current_position_x;
//     float current_position_y_prime = current_position_y;
//     float starting_point_x_prime = starting_point_x;
//     float starting_point_y_prime = starting_point_y;
//     float ending_point_y_prime = ending_point_y;
//     float ending_point_x_prime = ending_point_x;
//     //------------------------------------FINDING THE LENGTH TILL P-----------------------------------------
//         main_line_vector.x = ending_point_x - starting_point_x;//vector: x coordinate of main line
//         main_line_vector.y = ending_point_y - starting_point_y;//vector: y coordinate of main line
//
//         offset_line_vector.x = position.x - starting_point_x;//vector: x coordinate of offset line
//         offset_line_vector.y = position.y - starting_point_y;//vector: y coordinate of offset line
//
//         magn_main_line = sqrt(fabs((powf(ending_point_x - starting_point_x, 2) + powf(ending_point_y - starting_point_y, 2)))); //magnitude of main line
//
//         unit_vector.x = (main_line_vector.x / magn_main_line); //unit vector: x coordinate of the main line
//         unit_vector.y = (main_line_vector.y / magn_main_line); //unit vector: y coordinate of the main line
//
//         dot_product = ((offset_line_vector.x * unit_vector.x) + (offset_line_vector.y * unit_vector.y)); //this is the distance between start point and perpendicular point from current position
//
//
//         //-------------------------------------FINDING THE LENGTH TILL P-----------------------------------------
//
//         //-------------------------------------LEFT OR RIGHT OF LINE----------------------------------------------
//
//         // starting_point_x_prime -= ending_point_x;
//         // starting_point_y_prime -= ending_point_y;
//         //
//         // current_position_x_prime -= ending_point_x;
//         // current_position_y_prime -= ending_point_y;
//
//         //cross_product = (starting_point_x_prime * current_position_y) - (starting_point_y_prime * current_position_x);// this is to determine if current position is left or right of the main line
//          cross_product = (current_position_x_prime - starting_point_x) * (ending_point_y_prime - starting_point_y_prime) -
//          (current_position_y_prime - starting_point_y_prime) * (ending_point_x_prime - starting_point_x_prime);
//
        // if (cross_product > -10)
        // {
        // // printf("Right Of Line");
        // //pros::lcd::print(1, "RIGHT OF LINE");
        //
        // }
//
//         else if (cross_product < 10)
//         {
//         // printf("Left Of Line");
//         pros::lcd::print(1, "LEFT OF LINE");
//
//         }
//
//         else
//         {
//         // printf("On The Line");
//         pros::lcd::print(1, "ON LINE");
//
//         }
//         //-------------------------------------------FINDING HOW MUCH TO TURN TO GET ON LINE------------------------------------------------------------------
//
//         magn_offset_line = sqrt((powf(current_position_x - starting_point_x, 2) + powf(current_position_y - starting_point_y, 2))); // magnitude of offset line
//
//         offset_distance = sqrtf(powf(magn_offset_line, 2) - (powf(dot_product, 2))); // this is the perpendicular distance from current position to main line
//
//         angle_offset_tri =  atan2f(line_point,offset_distance);// * 180 / 3.1415
//
//         angle_to_turn = right_angle - angle_offset_tri; // this is how much i have to turn to get back on the line
//
//         //------------------------------------------------FINDING HOW MUCH TO TURN TO GET ON LINE--------------------------------------------------------------
//         pros::lcd::print(2, "cross_product %f\n", cross_product);
//         pros::lcd::print(4, "angle_to_turn %f\n", angle_to_turn);
//         pros::lcd::print(5, "magn_offset_line %f\n", magn_offset_line);
//         pros::lcd::print(6, "offset_distance %f\n", offset_distance);
//         pros::lcd::print(7, " dot_product%f\n", dot_product);
//         pros::delay(10);
//
//  }

  vector rotation_vector;
  vector delta_main_line;
  float angle_main_line;
  vector rotated_main_line;
  float line_ahead_point = 0.5;
  float target_orientation;
  float line_point_angle;

  while (true) {

    // vector rotation_vector;
    // vector delta_main_line;
    // float angle_main_line;
    // vector rotated_main_line;
    // float line_ahead_point = 0.5;
    // float target_orientation;
    // float line_point_angle;

    delta_main_line.x = ending_point_x - starting_point_x;
  	delta_main_line.y = ending_point_y - starting_point_y;

  	angle_main_line = atan2f(delta_main_line.x, delta_main_line.y);

  	rotation_vector.x = position.x - ending_point_x;
  	rotation_vector.y = position.y - ending_point_y;

  	rotated_main_line.x = (rotation_vector.x * cosf(angle_main_line)) - (rotation_vector.y * sinf(angle_main_line));
  	rotated_main_line.y = (rotation_vector.x * sinf(angle_main_line)) + (rotation_vector.y * cosf(angle_main_line));

    line_point_angle = atanf(rotated_main_line.x / line_ahead_point);
    target_orientation = angle_main_line + line_point_angle;

      //pros::lcd::print(2, "delta_main_line.x %f\n", delta_main_line.x);
      //pros::lcd::print(3, "delta_main_line.y %f\n", delta_main_line.y);

      //pros::lcd::print(4, "angle_main_line %f\n", angle_main_line);

      //pros::lcd::print(5, "orientation %f\n", orientation);
      //pros::lcd::print(6, "rotation_vector.y %f\n", rotation_vector.y);
      //pros::lcd::print(7, " target_orientation %f\n", target_orientation);

    pros::delay(10);
  }
}
