#include "main.h"
#include "drive.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "math.h"
//#include "ritam_drive.h"

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

float nearestangle(float target_angle, float reference_angle)
{
  return roundf(((reference_angle-target_angle) / (2*pi)) *  (2 * pi) + target_angle);
}

pros::task_t tracking_task;

//-------------------------------TRACKING VALUES------------------------------------------------------------------------------------------------------------------

polar vector_to_polar(vector v)
{
return{sqrt(powf(v.x,2) + powf(v.y,2)), atan2f(v.x,v.y)};
}

vector polar_to_vector(polar p)
{
return{p.r * sin(p.theta),p.r * cos(p.theta)};
}

void vectorToPolar(vector& vector, polar& polar)
{
	if (vector.x || vector.y)
	{
		polar.r = sqrt(vector.x * vector.x + vector.y * vector.y);
		polar.theta = atan2(vector.y, vector.x);
	}
	else
		polar.r = polar.theta = 0;
}

void polarToVector(polar& polar, vector& vector)
{
	if (polar.r)
	{
		vector.x = polar.r * cos(polar.theta);
		vector.y = polar.r * sin(polar.theta);
	}
	else
		vector.x = vector.y = 0;
}


void tracking_update(void*ignore)
{

  while(true){
    // float ticks_to_degs = 3.18627451;
    //float pi = 3.1415926535;
    float degrees_encoder_left = (left_encoder.get_value());//* ticks_to_degs
    float degrees_encoder_right = (right_encoder.get_value());//* ticks_to_degs
    float degrees_encoder_back = (back_encoder.get_value());//* ticks_to_degs

    float degrees_to_rad_left = (pi/180) * degrees_encoder_left; //gives back values in radians
    float degrees_to_rad_right = (pi/180) * degrees_encoder_right; //gives back values in radians
    float degrees_to_rad_back = (pi/180) * degrees_encoder_back; //gives back values in radians


    const float wheel_radius = 1.375;

    float inches_traveled_left = degrees_to_rad_left * wheel_radius; //gives back values in inches
    float inches_traveled_right = degrees_to_rad_right * wheel_radius; //gives back values in inches
    float inches_traveled_back = degrees_to_rad_back * wheel_radius; //gives back values in inches


    const double distance_between_centre = 6.200785;
    //CORDINATES facing the enemies side is 𝜃r = 0

    beginning_orientation = 0;

    float new_absolute_orientation = beginning_orientation + ((inches_traveled_left - inches_traveled_right)/(2*distance_between_centre)); // gives back values in radians and gives us the orientation of the bot

    float change_in_angle = new_absolute_orientation - orientation; // gives back value in radians and also how much it has rotated from its previous point

    vector local_offset;

    if (change_in_angle == 0)
    {
     local_offset = {inches_traveled_back - prev_inches_traveled_back , inches_traveled_right - prev_inches_traveled_right};
    }
    else
    {
      local_offset = { 2 * sin(change_in_angle / 2) * (((inches_traveled_back - prev_inches_traveled_back) / change_in_angle) + 3.6377953f),
      2 * sin(change_in_angle/2) * ((inches_traveled_right - prev_inches_traveled_right) / change_in_angle + 6.200785f)};
    }

    float average_orientation = orientation + (change_in_angle/2);
    float rotation_amount = orientation + (change_in_angle)/2;

    polar offset_polar = vector_to_polar(local_offset);
    offset_polar.theta += rotation_amount;
    vector global_offset = polar_to_vector(offset_polar);

    position.x += global_offset.x;
    position.y += global_offset.y;


  //  pros::lcd::print(5, "change_in_angle %f\n", change_in_angle);
  // pros::lcd::print(0, "orientation %f\n", orientation);
  // pros::lcd::print(1, "position.x %f\n", position.x);
  // pros::lcd::print(2, "position.y %f\n", position.y);
  // pros::lcd::print(4, "local_offset %f\n", local_offset);
  //pros::lcd::print(5, "change_left%f\n", inches_traveled_left - prev_inches_traveled_left);
  // pros::lcd::print(6, "change_right%f\n", inches_traveled_right- prev_inches_traveled_right);
 //pros::lcd::print(7, "change_in_angle %f\n", change_in_angle);
    orientation = new_absolute_orientation;//gives back value in radians
    prev_inches_traveled_left = inches_traveled_left;
    prev_inches_traveled_right = inches_traveled_right;
    prev_inches_traveled_back = inches_traveled_back;







    pros::delay(10);

    //return orientation;
  }
}
//------------------------------------------------------------------------------------------------------------------

void drive_line_up (int speed, int run_time_drive)
{
drive_set(speed);
pros::delay(run_time_drive);
drive_set(0);
}


void drive_pid(float target, unsigned int timeout, int max_speed)
{

  pros::ADIGyro gyro (gyro_port, 0.9192647);
  gyro.reset();
  reset_drive_encoders();		//reset encoder values


if (pros::competition::is_autonomous()){

    int failsafe = 2500;    //varible value
    int initial_millis = pros::millis();

    float Kp = .35; //0.28  //.3
    float Kd = .06;		//0.26 // .06
    float Ki = .0007; //0 // .0007
    //unsigned int timeout = 200;
    float error;
    float last_error;
    float final_power;

    float proportional;
    float integral;
    float derivative;

    int integral_limit = 50; //50
    unsigned int net_timer;
    float Kp_C = 0.4; //0.4
    double error_c;
    int direction;
    float PI = 3.1415926535;

    float left;
    float right;

    //motor_set_zero_position(DRIVE_LEFT, 0);

    int encoder_average;
    int average_left;
    int average_right;

    int max_positive_speed = 120;			//caping minimum and maximum speeds for robot
    int max_negative_speed = -120;		//to prevent stalling

    bool timer_drive = true;			//used to exit out of PID loop after robot reaches error

    net_timer = pros::millis() + timeout; //just to initialize net_timer at first

    if(target > 0){direction = 1;}
    else if (target < 0){direction = -1;}

      while((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis())){
printf("gyro %f\n", gyro.get_value());
        encoder_average = (drive_left.get_position() + drive_right.get_position() +
                          drive_left_b.get_position() + drive_right_b.get_position())/4;
        error = (target/(4*PI) * 900) - encoder_average;		//wheel size is 4.17 inches

        derivative = (error - last_error)*Kd;
        last_error = error;
        proportional = error*Kp;
        integral = (error + integral)*Ki;

        if (integral > integral_limit){
          integral = integral_limit;
        }

        if (-integral < -integral_limit){
          integral = -integral_limit;
        }

        if(fabs(error) > (12/(4*PI))){
          integral = 0;
        }

        final_power = proportional + derivative + integral;

          if (final_power > max_speed){
            final_power = max_speed;
          }

          if (final_power < -max_speed){
            final_power = -max_speed;
          }

        //error_c = correction_drive + adi_gyro_get(gyro);
      //  printf("error_c value: %5f\n", error_c);
      //  printf("gyro %5f\n", adi_gyro_get(gyro));

      if(fabs(error) > (0.07/(4*PI) * 900))//0.1   .01    .08
      {
        if (direction > 0){   //while going forward
          error_c = correction_drive +  gyro.get_value();

        //  printf("inside loop forwards\n");
          left_drive_set(final_power - error_c*Kp_C);
          right_drive_set(final_power + error_c*Kp_C);
        }

        else if (direction < 0){  //while going backwards
          error_c = correction_drive +  gyro.get_value();

        //  printf("inside loop backwards\n");
          left_drive_set(final_power - error_c*Kp_C);
          right_drive_set(final_power + error_c*Kp_C);
        }
      }

        else {
          left_drive_set(final_power);
          right_drive_set(final_power);
        }

        if (fabs(error) < (1.5/(4*PI) * 900)){	//less than 1 inches
          timer_drive = false;		//start timer to to exit PID loop
          integral = 0;
        }

        else if (timer_drive)
        {
          net_timer = pros::millis() + timeout;
        }

      pros::delay(20);

  }

}

  drive_set(0);		//set drive to 0 power
  printf("encoder avg: %f\n",(
    drive_left.get_position() + drive_right.get_position() +
     drive_left_b.get_position() + drive_right_b.get_position())/286.5);
  // lcd_print(5, "encoder left F %f", (motor_get_position(DRIVE_LEFT_F)/71.62));

  //printf("encoder left F %f\n", (motor_get_position(DRIVE_LEFT_F)/71.62));

    //
    correction_turn =  gyro.get_value() + correction_drive;
  prev_correction_drive = correction_drive;   //
  correction_drive =  gyro.get_value() + prev_correction_drive;
  //correction_turn = correction_drive;

  printf("correction drive %f\n", correction_drive);
    printf("correction_turn = %1f\n", correction_turn);
}


//-------------------------------------------------------------------------------------------------------------------------


void turn_pid(float degs, float Ki, unsigned int timeout) //decrease ki for small turns and increase for big turns
{

  pros::ADIGyro gyro (gyro_port, 0.9192647);
  gyro.reset();

    int failsafe = 1600; //still needs testing    1200
    int initial_millis = pros::millis();
    degrees_flag = degs*10;

    float Kp;
    float Kd;
    //float Ki;

    int max_positive_speed;			//caping minimum and maximum speeds for robot
    int max_negative_speed;		//to prevent stalling

    if(fabs(degs) > 0){

      gyro.reset();
      Kp = 0.25; //0.23 .253
      Kd = 1.21;  //1.20
    //  Ki = 0.25;    //0.23
      max_positive_speed = 100;			//caping minimum and maximum speeds for robot
      max_negative_speed = -100;		//to prevent stalling
    }

    else{
      //correction_turn = 0;
    //  Kp = 0.25;  //0.22
      //Kd = 0.14;		//0.145
    //  Ki = 0.15;    //0.15

      max_positive_speed = 30;			//caping minimum and maximum speeds for robot
      max_negative_speed = -30;		//to prevent stalling
    }


    float proportional, integral, derivative;

    double error;
    int last_error;
    int integral_limit = 50;  //50
    float final_power;
    unsigned int net_timer;
    int direction;

    int encoder_average;

    bool timer_turn = true;

    net_timer = pros::millis() + timeout; //just to initialize net_timer at first

      while((pros::millis() < net_timer) && pros::competition::is_autonomous()&& ((initial_millis + failsafe) > pros::millis()))// && ((initial_millis + failsafe) > millis())
      {

        error = degs*10 - gyro.get_value(); //(degs*10 - correction_turn)
        //error *= 3.14 / 180.0;
        //error = atan2(sin(error), cos(error)) * (180.0 / 3.14);

        derivative = (error - last_error)*Kd;
        last_error = error;
        integral = (error + integral)*Ki;
        proportional = error*Kp;

        if (fabs(error) > 250){ // start integral when target is less than 25 degrees
          integral = 0;
        }

        if (integral > integral_limit){
            integral = integral_limit;
        }

        if (-integral < -integral_limit){
            integral = -integral_limit;
        }

          final_power = proportional + derivative + integral;

          if (final_power > max_positive_speed){
            final_power = max_positive_speed;
          }

          if (final_power < max_negative_speed){
            final_power = max_negative_speed;
          }

          turn_set(final_power);

        if (fabs(error) < 10){ //12
          integral = 0;
          //printf("timer starts \n");
          timer_turn = false;
        }

        if (timer_turn){
          net_timer = pros::millis() + timeout;
        }

      //  printf("gyro value of turn: %1f\n", adi_gyro_get(gyro));
        pros::delay(20);

        //correction_drive = (correction_turn + adi_gyro_get(gyro)) - degs;
      }

    turn_set(0);
    printf("gyro value of turn: %1f\n", gyro.get_value());
    //lcd_print(3, "gyro turn value: %f", adi_gyro_get(gyro));

    correction_drive = (correction_turn + gyro.get_value() - degrees_flag);
    prev_correction_turn = correction_turn;
    correction_turn = (prev_correction_turn + gyro.get_value() - degrees_flag);
    error = 0;
    gyro.reset();
    //printf("gyro value of turn after reset: %1f\n", adi_gyro_get(gyro));
    //adi_gyro_reset(gyro);
// avg 897.75
}


void drive_pid_correction_switch(float target, unsigned int timeout, bool correction_switch, int max_speed = 100)
{

  pros::ADIGyro gyro (gyro_port, 0.9192647);
  gyro.reset();
  reset_drive_encoders();		//reset encoder values


if (pros::competition::is_autonomous()){

    int failsafe = 2500;    //varible value
    int initial_millis = pros::millis();

    float Kp = .35; //0.28  //.3
    float Kd = .06;		//0.26 // .06
    float Ki = .0007; //0 // .0007
    //unsigned int timeout = 200;
    float error;
    float last_error;
    float final_power;

    float proportional;
    float integral;
    float derivative;

    int integral_limit = 50; //50
    unsigned int net_timer;
    float Kp_C = 0.4; //0.4
    double error_c;
    int direction;
    float PI = 3.1415926535;

    float left;
    float right;

    //motor_set_zero_position(DRIVE_LEFT, 0);

    int encoder_average;
    int average_left;
    int average_right;

    int max_positive_speed = 120;			//caping minimum and maximum speeds for robot
    int max_negative_speed = -120;		//to prevent stalling

    bool timer_drive = true;			//used to exit out of PID loop after robot reaches error

    net_timer = pros::millis() + timeout; //just to initialize net_timer at first

    if(target > 0){direction = 1;}
    else if (target < 0){direction = -1;}

      while((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis())){
printf("gyro %f\n", gyro.get_value());
        encoder_average = (drive_left.get_position() + drive_right.get_position() +
                          drive_left_b.get_position() + drive_right_b.get_position())/4;
        error = (target/(4*PI) * 900) - encoder_average;		//wheel size is 4.17 inches

        derivative = (error - last_error)*Kd;
        last_error = error;
        proportional = error*Kp;
        integral = (error + integral)*Ki;

        if (integral > integral_limit){
          integral = integral_limit;
        }

        if (-integral < -integral_limit){
          integral = -integral_limit;
        }

        if(fabs(error) > (12/(4*PI))){
          integral = 0;
        }

        final_power = proportional + derivative + integral;

          if (final_power > max_speed){
            final_power = max_speed;
          }

          if (final_power < -max_speed){
            final_power = -max_speed;
          }

        //error_c = correction_drive + adi_gyro_get(gyro);
      //  printf("error_c value: %5f\n", error_c);
      //  printf("gyro %5f\n", adi_gyro_get(gyro));

      if(fabs(error) > (0.07/(4*PI) * 900) && correction_switch)//0.1   .01    .08
      {
        if (direction > 0){   //while going forward
          error_c = correction_drive +  gyro.get_value();

        //  printf("inside loop forwards\n");
          left_drive_set(final_power - error_c*Kp_C);
          right_drive_set(final_power + error_c*Kp_C);
        }

        else if (direction < 0){  //while going backwards
          error_c = correction_drive +  gyro.get_value();

        //  printf("inside loop backwards\n");
          left_drive_set(final_power - error_c*Kp_C);
          right_drive_set(final_power + error_c*Kp_C);
        }
      }

        else {
          left_drive_set(final_power);
          right_drive_set(final_power);
        }

        if (fabs(error) < (1.5/(4*PI) * 900)){	//less than 1 inches
          timer_drive = false;		//start timer to to exit PID loop
          integral = 0;
        }

        else if (timer_drive)
        {
          net_timer = pros::millis() + timeout;
        }

      pros::delay(20);

  }

}

  drive_set(0);		//set drive to 0 power
  printf("encoder avg: %f\n",(
    drive_left.get_position() + drive_right.get_position() +
     drive_left_b.get_position() + drive_right_b.get_position())/286.5);
  // lcd_print(5, "encoder left F %f", (motor_get_position(DRIVE_LEFT_F)/71.62));

  //printf("encoder left F %f\n", (motor_get_position(DRIVE_LEFT_F)/71.62));

    //
    correction_turn =  gyro.get_value() + correction_drive;
  prev_correction_drive = correction_drive;   //
  correction_drive =  gyro.get_value() + prev_correction_drive;
  //correction_turn = correction_drive;

  printf("correction drive %f\n", correction_drive);
    printf("correction_turn = %1f\n", correction_turn);
}


//----------------------------ENCODER PIDS------------------------------------------------------------------------------------------------------


void turn_pid_encoder_average(double target, unsigned int timeout){ //makn encoder pid

  drive_distance_correction = 0;
  reset_drive_encoders();

  int ticks_to_deg = 3;  //3.18627451 //3 //3.044444444
  degrees_flag = target*ticks_to_deg;

  float Kp = 0.45;  //0.45
  float Kd = 0.7;		//0.7
  float Ki = 0.25;    //0.25
  float proportional, integral, derivative;

  float error;
  int last_error = 0;
  float final_power;
  float encoder_avg;

  int max_speed = 110; //90

  int failsafe = 2000;    //2000
  int initial_millis = pros::millis();

  int integral_limit = 30;
  unsigned int net_timer;

  bool timer_turn = true;


  net_timer = pros::millis() + timeout; //just to initialize net_timer at first

    while((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis())){

      encoder_avg = (left_encoder.get_value() - right_encoder.get_value())/2;
      error = (target*ticks_to_deg - correction_turn) - encoder_avg;


      derivative = (error - last_error)*Kd;
      last_error = error;
      integral = (error + integral)*Ki;
      proportional = error*Kp;

      // start integral when target is less than 30 degrees
        if (fabs(error) > (30*ticks_to_deg)){ integral = 0; }

        if (integral > integral_limit){ integral = integral_limit; }

        if (-integral < -integral_limit){ integral = -integral_limit; }


      final_power = proportional + derivative + integral;

          if (final_power > max_speed){
            final_power = max_speed;
          }

          if (final_power < -max_speed){
            final_power = -max_speed;
          }


      turn_set(final_power);


      if (timer_turn == true){
        net_timer = pros::millis() + timeout;
      }

      if (fabs(error) < 2*ticks_to_deg){   //if less than 2 degrees
        integral = 0;
        //printf("timer starts \n");
        timer_turn = false;
      }


      pros::delay(20); //20

}
  turn_set(0);

  //printf("gyro value of turn: %1f\n", adi_gyro_get(gyro));
  //  lcd_print(4, "gyro turn value: %f", adi_gyro_get(gyro));

  //adi_gyro_reset(gyro);
  //printf("gyro value of turn after reset: %1f\n", adi_gyro_get(gyro));
  //adi_gyro_reset(gyro);

    correction_drive = (correction_turn + (left_encoder.get_value() - right_encoder.get_value())/2 - degrees_flag);
    prev_correction_turn = correction_turn;
    correction_turn = ((left_encoder.get_value() - right_encoder.get_value())/2 - degrees_flag + prev_correction_turn);///ticks_to_deg;

    error = 0;
    printf("correction turn %f\n", correction_turn);
    printf("encoder avg %d\n", (left_encoder.get_value() + right_encoder.get_value())/2);

//  reset_error_globals();
  //  correction_turn = (left_encoder.get_value() - right_encoder.get_value()) + correction_drive;
  }



  void drive_pid_encoder(float target, unsigned int timeout, int max_speed, float Kp_C){//, float Kp){

    //reset_drive_encoders();			reset encoder values

  if (pros::competition::is_autonomous()){

    int failsafe = 2500;    //varible value
    int initial_millis = pros::millis();

    float Kp = 0.5; //0.2
    float Kd = 0.8;		//0.27
    float Ki = 0; //0
    //unsigned int timeout = 200;
    float error;
    float last_error;
    float final_power;

    float proportional;
    float integral;
    float derivative;

    int integral_limit = 50;
    unsigned int net_timer;
    //float Kp_C = 0.4; //0.4
    double error_c;
    int direction;

    float left;
    float right;

    //motor_set_zero_position(DRIVE_LEFT, 0);

    int encoder_average;
    int average_left;
    int average_right;

    //int max_speed = 110;

    bool timer_drive = true;			//used to exit out of piD loop after robot reaches error

    net_timer = pros::millis() + timeout; //just to initialize net_timer at first

      if(target > 0){direction = 1;}
      else if (target < 0){direction = -1;}

      	while((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis())){

          encoder_average = (right_encoder.get_value() + left_encoder.get_value())/2;

      		error = ((target)/(4*pi) * 360)  - drive_distance_correction - encoder_average;		//wheel size is 4.17 inches

      		derivative = (error - last_error)*Kd;
      		last_error = error;
      		proportional = error*Kp;
          integral = (error + integral)*Ki;

          if (integral > integral_limit){
            integral = integral_limit;
          }

          if (-integral < -integral_limit){
            integral = -integral_limit;
          }

          if(fabs(error) > (12/(4*pi))){
            integral = 0;
          }

          final_power = proportional + derivative + integral;

            if (final_power > max_speed){
              final_power = max_speed;
            }

            if (final_power < -max_speed){
              final_power = -max_speed;
            }

          //error_c = correction_drive + gyro.get_value();
        //  printf("error_c value: %5f\n", error_c);
        //  printf("gyro %5f\n", gyro.get_value());

        error_c = (left_encoder.get_value() - right_encoder.get_value());

        if(fabs(error) > (0.1/(4*pi) * 360)){

            error_c = error_c + correction_drive;

            left_drive_set(final_power - error_c*Kp_C);
            right_drive_set(final_power + error_c*Kp_C);
        }


          else {
            left_drive_set(final_power);
            right_drive_set(final_power);
          }

      		if (fabs(error) < (1/(4*pi) * 360)){	//less than 1 inches
      			timer_drive = false;		//start timer to to exit piD loop
            integral = 0;
      		}

      		else if (timer_drive){
      			net_timer = pros::millis() + timeout;
      		}

    		pros::delay(20);

    }

  }

  	drive_set(0);		//set drive to 0 power

  //  printf("encoder avg: %d\n", (left_encoder.get_value() + right_encoder.get_value())/2);
  //  pros::lcd::print(7, "encoder avg %d", (left_encoder.get_value() + right_encoder.get_value())/2);


    correction_turn = ((left_encoder.get_value() - right_encoder.get_value())/2 + correction_drive);///ticks_to_deg;
    prev_correction_drive = correction_drive;
    correction_drive = (left_encoder.get_value() - right_encoder.get_value()) + prev_correction_drive;

    drive_distance_correction = ((left_encoder.get_value() + right_encoder.get_value())/2)
                    - ((target)/(4*pi) * 360) + drive_distance_correction;

    printf("correction drive %f\n", correction_drive);
    printf("correction_turn = %1f\n", correction_turn);

    pros::lcd::print(0, "correction drive %f", correction_drive);
  }

  //
  // correction_drive = (correction_turn + (left_encoder.get_value() - right_encoder.get_value())/2 - degrees_flag);
  // prev_correction_turn = correction_turn;
  // correction_turn = (left_encoder.get_value() - right_encoder.get_value())/2 - degrees_flag + prev_correction_turn;

//-------------------------------------POSITION PIDS--------------------------------------------------------------------


  void position_turn(float target, int timeout, float kp)//, int timeout
  {
    //tracking_update();
    float kd = 3;//70   3
    float ki = 0;//0.06   0
    float proportional, derivative, integral;

    float error;
    float final_power;
    float encoder_avg;
    int last_error = 0;
    int integral_limit = 50;

    int max_speed = 100; //90
    float max_error = 0.001f;
    bool timer_turn = true;
    float net_timer;

    int failsafe = 2000;    //varible value
    int initial_millis = pros::millis();

    do {
      target = nearestangle(target,orientation);

          encoder_avg = orientation;
          error = (target * pi/180) - encoder_avg;
          derivative = (error - last_error)*kd;
          last_error = error;
          integral = error + integral;
          proportional = error*kp;

          pros::lcd::print(0, "orientation %f\n", orientation);
          pros::lcd::print(1, "error %f\n", error);

          // start integral when target is less than 30 degrees
          if (fabs(error) > (0.383972)){ integral = 0; }//0.349066

          if (integral > integral_limit){ integral = integral_limit; }

          if (-integral < -integral_limit){ integral = -integral_limit; }


          final_power = proportional + derivative + (integral * ki);


          turn_set(final_power);
          // drive_left.move_velocity(final_power);
          // drive_left_b.move_velocity(final_power);
          // drive_right.move_velocity(-final_power);
          // drive_right_b.move_velocity(-final_power);

          pros::lcd::print(2, "final_power %f\n", final_power);


          if (timer_turn == true){
          net_timer = pros::millis() + timeout;
          }

          if (fabs(error) < 0.0174533)//0.0349066
          {   //if less than 2 degrees // ticks_to_deg = 3.18627451      (2*3.18627451) * (pi/180)
          timer_turn = false;
          }

          pros::delay(20); //20

          }
      while((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis()));
      // drive_left.move_velocity(0);
      // drive_left_b.move_velocity(0);
      // drive_right.move_velocity(0);
      // drive_right_b.move_velocity(0);
      turn_set(0);
      printf("Degrees Turned: %f\n", orientation);

 }


 void position_face_point(float target_x, float target_y,int timeout)
 {
    vector error;
    float Kp = 50; //0.2

    float kd = 0;//70   3
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

    int failsafe = 2000;    //varible value
    int initial_millis = pros::millis();


    float direction_face;
    //net_timer = pros::millis() + timeout; //just to initialize net_timer at first

    float encoder_average;
    float error_p;

      do {
        error.x = target_x - position.x;		//wheel size is 4.17 inches
        error.y = target_y - position.y;

        direction_face = atan2f(error.y , error.x);

        encoder_avg = orientation;
        error_p = direction_face - encoder_avg;
        derivative = (error_p - last_error)*kd;
        last_error = error_p;
        integral = (error_p + integral);
        proportional = error_p*Kp;



        //pros::lcd::print(6, "target_x %f\n", target_x);
        //pros::lcd::print(7, "error_p %f\n", error_p);

        // start integral when target is less than 30 degrees
        if (fabs(error_p) > (0.383972)){ integral = 0; }//0.349066

        if (integral > integral_limit){ integral = integral_limit; }

        if (-integral < -integral_limit){ integral = -integral_limit; }


        final_power = proportional + derivative + (integral * ki);//

        pros::lcd::print(6, "final_power %f\n", final_power);
        turn_set(final_power);

        if (timer_turn == true){
        net_timer = pros::millis() + timeout;
        }

        if (fabs(error_p) < 0.0174533)//0.0349066
        {   //if less than 2 degrees // ticks_to_deg = 3.18627451      (2*3.18627451) * (pi/180)
        timer_turn = false;
        }

        pros::delay(20); //20

        }
       while((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis()));
      turn_set(0);		//set drive to 0 power
 }


 void position_drive(float starting_point_x, float starting_point_y, float ending_point_x, float ending_point_y, float max_speed, float max_error, int timeout)
 {
    vector error;
    vector positionErr;
    polar positionErrPolar;
    //TURN VALUES
    float kp_tu = 4; //6
    float proportional_tu;//, derivative_t, integral_t
    float final_power_tu;
    //TURN VALUES
    //DRIVE VALUES
    float kp_d = 4; //0.2
    float proportional_d;//, derivative_d
    float final_power_d;
    //DRIVE VALUES

    float encoder_avg;
    int last_error = 0;


    //int max_speed = 100; //90
    //float max_error = 0.001f;
    bool timer_drive = true;
    unsigned int net_timer;

    int failsafe = 2000;    //varible value
    int initial_millis = pros::millis();
    int direction_face;
    bool timer_turn = true;

    float magnitude;
    float encoder_average;
    float error_tu;
    float error_d;

    vector rotation_vector;
    vector delta_main_line;
    float angle_main_line;
    vector rotated_main_line;
    float line_ahead_point = 0.5;
    float target_orientation;
    float line_point_angle;
    float line_angle;
    float correction = 0;
    float finalpower;
    float err_angle;
    float err_x;
    float correctA;

    printf("Moving to %f %f from %f %f at %f \n", ending_point_x, ending_point_y, starting_point_x, starting_point_y, max_speed);

    delta_main_line.x = ending_point_x - starting_point_x;
    delta_main_line.y = ending_point_y - starting_point_y;

         while(true)//(pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis())
         {
            angle_main_line = atan2f(delta_main_line.x, delta_main_line.y);
            line_angle = nearestangle(angle_main_line - (max_speed < 0 ? pi : 0), orientation);
            // rotation_vector.x = position.x - ending_point_x;
            // rotation_vector.y = position.y - ending_point_y;
            //
            // rotated_main_line.x = (rotation_vector.x * cosf(angle_main_line)) - (rotation_vector.y * sinf(angle_main_line));
            // rotated_main_line.y = (rotation_vector.x * sinf(angle_main_line)) + (rotation_vector.y * cosf(angle_main_line));
            //
            // line_point_angle = atanf(rotated_main_line.x / line_ahead_point);
            // target_orientation = angle_main_line + line_point_angle;
          positionErr.x = position.x - ending_point_x;
          positionErr.y = position.y - ending_point_y;
          vectorToPolar(positionErr, positionErrPolar);
          positionErrPolar.theta += angle_main_line;
          polarToVector(positionErrPolar, positionErr);



            if (max_error)
            		{
          			err_angle = orientation - line_angle;
          			err_x = positionErr.x + positionErr.y * tan(err_angle);
          			correctA = atan2(ending_point_x - position.x, ending_point_y - position.y);
          			if (max_speed < 0)
          				correctA += pi;
          			correction = fabs(err_x) > max_error ? 5.0 * (nearestangle(correctA, orientation) - orientation) * sgn(max_speed) : 0; //8.0
            		}


    //------------------------------------------------------------math--------------------------------------------------------

        // error.x = ending_point_x - position.x;
        // error.y = ending_point_y - position.y;
        //
        // encoder_avg = orientation;
        // error_tu = target_orientation - encoder_avg;
        // last_error = error_tu;
        // proportional_tu = error_tu * kp_tu;

      finalpower = round(-127.0 / 30 * positionErr.y) * sgn(max_speed); //40

      // if (finalpower > max_speed)
      // {
      //   finalpower = max_speed;
      // }
      //
      // if (finalpower < -max_speed)
      // {
      //   finalpower = -max_speed;
      // }


      switch (sgn(correction))
        		{
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


        // final_power_tu = proportional_tu;

        //------------------------------DRIVE PART------------------------------------------------
        //
        // error.x = ending_point_x - position.x;		//wheel size is 4.17 inches
        // error.y = ending_point_y - position.y;


        // magnitude = sqrt(powf(error.x,2) + powf(error.y,2)); //output is in inches because error.x and error.y are in inches
        //
        // error_d = magnitude;
        // proportional_d = error_d * kp_d;
        // final_power_d = proportional_d;
        //
        // if (reverse)
        // {
        //   final_power_d = -final_power_d;
        //   target_orientation += pi;
        // }

        pros::lcd::print(1, "final_power_d %f\n", final_power_d);
        pros::lcd::print(2, "angle_to_turn %f\n", target_orientation);

        printf("back_encoder %d\n", back_encoder.get_value());
        printf(" \n");
        printf("position.x %f\n", position.x);
        printf(" \n");
        printf("position.y %f\n", position.y);
        printf(" \n");
        printf("positionErr.x %f\n", positionErr.x);
        printf(" \n");
        printf("positionErr.y %f\n", positionErr.y);
        printf(" \n");
        printf("final_power %f\n", finalpower);
        printf(" \n");
        printf("err_angle %f\n", err_angle);
        printf(" \n");
        printf("err_x %f\n", err_x);
        printf(" \n");
        printf("angle_main_line %f\n", angle_main_line);
        printf(" \n");
        printf("line_angle %f\n", line_angle);
        printf(" \n");
        printf("correctA %f\n", correctA);
        printf(" \n");
        printf("correction %f\n", correction);
        printf(" \n");
        printf("max_error %f\n", max_error);
        printf(" \n");
        printf("orientation %f\n", orientation);
        printf(" \n");
        printf("sgn(max_speed) %d\n", sgn(max_speed));
        printf(" \n");
        printf("tan(err_angle) %f \n", tan(err_angle));
        printf(" \n");
        printf("exp(correction) %f \n", exp(correction));
        printf(" \n");
        printf("Moving to %f %f from %f %f at %f \n", ending_point_x, ending_point_y, starting_point_x, starting_point_y, max_speed);
        printf(" \n");
        printf("--------------------------------------------------------------------------------------\n");
        printf(" \n");


        // if (abs(error_tu) > 0.174533)
        // {
        // final_power_d = 0;
        // }
        //
        // else
        // {
        // final_power_d = proportional_d;
        // }

        // if (abs(error_tu) > 0.0174533) //0.174533
        // {
        // final_power_d *= 1.046774844 - 0.268 * abs(error_tu); //test for smooth thing
        // }
        //
        // left_drive_set(final_power_d - final_power_tu);
        // right_drive_set(final_power_d + final_power_tu);
        //
        // if (timer_turn == true){
        // net_timer = pros::millis() + timeout;
        // }
        //
        // if (fabs(positionErr.y) < 0.3) //test needed  && fabs(err_angle) < 0.0349066
        // {
        // timer_turn = false;
        // }
        // pros::delay(20);
        //   }
        //   drive_set(0);		//set drive to 0 power
        //   printf("driving done\n");
        //
         }
      }


    void position_drive_forward(float target_y, bool reverse, int timeout)
    {
    vector error;

    //DRIVE VALUES
    float kp_d = 3; //0.2
    float proportional_d;//, derivative_d
    float final_power_d;
    //DRIVE VALUES

    float encoder_avg;
    int last_error = 0;


    int max_speed = 100; //90
    float max_error = 0.001f;
    bool timer_drive = true;
    unsigned int net_timer;

    int failsafe = 2000;    //varible value
    int initial_millis = pros::millis();
    bool timer_turn = true;

    float error_d;

    printf("Moving to %f\n", target_y);

    while((pros::millis() < net_timer) && pros::competition::is_autonomous() && ((initial_millis + failsafe) > pros::millis()))
    {

    error.y = target_y - position.y;
    proportional_d = error.y * kp_d;
    final_power_d = proportional_d;

    if (reverse)
    {
    final_power_d = -final_power_d;
    }
    pros::lcd::print(1, "final_power_d %f\n", final_power_d);

    printf("back_encoder %d\n", back_encoder.get_value());
    printf(" \n");
    printf("position.x %f\n", position.x);
    printf(" \n");
    printf("position.y %f\n", position.y);
    printf(" \n");
    printf("final_power_d %f\n", final_power_d);
    printf(" \n");
    printf("error.y %f\n", error.y);
    printf(" \n");
    printf("--------------------------------------------------------------------------------------\n");
    printf(" \n");

    left_drive_set(final_power_d);
    right_drive_set(final_power_d);

    if (timer_turn == true){
    net_timer = pros::millis() + timeout;
    }

    if (fabs(error.y) < 2)//0.0349066     error_d
    {
    timer_turn = false;
    }


    pros::delay(20);
    }
    drive_set(0);		//set drive to 0 power
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
//         if (cross_product > -10)
//         {
//         // printf("Right Of Line");
//         pros::lcd::print(1, "RIGHT OF LINE");
//
//         }
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

while (true)
{
  vector rotation_vector;
  vector delta_main_line;
  float angle_main_line;
  vector rotated_main_line;
  float line_ahead_point = 0.5;
  float target_orientation;
  float line_point_angle;


  delta_main_line.x = ending_point_x - starting_point_x;
  		delta_main_line.y = ending_point_y - starting_point_y;

  		angle_main_line = atan2f(delta_main_line.x, delta_main_line.y);

  		rotation_vector.x = position.x - ending_point_x;
  		rotation_vector.y = position.y - ending_point_y;

  		rotated_main_line.x = (rotation_vector.x * cosf(angle_main_line)) - (rotation_vector.y * sinf(angle_main_line));
  		rotated_main_line.y = (rotation_vector.x * sinf(angle_main_line)) + (rotation_vector.y * cosf(angle_main_line));

      line_point_angle = atanf(rotated_main_line.x / line_ahead_point);
		  target_orientation = angle_main_line + line_point_angle;

      pros::lcd::print(2, "delta_main_line.x %f\n", delta_main_line.x);
      pros::lcd::print(3, "delta_main_line.y %f\n", delta_main_line.y);

      pros::lcd::print(4, "angle_main_line %f\n", angle_main_line);

      pros::lcd::print(5, "orientation %f\n", orientation);
      pros::lcd::print(6, "rotation_vector.y %f\n", rotation_vector.y);
      pros::lcd::print(7, " target_orientation %f\n", target_orientation);

      pros::delay(10);
}
 }
