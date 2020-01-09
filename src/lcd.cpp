#include "main.h"
#include "pros/apix.h"
#include <sstream>
#include <iomanip>
#include "display/lv_conf.h"
#include "lcd.h"
#include "motor_setup.h"
#include "motor_sensor_init.h"
#include "drive.h"
#include "all_used.h"

//ALL DECLARATIONS OF BUTTONS_____________________________________
int switcher;
//test comment
LV_IMG_DECLARE(logo);
lv_obj_t * scr1  = lv_obj_create(NULL, NULL);
lv_obj_t * scr2  = lv_obj_create(NULL, NULL);
lv_obj_t * scr3  = lv_obj_create(NULL, NULL);
lv_obj_t * scr4  = lv_obj_create(NULL, NULL);

lv_obj_t * red_tile = lv_btn_create (scr1, NULL);
lv_obj_t * blue_tile = lv_btn_create (scr1, NULL);
lv_obj_t * skills = lv_btn_create (scr1, NULL);
lv_obj_t * testing = lv_btn_create (scr1, NULL);
lv_obj_t * values = lv_btn_create (scr1, NULL);

lv_obj_t * home_button_red = lv_btn_create (scr2, NULL);
lv_obj_t * home_button_blue = lv_btn_create (scr3, NULL);
lv_obj_t * home_button_values = lv_btn_create (scr4, NULL);

lv_obj_t * red_button_label = lv_label_create(red_tile, NULL);
lv_obj_t * blue_button_label = lv_label_create(blue_tile, NULL);
lv_obj_t * skills_button_label = lv_label_create(skills, NULL);
lv_obj_t * testing_button_label = lv_label_create(testing, NULL);
lv_obj_t * values_button_label = lv_label_create(values, NULL);
lv_obj_t * home_button_label_red = lv_label_create(home_button_red, NULL);
lv_obj_t * home_button_label_blue = lv_label_create(home_button_blue, NULL);
lv_obj_t * home_button_label_values = lv_label_create(home_button_values, NULL);
lv_obj_t * battery_label = lv_label_create(scr4, NULL);
lv_obj_t * encoder_left_label = lv_label_create(scr4, NULL);
lv_obj_t * encoder_right_label = lv_label_create(scr4, NULL);
lv_obj_t * encoder_back_label = lv_label_create(scr4, NULL);
lv_obj_t * position_x_label = lv_label_create(scr4, NULL);
lv_obj_t * position_y_label = lv_label_create(scr4, NULL);
lv_obj_t * orientation_label = lv_label_create(scr4, NULL);
lv_obj_t * velocity_x_label = lv_label_create(scr4, NULL);
lv_obj_t * velocity_y_label = lv_label_create(scr4, NULL);
lv_obj_t * switcher_label = lv_label_create(scr4, NULL);
lv_obj_t * arm_potentiometer_label = lv_label_create(scr4, NULL);
lv_obj_t * angler_potentiometer_label = lv_label_create(scr4, NULL);
lv_obj_t * timer_label = lv_label_create(scr4, NULL);


lv_obj_t * reset = lv_btn_create (scr4, NULL);
lv_obj_t * reset_label = lv_label_create(reset, NULL);

lv_style_t red_button_style;
lv_style_t blue_button_style;
lv_style_t home_button_style;
lv_style_t skills_button_style;
lv_style_t values_button_style;
lv_style_t red_autos_button_style;
lv_style_t blue_autos_button_style;

//--------------------------------------------------------------


//ALL AUTOS-----------------------------------------------------

//RED AUTOS
static lv_res_t red_auto1 (lv_obj_t * btn) {
  switcher = 1;
  //printf("switcher red %d\n", switcher);
  return LV_RES_OK;
}

static lv_res_t red_auto2 (lv_obj_t * btn) {
  switcher = 2;
  //printf("switcher red %d\n", switcher);
  return LV_RES_OK;
}

static lv_res_t red_auto3 (lv_obj_t * btn) {
  switcher = 3;
  //printf("switcher red %d\n", switcher);
  return LV_RES_OK;
}

static lv_res_t red_auto4 (lv_obj_t * btn)  {
  switcher = 4;
  //printf("switcher red %d\n", switcher);
  return LV_RES_OK;
}

//BLUE AUTOS
static lv_res_t blue_auto1 (lv_obj_t * btn) {
  switcher = 5;
  //printf("switcher  %d\n", switcher);
  return LV_RES_OK;
}

static lv_res_t blue_auto2 (lv_obj_t * btn) {
  switcher = 6;
  //printf("switcher red %d\n", switcher);
  return LV_RES_OK;
}

static lv_res_t blue_auto3 (lv_obj_t * btn) {
  switcher = 7;
  //printf("switcher red %d\n", switcher);
  return LV_RES_OK;
}

static lv_res_t blue_auto4 (lv_obj_t * btn) {
  switcher = 8;
  //printf("switcher red %d\n", switcher);
  return LV_RES_OK;
}

//OTHER AUTOS
static lv_res_t skills_auto (lv_obj_t * btn) {
  switcher = 9;
  return LV_RES_OK;
}

static lv_res_t testing_auto (lv_obj_t * btn) {
  switcher = 10;
  return LV_RES_OK;
}

static lv_res_t reset_values (lv_obj_t * btn) {
  left_encoder.reset();
  right_encoder.reset();
  back_encoder.reset();
  beginning_orientation = 0;
  prev_inches_traveled_left = 0;
  prev_inches_traveled_right = 0;
  prev_inches_traveled_back = 0;
  position.y = 0;
  position.x = 0;
  orientation = 0;
  return LV_RES_OK;
}
//--------------------------------------------------------------


//ALL SCREENS AND BUTTONS PRESSINGS-----------------------------

static lv_res_t home_screen(lv_obj_t * btn2) {
  lv_scr_load(scr1);
    return LV_RES_OK;   /*The button is not deleted*/
}

static lv_res_t red_tile_screen(lv_obj_t * btn) {
  lv_scr_load(scr2);
  lv_obj_t * red_auto_1 = lv_btn_create (lv_scr_act(), NULL);//change name when making auto
  lv_obj_t * red_auto_1_label = lv_label_create(red_auto_1, NULL);

  lv_obj_t * red_auto_2 = lv_btn_create (lv_scr_act(), NULL);//change name when making auto
  lv_obj_t * red_auto_2_label = lv_label_create(red_auto_2, NULL);

  lv_obj_t * red_auto_3 = lv_btn_create (lv_scr_act(), NULL);//change name when making auto
  lv_obj_t * red_auto_3_label = lv_label_create(red_auto_3, NULL);

  lv_obj_t * red_auto_4 = lv_btn_create (lv_scr_act(), NULL);//change name when making auto
  lv_obj_t * red_auto_4_label = lv_label_create(red_auto_4, NULL);


  lv_style_copy(&red_autos_button_style, &lv_style_plain);
  red_autos_button_style.body.main_color = LV_COLOR_MAKE(200, 0, 0);
  red_autos_button_style.body.grad_color = LV_COLOR_MAKE(200, 0, 0);
  red_autos_button_style.body.radius = LV_RADIUS_CIRCLE;
  red_autos_button_style.text.color = LV_COLOR_MAKE(0, 0, 0);

  lv_obj_set_size(red_auto_1, 200, 100);//change name when making auto
  lv_obj_align(red_auto_1, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10);
  lv_label_set_text(red_auto_1_label, "Red Front");
  lv_btn_set_style(red_auto_1, LV_BTN_STYLE_REL, &red_autos_button_style);

  lv_obj_set_size(red_auto_2, 200, 100); //change name when making auto
  lv_obj_align(red_auto_2, NULL, LV_ALIGN_IN_TOP_RIGHT, -10, 10);
  lv_label_set_text(red_auto_2_label, "Red Back");
  lv_btn_set_style(red_auto_2, LV_BTN_STYLE_REL, &red_autos_button_style);

  lv_obj_set_size(red_auto_3, 200, 100); //change name when making auto
  lv_obj_align(red_auto_3, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 10, -10);
  lv_label_set_text(red_auto_3_label, "Right 1 Point Unlock");
  lv_btn_set_style(red_auto_3, LV_BTN_STYLE_REL, &red_autos_button_style);

  lv_obj_set_size(red_auto_4, 200, 100); //change name when making auto
  lv_obj_align(red_auto_4, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -10, -10);
  lv_label_set_text(red_auto_4_label, "Left 1 Point Unlock");
  lv_btn_set_style(red_auto_4, LV_BTN_STYLE_REL, &red_autos_button_style);

  //HOME
  lv_style_copy(&home_button_style, &lv_style_plain);
  home_button_style.body.main_color = LV_COLOR_MAKE(0, 0, 0);
  home_button_style.body.grad_color = LV_COLOR_MAKE(0, 0, 0);
  home_button_style.body.radius = LV_RADIUS_CIRCLE;
  home_button_style.text.color = LV_COLOR_MAKE(200, 200, 200);

  lv_obj_set_size(home_button_red, 75, 75);
  lv_obj_align(home_button_red, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_label_set_text(home_button_label_red, "HOME");
  lv_btn_set_style(home_button_red, LV_BTN_STYLE_REL, &home_button_style);

  lv_btn_set_action(home_button_red, LV_BTN_ACTION_CLICK, home_screen);
  lv_btn_set_action(red_auto_1, LV_BTN_ACTION_CLICK, red_auto1);
  lv_btn_set_action(red_auto_2, LV_BTN_ACTION_CLICK, red_auto2);
  lv_btn_set_action(red_auto_3, LV_BTN_ACTION_CLICK, red_auto3);
  lv_btn_set_action(red_auto_4, LV_BTN_ACTION_CLICK, red_auto4);
  return LV_RES_OK;   /*The button is not deleted*/
}

static lv_res_t blue_tile_screen(lv_obj_t * btn1) {

  lv_scr_load(scr3);
  lv_obj_t * blue_auto_1 = lv_btn_create (lv_scr_act(), NULL);
  lv_obj_t * blue_auto_1_label = lv_label_create(blue_auto_1, NULL);

  lv_obj_t * blue_auto_2 = lv_btn_create (lv_scr_act(), NULL);//change name when making auto
  lv_obj_t * blue_auto_2_label = lv_label_create(blue_auto_2, NULL);

  lv_obj_t * blue_auto_3 = lv_btn_create (lv_scr_act(), NULL);//change name when making auto
  lv_obj_t * blue_auto_3_label = lv_label_create(blue_auto_3, NULL);

  lv_obj_t * blue_auto_4 = lv_btn_create (lv_scr_act(), NULL);//change name when making auto
  lv_obj_t * blue_auto_4_label = lv_label_create(blue_auto_4, NULL);


  lv_style_copy(&blue_autos_button_style, &lv_style_plain);
  blue_autos_button_style.body.main_color = LV_COLOR_MAKE(0, 0, 200);
  blue_autos_button_style.body.grad_color = LV_COLOR_MAKE(0, 0, 200);
  blue_autos_button_style.body.radius = LV_RADIUS_CIRCLE;
  blue_autos_button_style.text.color = LV_COLOR_MAKE(0, 0, 0);


  lv_obj_set_size(blue_auto_1, 200, 100);
  lv_obj_align(blue_auto_1, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10);
  lv_label_set_text(blue_auto_1_label, "Red Front");
  lv_btn_set_style(blue_auto_1, LV_BTN_STYLE_REL, &blue_autos_button_style);

  lv_obj_set_size(blue_auto_2, 200, 100); //change name when making auto
  lv_obj_align(blue_auto_2, NULL, LV_ALIGN_IN_TOP_RIGHT, -10, 10);
  lv_label_set_text(blue_auto_2_label, "Red Back");
  lv_btn_set_style(blue_auto_2, LV_BTN_STYLE_REL, &blue_autos_button_style);

  lv_obj_set_size(blue_auto_3, 200, 100); //change name when making auto
  lv_obj_align(blue_auto_3, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 10, -10);
  lv_label_set_text(blue_auto_3_label, "Right 1 Point Unlock");
  lv_btn_set_style(blue_auto_3, LV_BTN_STYLE_REL, &blue_autos_button_style);

  lv_obj_set_size(blue_auto_4, 200, 100); //change name when making auto
  lv_obj_align(blue_auto_4, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -10, -10);
  lv_label_set_text(blue_auto_4_label, "Left 1 Point Unlock");
  lv_btn_set_style(blue_auto_4, LV_BTN_STYLE_REL, &blue_autos_button_style);

  //HOME
  lv_style_copy(&home_button_style, &lv_style_plain);
  home_button_style.body.main_color = LV_COLOR_MAKE(0, 0, 0);
  home_button_style.body.grad_color = LV_COLOR_MAKE(0, 0, 0);
  home_button_style.body.radius = LV_RADIUS_CIRCLE;
  home_button_style.text.color = LV_COLOR_MAKE(200, 200, 200);

  lv_obj_set_size(home_button_blue, 75, 75);
  lv_obj_align(home_button_blue, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_label_set_text(home_button_label_blue, "HOME");
  lv_btn_set_style(home_button_blue, LV_BTN_STYLE_REL, &home_button_style);

  lv_btn_set_action(home_button_blue, LV_BTN_ACTION_CLICK, home_screen);
  lv_btn_set_action(blue_auto_1, LV_BTN_ACTION_CLICK, blue_auto1);
  lv_btn_set_action(blue_auto_2, LV_BTN_ACTION_CLICK, blue_auto2);
  lv_btn_set_action(blue_auto_3, LV_BTN_ACTION_CLICK, blue_auto3);
  lv_btn_set_action(blue_auto_4, LV_BTN_ACTION_CLICK, blue_auto4);
  return LV_RES_OK;   /*The button is not deleted*/
}

static lv_res_t values_screen(lv_obj_t * btn3) {
  pros::Task([] (void *) {
    lv_scr_load(scr4);
    while (true)
  {
    std::ostringstream battery_cap;
    battery_cap << "Battery Level: " << std::setprecision(3) << pros::battery::get_capacity() << "%";
    auto c = battery_cap.str();
    lv_label_set_text(battery_label, c.c_str());

    std::ostringstream encoder_left;
    encoder_left << "Encoder Left: " << std::setprecision(3) << left_encoder.get_value();
    auto el = encoder_left.str();
    lv_label_set_text(encoder_left_label, el.c_str());
    lv_obj_align(encoder_left_label, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 22);

    std::ostringstream encoder_right;
    encoder_right << "Encoder Right: " << std::setprecision(3) << right_encoder.get_value();
    auto er = encoder_right.str();
    lv_label_set_text(encoder_right_label, er.c_str());
    lv_obj_align(encoder_right_label, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 44);

    std::ostringstream encoder_back;
    encoder_back << "Encoder Back: " << std::setprecision(3) << back_encoder.get_value();
    auto eb = encoder_back.str();
    lv_label_set_text(encoder_back_label, eb.c_str());
    lv_obj_align(encoder_back_label, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 66);

    std::ostringstream position_x;
    position_x << "Position.x: " << std::setprecision(3) << position.x;
    auto px = position_x.str();
    lv_label_set_text(position_x_label, px.c_str());
    lv_obj_align(position_x_label, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 88);

    std::ostringstream position_y;
    position_y << "Position.y: " << std::setprecision(3) << position.y;
    auto py = position_y.str();
    lv_label_set_text(position_y_label, py.c_str());
    lv_obj_align(position_y_label, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 110);

    std::ostringstream orientationL;
    orientationL << "Orientation: " << std::setprecision(3) << orientation << " , " << radToDeg(orientation);
    auto o = orientationL.str();
    lv_label_set_text(orientation_label, o.c_str());
    lv_obj_align(orientation_label, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 132);

    std::ostringstream vel_x;
    vel_x << "Velocity.x: " << std::setprecision(3) << velocity.x;
    auto vx = vel_x.str();
    lv_label_set_text(velocity_x_label, vx.c_str());
    lv_obj_align(velocity_x_label, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 154);

    std::ostringstream vel_y;
    vel_y << "Velocity.y: " << std::setprecision(3) << velocity.y;
    auto vy = vel_x.str();
    lv_label_set_text(velocity_y_label, vy.c_str());
    lv_obj_align(velocity_y_label, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 176);

    std::ostringstream switcherV;
    switcherV << "Switcher: " << std::setprecision(3) << switcher;
    auto s = switcherV.str();
    lv_label_set_text(switcher_label, s.c_str());
    lv_obj_align(switcher_label, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 198);

    std::ostringstream arm_potentiometerV;
    arm_potentiometerV << "Arm Encoder: " << arm.get_position();
    auto arpv = arm_potentiometerV.str();
    lv_label_set_text(arm_potentiometer_label, arpv.c_str());
    lv_obj_align(arm_potentiometer_label, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 220);

    std::ostringstream angler_potentiometerV;
    angler_potentiometerV << "Angler Potentiometer: " << std::setprecision(3) << potentiometer_angler.get_value();
    auto anpv = angler_potentiometerV.str();
    lv_label_set_text(angler_potentiometer_label, anpv.c_str());
    lv_obj_align(angler_potentiometer_label, NULL, LV_ALIGN_IN_TOP_MID, 45, 0);

    std::ostringstream timerV;
    timerV << "Auto Time: " << std::setprecision(3) << (timerAuto/1000);
    auto tv = timerV.str();
    lv_label_set_text(timer_label, tv.c_str());
    lv_obj_align(timer_label, NULL, LV_ALIGN_IN_TOP_MID, 45, 22);


    lv_obj_set_size(reset, 75, 75);
    lv_obj_align(reset, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -13, -85);
    lv_label_set_text(reset_label, "RESET");
    lv_btn_set_style(reset, LV_BTN_STYLE_REL, &home_button_style);

    //HOME
    lv_style_copy(&home_button_style, &lv_style_plain);
    home_button_style.body.main_color = LV_COLOR_MAKE(0, 0, 0);
    home_button_style.body.grad_color = LV_COLOR_MAKE(0, 0, 0);
    home_button_style.body.radius = LV_RADIUS_CIRCLE;
    home_button_style.text.color = LV_COLOR_MAKE(200, 200, 200);

    lv_obj_set_size(home_button_values, 75, 75);
    lv_obj_align(home_button_values, NULL, LV_ALIGN_IN_BOTTOM_MID, 185, -8);
    lv_label_set_text(home_button_label_values, "HOME");
    lv_btn_set_style(home_button_values, LV_BTN_STYLE_REL, &home_button_style);
    lv_btn_set_action(home_button_values, LV_BTN_ACTION_CLICK, home_screen);

    lv_btn_set_action(reset, LV_BTN_ACTION_CLICK, reset_values);
    pros::delay(5);
    }
  },
  nullptr,"VALUES FOREVER");
  return LV_RES_OK;   /*The button is not deleted*/
}
  //-------------------------------------------------------------


  void auto_selecter (void*ignore)
  {
  lv_scr_load(scr1);
  // while (true)
  // {
  lv_obj_t * img1 = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img1, &logo);
  lv_obj_align(img1, NULL, LV_ALIGN_CENTER, 0, 0);
  
  switcher = 0;
  lv_style_copy(&red_button_style, &lv_style_plain);
  red_button_style.body.main_color = LV_COLOR_MAKE(200, 0, 0);
  red_button_style.body.grad_color = LV_COLOR_MAKE(200, 0, 0);
  red_button_style.body.radius = LV_RADIUS_CIRCLE;
  red_button_style.text.color = LV_COLOR_MAKE(0, 0, 0);

  lv_style_copy(&blue_button_style, &lv_style_plain);
  blue_button_style.body.main_color = LV_COLOR_MAKE(0, 0, 200);
  blue_button_style.body.grad_color = LV_COLOR_MAKE(0, 0, 200);
  blue_button_style.body.radius = LV_RADIUS_CIRCLE;
  blue_button_style.text.color = LV_COLOR_MAKE(0, 0, 0);

  lv_style_copy(&skills_button_style, &lv_style_plain);
  skills_button_style.body.main_color = LV_COLOR_MAKE(153, 0, 255);
  skills_button_style.body.grad_color = LV_COLOR_MAKE(200, 0, 0);
  skills_button_style.body.radius = LV_RADIUS_CIRCLE;
  skills_button_style.text.color = LV_COLOR_MAKE(0, 0, 0);

  lv_style_copy(&values_button_style, &lv_style_plain);
  values_button_style.body.main_color = LV_COLOR_MAKE(0, 0, 0);
  values_button_style.body.grad_color = LV_COLOR_MAKE(0, 0, 0);
  values_button_style.body.radius = LV_RADIUS_CIRCLE;
  values_button_style.text.color = LV_COLOR_MAKE(200, 200, 200);

  lv_obj_set_size(red_tile, 200, 100);
  lv_obj_align(red_tile, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10); // 30 , 10
  lv_label_set_text(red_button_label, "RED");
  lv_btn_set_style(red_tile, LV_BTN_STYLE_REL, &red_button_style);

  lv_obj_set_size(blue_tile, 200, 100);
  lv_obj_align(blue_tile, NULL, LV_ALIGN_IN_TOP_RIGHT, -10, 10); //-20 , 10
  lv_label_set_text(blue_button_label, "BLUE");
  lv_btn_set_style(blue_tile, LV_BTN_STYLE_REL, &blue_button_style);

  lv_obj_set_size(skills, 200, 100);
  lv_obj_align(skills, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 10, -10);
  lv_label_set_text(skills_button_label, "SKILLS");
  lv_btn_set_style(skills, LV_BTN_STYLE_REL, &skills_button_style);

  lv_obj_set_size(testing, 200, 100);
  lv_obj_align(testing, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -10, -10);
  lv_label_set_text(testing_button_label, "TESTING");
  lv_btn_set_style(testing, LV_BTN_STYLE_REL, &skills_button_style);

  lv_obj_set_size(values, 75, 75);
  lv_obj_align(values, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_label_set_text(values_button_label, "VALUES");
  lv_btn_set_style(values, LV_BTN_STYLE_REL, &values_button_style);

  lv_btn_set_action(red_tile, LV_BTN_ACTION_CLICK, red_tile_screen);
  lv_btn_set_action(blue_tile, LV_BTN_ACTION_CLICK, blue_tile_screen);
  lv_btn_set_action(values, LV_BTN_ACTION_CLICK, values_screen);
  lv_btn_set_action(skills, LV_BTN_ACTION_CLICK, skills_auto);
  lv_btn_set_action(testing, LV_BTN_ACTION_CLICK, testing_auto);

  pros::delay(10);
// }
}
