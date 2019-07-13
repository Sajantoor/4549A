#include "main.h"
#include "motor_sensor_init.h"

void stack(int encoder_units, int speed)
{
stacker.move_absolute(encoder_units, speed);
}
