#ifndef LCD_H
#define LCD_H
#include "main.h"

void auto_selecter(void*ignore);
extern pros::task_t auto_selecter_task;

// void front_middle_auto();

extern int switcher;
extern float timerAuto;

#endif
