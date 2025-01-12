#ifndef HOMING_H
#define HOMING_H
#include "motor.h"
#include "limit_switch.h"

void home_all_motors(motor_t* m1, motor_t* m2, limit_switch_t* left_limit_switch, limit_switch_t* right_limit_switch, limit_switch_t* m1_limit_switch);
#endif