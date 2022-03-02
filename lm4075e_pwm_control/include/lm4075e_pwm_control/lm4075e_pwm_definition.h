#ifndef LM4075E_PWM_DEFINITION_H
#define LM4075E_PWM_DEFINITION_H

#include <wiringPi.h>
#include <iostream>
#include <lm4075e_msgs/Int32.h>

#define Left_Pin_PWM 26
#define Left_Pin_DIR 31

#define Right_Pin_PWM 23
#define Right_Pin_DIR 30

int des_pos_l = 0;
int des_pos_r = 0;

int enc_pos_l = 0;
int enc_pos_r = 0;

double Kp;
double Kd;
double Ki;

double pos_error_l = 0;
double pos_error_r = 0;

double pos_prev_error_l = 0;
double pos_prev_error_r = 0;

double dpos_error_l = 0;
double dpos_error_r = 0;

double Ipos_error_l = 0;
double Ipos_error_r = 0;

int control_input_l = 0;
int control_input_r = 0;

int control_dir_l = 0;
int control_dir_r = 0;

int max_pwm = 1000;

#endif
