/*
 * servo_controls.c
 *
 * function definitions for the servos
 * Use to control sail and rudder servos
 *  Created on: Apr 9, 2025
 *      Author: Arnav
 */

#include "servo_controls.h"
#include "stdint.h"
 void set_servo_angle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle){
    /*set a servo angle by taking the channel angle and tim 
     the maths maybe works like 1.5 ms pulse width is neutral with .5 and 2.5 being either end
     so we have a 1us count so 500us/1ms -> 500 counts and 2500us -> 2500 counts*/
     uint32_t pulse_length = 500 +(angle*((2500-500)/180));
    __HAL_TIM_SET_COMPARE(htim, channel, pulse_length);
  
  }