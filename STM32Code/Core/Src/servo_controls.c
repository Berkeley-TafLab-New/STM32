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

 void set_servo_angle(TIM_HandleTypeDef *htim, uint32_t channel, float angle){
    /*set a servo angle by taking the channel angle and tim 
     the maths maybe works like 1.5 ms pulse width is neutral with .5 and 2.5 being either end
     so we have a 1us count so 500us/1ms -> 500 counts and 2500us -> 2500 counts*/
     uint32_t pulse_width = 500 +(angle*((2500-500)/180));
    __HAL_TIM_SET_COMPARE(htim, channel, pulse_width);
  
  }
  void set_servo_angle_gradual(TIM_HandleTypeDef *htim, uint32_t channel, float angle){
    /*set a servo angle by taking the channel angle and tim and the channel*/
     uint32_t current_pulse_width = __HAL_TIM_GET_COMPARE(htim, channel); // returns current pulse width(between 500 and 2500)
     uint32_t desired_pulse_width = 500 +(angle*((2500-500)/180));
     while (desired_pulse_width != current_pulse_width){
        if (desired_pulse_width > current_pulse_width){
            current_pulse_width++; 
            HAL_Delay(10);
            current_pulse_width = __HAL_TIM_GET_COMPARE(htim, channel); 
        }
        else if (desired_pulse_width < current_pulse_width)
        {
            current_pulse_width--; 
            HAL_Delay(10);
            current_pulse_width = __HAL_TIM_GET_COMPARE(htim, channel); 
        }
      
    }
  }
  void copy_wind_pos(TIM_HandleTypeDef *htim, uint32_t channel, float sail_angle){
    /*the servo motor is placed at 90 degrees to the sail 
    then the the angle can be calculated as such */
    // do this
    
    //set_servo_angle(htim, channel, offset_angle);
  }

  //calibration sequence set servo angle to 90 from there it has 90 degrees of play in either direction

