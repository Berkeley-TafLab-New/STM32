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
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f; //limits
    
    float min_pulse = 500.0f;
    float max_pulse = 2500.0f;
     
    uint32_t pulse_width = min_pulse +(angle*((max_pulse - min_pulse)/180.0f));
    uint32_t rounded_pulse_width = (uint32_t)(pulse_width + 0.5f); 
    __HAL_TIM_SET_COMPARE(htim, channel, rounded_pulse_width);
  
  }
  void set_servo_angle_gradual(TIM_HandleTypeDef *htim, uint32_t channel, float angle){
    /*set a servo angle by taking the channel angle and tim and the channel*/
     uint32_t current_pulse_width = __HAL_TIM_GET_COMPARE(htim, channel); // returns current pulse width(between 500 and 2500)
     uint32_t desired_pulse_width = 500 +(angle*((2500-500)/180));
     while (desired_pulse_width != current_pulse_width){
        if (desired_pulse_width > current_pulse_width){
            current_pulse_width++; 
            __HAL_TIM_SET_COMPARE(htim, channel, current_pulse_width);
            HAL_Delay(10);
            current_pulse_width = __HAL_TIM_GET_COMPARE(htim, channel);

        }
        else if (desired_pulse_width < current_pulse_width)
        {
            current_pulse_width--; 
            __HAL_TIM_SET_COMPARE(htim, channel, current_pulse_width);

            HAL_Delay(10);
            current_pulse_width = __HAL_TIM_GET_COMPARE(htim, channel); 
        }
      
    }
  }
  void copy_wind_pos(ServoController *ctrl, float wind_angle){
    float adj_angle=  fmod(wind_angle, 180.0f);  //all opposite angles are the same so 200 degrees = 20 degrees 
    
    // explicit case for 180 degrees
    if(wind_angle >= 180.0f && adj_angle == 0.0f) {
      adj_angle = 180.0f;
    }
    
    set_servo_angle(ctrl->htim, ctrl->channel, adj_angle);
  

    //set_servo_angle();
  }
//ence set servo angle to 90 from there it has 90 degrees of play in either direction

