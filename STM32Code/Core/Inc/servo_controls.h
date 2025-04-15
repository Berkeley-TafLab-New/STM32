#ifndef SERVO_CONTROLS_H
#define SERVO_CONTROLS_H

#include "stm32h7xx_hal.h"
// Define servo control constants
//float offset_angle;
//float offset_value;
// Function prototypes


typedef struct {
	uint32_t current_pulse;
    uint32_t target_pulse;
    TIM_HandleTypeDef *htim;
    uint32_t channel;
  	  } ServoController;

void set_servo_angle(TIM_HandleTypeDef *htim, uint32_t channel, float angle);
void set_servo_angle_gradual(TIM_HandleTypeDef *htim, uint32_t channel, float angle);
void copy_wind_pos(ServoController *ctrl, float wind_angle);
#endif // SERVO_CONTROLS_H
