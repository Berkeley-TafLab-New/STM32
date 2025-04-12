#ifndef SERVO_CONTROLS_H
#define SERVO_CONTROLS_H

#include "stm32h7xx_hal.h"
// Define servo control constants
//float offset_angle;
//float offset_value;
// Function prototypes
void set_servo_angle(TIM_HandleTypeDef *htim, uint32_t channel, float angle);
//void set_offset_angle();
void set_servo_angle_gradual(TIM_HandleTypeDef *htim, uint32_t channel, float angle);
void copy_wind_pos(TIM_HandleTypeDef *htim, uint32_t channel, float sail_angle);
#endif // SERVO_CONTROLS_H
