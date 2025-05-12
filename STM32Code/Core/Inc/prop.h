/*
 * prop.h
 *
 *  Created on: May 1, 2025
 *      Author: arnav
 */

#ifndef PROP_H_
#define PROP_H_

#include "stm32h7xx_hal.h"
#include "stdint.h"

#define ESC_PULSE_MIN 1000  // 1000us = 1ms (minimum throttle)
#define ESC_PULSE_MAX 2000 // 2000us = 2ms (maximum throttle)
#define ESC_PULSE_RANGE (ESC_PULSE_MAX - ESC_PULSE_MIN)
#define PROPELLER_SPEED_STEP 5.0f


void Propeller_Arm(TIM_HandleTypeDef *htim, uint32_t channel);


void Propeller_SetPulseWidth(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_us);
void Propeller_SetSpeed(TIM_HandleTypeDef *htim, uint32_t channel, float percentage);
void Propeller_IncreaseSpeed(TIM_HandleTypeDef *htim, uint32_t channel);
void Propeller_DecreaseSpeed(TIM_HandleTypeDef *htim, uint32_t channel);

//extern  float current_propeller_speed_percent;
static float current_propeller_speed_percent = 0.0f;
#endif /* PROP_H_ */
