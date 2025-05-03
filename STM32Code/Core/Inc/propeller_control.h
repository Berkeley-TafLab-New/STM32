#ifndef INC_PROPELLER_CONTROL_H_
#define INC_PROPELLER_CONTROL_H_

#include "stm32h7xx_hal.h"

void propeller_init(TIM_HandleTypeDef *htim, uint32_t channel);
void propeller_set_speed(float speed); 
float propeller_get_speed(void);
void propeller_stop(void);

#endif 