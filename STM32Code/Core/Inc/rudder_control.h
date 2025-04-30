#ifndef __RUDDER_CONTROL_H__
#define __RUDDER_CONTROL_H__

#include "main.h"
#include "math.h"

// External constants 


// API functions
void rudder_init(TIM_HandleTypeDef *htim, uint32_t channel);
void rudder_set_target_angle(float angle);
float rudder_get_target_angle(void);
float rudder_get_straight(void);
float rudder_get_range(void);

// Smooth update toward target angle
void rudder_update(void); 

void rudder_turn_to(float bearing, float directionBearing);

#endif
