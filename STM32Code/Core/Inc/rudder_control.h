#ifndef __RUDDER_CONTROL_H__
#define __RUDDER_CONTROL_H__

#include "main.h"
#include "math.h"

// External variables if needed
extern const float rudder_straight;
extern const float rudder_range;


// Getter/setter functions
float rudder_get_target_angle(void);
void rudder_set_target_angle(float angle);
float rudder_get_straight(void);
float rudder_get_range(void);

// Existing functions
void rudder_init(TIM_HandleTypeDef *htim, uint32_t channel);
void rudder_turn_to(float bearing, float directionBearing);
void rudder_move_to(void);

#endif
