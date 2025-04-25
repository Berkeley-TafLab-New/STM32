#ifndef RUDDER_CONTROL_H
#define RUDDER_CONTROL_H

#include "main.h"

// Initializes rudder control with the timer and channel
void rudder_init(TIM_HandleTypeDef *htim, uint32_t channel);

// Sets the target rudder angle based on current and desired bearings
void rudder_turn_to(float bearing, float directionBearing);

// Moves the rudder toward the target angle
void rudder_move_to(void);

#endif
