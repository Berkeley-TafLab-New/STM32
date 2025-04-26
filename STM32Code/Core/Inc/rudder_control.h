/*rudder controls.h*/
#pragma once

#include "main.h"
#include "math.h"
#include "stdint.h"




static TIM_HandleTypeDef *rudder_htim = NULL;
static uint32_t rudder_channel = 0;

static float rudder_straight = 90.0;
static float rudder_range = 40.0;

static float rudder_current_angle = 90.0;
static float rudder_target_angle = 90.0;


void rudder_init(TIM_HandleTypeDef *htim, uint32_t channel);
void rudder_turn_to(float bearing, float directionBearing);
void rudder_move_to(void);
