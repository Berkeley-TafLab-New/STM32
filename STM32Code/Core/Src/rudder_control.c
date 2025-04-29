#include "rudder_control.h"
#include "servo_controls.h"

TIM_HandleTypeDef *rudder_htim = NULL;
uint32_t rudder_channel = 0;
float rudder_target_angle = 90.0f;
float rudder_current_angle = 90.0f;
const float rudder_straight = 90.0f;
const float rudder_range = 45.0f;

void rudder_init(TIM_HandleTypeDef *htim, uint32_t channel) {
    rudder_htim = htim;
    rudder_channel = channel;
}

void rudder_turn_to(float bearing, float directionBearing) {
    float diff = bearing - directionBearing;

    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;

    float power = sqrtf(fabsf(diff) / 180.0f);
    if (diff > 0) {
        rudder_target_angle = rudder_straight - power * rudder_range;
    } else if (diff < 0) {
        rudder_target_angle = rudder_straight + power * rudder_range;
    } else {
        rudder_target_angle = rudder_straight;
    }
}

void rudder_move_to(void) {
    if (fabsf(rudder_target_angle - rudder_current_angle) > 1) {
        if (rudder_current_angle < rudder_target_angle) {
            rudder_current_angle += 1;
        } else {
            rudder_current_angle -= 1;
        }

        uint32_t pulse_width = 500 + (rudder_current_angle * (2000.0f / 180.0f));
        __HAL_TIM_SET_COMPARE(rudder_htim, rudder_channel, pulse_width);
    }
}

float rudder_get_target_angle(void) {
    return rudder_target_angle;
}

void rudder_set_target_angle(float angle) {
    rudder_target_angle = angle;
}

float rudder_get_straight(void) {
    return rudder_straight;
}

float rudder_get_range(void) {
    return rudder_range;
}
