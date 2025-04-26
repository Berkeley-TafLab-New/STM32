
#include "rudder_control.h"

void rudder_init(TIM_HandleTypeDef *htim, uint32_t channel) {
    rudder_htim = htim;
    rudder_channel = channel;
}

void rudder_turn_to(float bearing, float directionBearing) {
    float diff = bearing - directionBearing;

    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;

    float power = sqrtf(fabsf(diff) / 180.0);
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
