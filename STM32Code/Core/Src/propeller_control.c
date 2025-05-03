#include "propeller_control.h"

static TIM_HandleTypeDef *prop_htim = NULL;
static uint32_t prop_channel = 0;
static float prop_speed_percent = 0.0f; 

void propeller_init(TIM_HandleTypeDef *htim, uint32_t channel) {
    prop_htim = htim;
    prop_channel = channel;
    prop_speed_percent = 0.0f;
    __HAL_TIM_SET_COMPARE(prop_htim, prop_channel, 0);
}

void propeller_set_speed(float speed) {
    if (speed < 0.0f) 
        speed = 0.0f;
    if (speed > 100.0f) 
        speed = 100.0f;
        
    prop_speed_percent = speed;

    uint32_t max_pulse = __HAL_TIM_GET_AUTORELOAD(prop_htim);
    uint32_t pulse = (uint32_t)((speed / 100.0f) * max_pulse);
    __HAL_TIM_SET_COMPARE(prop_htim, prop_channel, pulse);
}

float propeller_get_speed(void) {
    return prop_speed_percent;
}

void propeller_stop(void) {
    propeller_set_speed(0.0f);
}
