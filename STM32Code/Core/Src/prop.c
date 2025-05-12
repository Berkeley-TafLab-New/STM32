#include "prop.h"
#include "stdio.h"






void Propeller_SetPulseWidth(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_us) {
    // Clamp the value to the valid ESC range
    if (pulse_us < ESC_PULSE_MIN) {
        pulse_us = ESC_PULSE_MIN;
    } else if (pulse_us > ESC_PULSE_MAX) {
        pulse_us = ESC_PULSE_MAX;
    }
    // Set the Compare value for TIM2 Channel 1

    __HAL_TIM_SET_COMPARE(htim, channel, pulse_us);
    
}


void Propeller_SetSpeed(TIM_HandleTypeDef *htim, uint32_t channel, float percentage) {
    if (percentage < 0.0f) percentage = 0.0f;
    if (percentage > 100.0f) percentage = 100.0f;

    // Map percentage to pulse width
    current_propeller_speed_percent = percentage;
    uint16_t pulse_us = ESC_PULSE_MIN + (uint16_t)((percentage / 100.0f) * ESC_PULSE_RANGE);
    
    Propeller_SetPulseWidth(htim, channel, pulse_us);
}

void Propeller_IncreaseSpeed(TIM_HandleTypeDef *htim, uint32_t channel) {
    float new_speed = current_propeller_speed_percent + PROPELLER_SPEED_STEP;
    Propeller_SetSpeed(htim, channel, new_speed);
}

void Propeller_DecreaseSpeed(TIM_HandleTypeDef *htim, uint32_t channel) {
    float new_speed = current_propeller_speed_percent - PROPELLER_SPEED_STEP;
    Propeller_SetSpeed(htim, channel, new_speed); // SetSpeed handles clamping and feedback
}


 void Propeller_Arm(TIM_HandleTypeDef *htim, uint32_t channel) {

     printf("Arming ESC... Sending min throttle.\r\n");
     Propeller_SetPulseWidth(htim,channel,ESC_PULSE_MIN);
     HAL_Delay(2000); // Wait 2 seconds for ESC to initialize (adjust if needed)
     printf("ESC should be armed now.\r\n");
 }
