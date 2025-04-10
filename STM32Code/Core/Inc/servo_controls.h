#ifndef SERVO_CONTROLS_H
#define SERVO_CONTROLS_H

#include "stm32h7xx_hal.h"
// Define servo control constants
//#define SERVO_MIN_PULSE_WIDTH 500   // Minimum pulse width in microseconds
//#define SERVO_MAX_PULSE_WIDTH 2500  // Maximum pulse width in microseconds
//#define SERVO_FREQUENCY 50          // Servo PWM frequency in Hz

// Function prototypes
void set_servo_angle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle);

#endif // SERVO_CONTROLS_H