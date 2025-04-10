/*
 * AS5600.h
 *  Magnetic encoder header file
 *  Created on: Apr 5, 2025
 *      Author: arnav
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include "stm32h7xx_hal.h"

#define AS5600_ADRESS 0x36 // 7-bit I2C address
#define AS5600_RAW_ANGLE_REG   0x0C    // Raw angle register (12-bit)

HAL_StatusTypeDef AS5600_read_angle(I2C_HandleTypeDef *hi2c, float *angle);
//HAL_StatusTypeDef AS5600_config_ZPOS(I2C_HandleTypeDef *hi2c, float *angle); //implement configs if needed 
uint8_t check_magnet_presence(I2C_HandleTypeDef *hi2c);
void handle_error(HAL_StatusTypeDef status);
#endif /* INC_AS5600_H_ */
