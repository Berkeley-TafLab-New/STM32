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
#define AS5600_ZPOS_HI_REG 0x01
#define AS5600_ZPOS_LO_REG 0x02

HAL_StatusTypeDef AS5600_read_angle(I2C_HandleTypeDef *hi2c, float *angle);
//HAL_StatusTypeDef AS5600_config_ZPOS(I2C_HandleTypeDef *hi2c, float *angle); //implement configs if needed 
uint8_t check_magnet_presence(I2C_HandleTypeDef *hi2c);
void handle_error(HAL_StatusTypeDef status);

HAL_StatusTypeDef AS5600_calibrate_zero(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef ret;
    uint8_t angle_buff;
    
    // Read current raw angle
    ret = HAL_I2C_Mem_Read(hi2c, (AS5600_ADRESS<<1), AS5600_RAW_ANGLE_REG, 
                          I2C_MEMADD_SIZE_8BIT, angle_buff, 2, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        handle_error(ret);
        return ret;
    }
    
    // Extract 12-bit raw angle
    uint16_t raw_angle = ((angle_buff << 8) | angle_buff) & 0x0FFF;
    
    // Split into ZPOS register values
    uint8_t zpos_data = {
        (uint8_t)((raw_angle >> 8) & 0x0F),  // High byte (4 bits)
        (uint8_t)(raw_angle & 0xFF)          // Low byte (8 bits)
    };
    
    // Write to ZPOS registers
    ret = HAL_I2C_Mem_Write(hi2c, (AS5600_ADRESS<<1), AS5600_ZPOS_HI_REG,
                           I2C_MEMADD_SIZE_8BIT, &zpos_data, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;
    
    ret = HAL_I2C_Mem_Write(hi2c, (AS5600_ADRESS<<1), AS5600_ZPOS_LO_REG,
                           I2C_MEMADD_SIZE_8BIT, &zpos_data, 1, HAL_MAX_DELAY);
    return ret;
    
    }

#endif /* INC_AS5600_H_ */
