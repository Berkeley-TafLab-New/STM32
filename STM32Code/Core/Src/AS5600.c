/*
 * AS5600.c
 *
 * function definitions for the Magnetic Encoder
 * it is just a potentiometer that can stores a 12 bit raw angle (4096 positions)
 *  Created on: Apr 5, 2025
 *      Author: Arnav
 */

#include "AS5600.h"
#include "stdint.h"

HAL_StatusTypeDef AS5600_read_angle(I2C_HandleTypeDef *hi2c, float *angle){
    
    HAL_StatusTypeDef ret ;
    uint8_t angle_buff[2]; // hold the 2 bits from the Raw Angle 
    ret = HAL_I2C_Mem_Read(hi2c, (AS5600_ADRESS<<1), AS5600_SCALED_ANGLE_REG, I2C_MEMADD_SIZE_8BIT,angle_buff,2,HAL_MAX_DELAY);//HAL_MAX_DELAY is blockling
    if (ret != HAL_OK){
    	handle_error(ret);
    }
    uint16_t raw_angle = (((angle_buff[0]<<8)|angle_buff[1])&0x0FFF);

    *angle = 360.0f*((float)raw_angle/4096.0f);
    return HAL_OK;
}

uint8_t check_magnet_presence(I2C_HandleTypeDef *hi2c) {
    uint8_t status = 0;
    HAL_StatusTypeDef result;
    result = HAL_I2C_Mem_Read(hi2c, AS5600_ADRESS << 1, 0x0B, I2C_MEMADD_SIZE_8BIT, &status, 1, HAL_MAX_DELAY);

    if (result != HAL_OK) {
        printf("I2C communication error\n");
        return 0; // Return 0 to indicate error
    }

    // Check the MD (Magnet Detected) bit (bit 5 of STATUS register)
    if ((status & (1 << 5)) != 0) {
        //printf("Magnet detected with proper field strength\n");
        return 1; // Magnet is detected
    } else {
        printf("No magnet detected or improper alignment\n");
        return 0; // Magnet is not detected
    }
}

HAL_StatusTypeDef AS5600_config_ZPOS(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef ret;
    uint8_t angle_read_buff[2]; // Buffer for reading raw angle

    // Read current raw angle (2 bytes) from 0x0C
    ret = HAL_I2C_Mem_Read(hi2c, (AS5600_ADRESS << 1), AS5600_RAW_ANGLE_REG,
                          I2C_MEMADD_SIZE_8BIT, angle_read_buff, 2, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        handle_error(ret);
        return ret;
    }

    // Extract 12-bit raw angle: angle_read_buff[0] is high, angle_read_buff[1] is low
    uint16_t raw_angle = (((uint16_t)angle_read_buff[0] << 8) | angle_read_buff[1]) & 0x0FFF;

    // Prepare ZPOS register values
    uint8_t zpos_data[2];
    zpos_data[0] = (uint8_t)((raw_angle >> 8) & 0x0F); // High byte for ZPOS_HI (Reg 0x01)
    zpos_data[1] = (uint8_t)(raw_angle & 0xFF);       // Low byte for ZPOS_LO (Reg 0x02)

    // Write High byte to ZPOS_HI register (0x01)
    ret = HAL_I2C_Mem_Write(hi2c, (AS5600_ADRESS << 1), AS5600_ZPOS_HI_REG,
                           I2C_MEMADD_SIZE_8BIT, &zpos_data[0], 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        handle_error(ret);
        return ret;
    }

    // Write Low byte to ZPOS_LO register (0x02)
    ret = HAL_I2C_Mem_Write(hi2c, (AS5600_ADRESS << 1), AS5600_ZPOS_LO_REG,
                           I2C_MEMADD_SIZE_8BIT, &zpos_data[1], 1, HAL_MAX_DELAY);

    if (ret != HAL_OK) {
        handle_error(ret);
    }

    // Optional: Small delay might be needed for sensor processing
    // HAL_Delay(10);

    return ret; // Return the status of the last write operation
}




void handle_error(HAL_StatusTypeDef status) {
    if (status != HAL_OK) {
        // Implement error handling logic (e.g., log error or reset I²C bus)
        printf("I2C Error: %d\n", status);
    }
}
/*HAL_StatusTypeDef AS5600_init(I2C_HandleTypeDef *hi2c, uint8_t address) {
    // Example: Write configuration registers (if needed)
    uint8_t config_data = 0x00; // Replace with actual configuration
    return HAL_I2C_Mem_Write(hi2c, (address << 1), AS5600_CONFIG_REG,
                             I2C_MEMADD_SIZE_8BIT, &config_data, 1, HAL_MAX_DELAY);
}*/ //Implement later ...
