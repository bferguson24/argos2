#pragma once

#include "pca9685.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"
#include <math.h>
#include <string.h>
#include <sys/types.h>


extern I2C_HandleTypeDef hi2c1;



bool PCA9685_Init(){
   

    // Reset the PCA9685
    uint8_t mode1 = MODE1_SLEEP; // Enter sleep mode to set prescaler
    if (PCA9685_Write(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_MODE1, &mode1, 1) != HAL_OK) {
        return false;
    }

    // Enable auto-increment for registers and restart
    mode1 = MODE1_AI | MODE1_RESTART;
    if (PCA9685_Write(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_MODE1, &mode1, 1) != HAL_OK) {
        return false;
    }

    // Set default MODE2
    uint8_t mode2 = MODE2_OUTDRV;
    if (PCA9685_Write(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_MODE2, &mode2, 1) != HAL_OK) {
        return false;
    }

    return true;
}



bool PCA9685_SetPWMFrequency(uint16_t frequency) {
    if (frequency < 24 || frequency > 1526) {
        return false; // Invalid frequency 
    }

    // Calculate prescaler value
    float prescale_val = ((float)FREQUENCY_OSCILLATOR / (frequency * 4096.0)) - 1;
    uint8_t prescale = (uint8_t)(floor(prescale_val + 0.5));

    // Set to sleep mode to update prescaler
    uint8_t mode1;
    if (PCA9685_Read(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_MODE1, &mode1, 1) != HAL_OK) {
        return false;
    }

    uint8_t old_mode = mode1;
    mode1 = (old_mode & ~MODE1_RESTART) | MODE1_SLEEP;
    if (PCA9685_Write(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_MODE1, &mode1, 1) != HAL_OK) {
        return false;
    }

    // Write prescaler
    if (PCA9685_Write(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_PRESCALE, &prescale, 1) != HAL_OK) {
        return false;
    }

    // Restore mode and enable restart
    if (PCA9685_Write(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_MODE1, &old_mode, 1) != HAL_OK) {
        return false;
    }
    HAL_Delay(1); // Delay for oscillator stabilization
    old_mode |= MODE1_RESTART;
    if (PCA9685_Write(&hi2c1, PCA9685_I2C_ADDRESS, PCA9685_MODE1, &old_mode, 1) != HAL_OK) {
        return false;
    }

    return true;
}


HAL_StatusTypeDef PCA9685_Write(I2C_HandleTypeDef *hi2c, uint8_t device_address, uint8_t register_address, uint8_t *data, uint16_t size) {
    uint8_t buffer[size + 1];
    buffer[0] = register_address;
    memcpy(&buffer[1], data, size);
    return HAL_I2C_Master_Transmit(hi2c, (device_address << 1), buffer, size + 1, HAL_MAX_DELAY);
}




HAL_StatusTypeDef PCA9685_Read(I2C_HandleTypeDef *hi2c, uint8_t device_address, uint8_t register_address, uint8_t *data, uint16_t size) {
    if (HAL_I2C_Master_Transmit(hi2c, (device_address << 1), &register_address, 1, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_I2C_Master_Receive(hi2c, (device_address << 1), data, size, HAL_MAX_DELAY);
}


HAL_StatusTypeDef PCA9685_LED_Status(u_int8_t MemAddress, uint8_t memAddSize){

    uint8_t data[4];

        volatile int status = HAL_I2C_Mem_Read(&hi2c1, PCA9685_I2C_ADDRESS, MemAddress, memAddSize, data, 4, HAL_MAX_DELAY);
    if (status == HAL_OK){

        volatile uint16_t on = (uint8_t)data[1] << 8 | data[0]; 
        volatile uint16_t off = (uint8_t)data[3] << 8 | data[2]; 
        
        return HAL_OK;
    } else {
        return HAL_ERROR;
    }


}

HAL_StatusTypeDef setPWM(uint8_t num, uint16_t on, uint16_t off) {
    uint8_t buffer[5];
    
    // Set the register address for the correct LED (0-15)
    buffer[0] = PCA9685_LED0_ON_L + 4 * num;
    
   //Buffer Data
    buffer[1] = (uint8_t)(on & 0xFF);       
    buffer[2] = (uint8_t)((on >> 8) & 0xFF); 
    buffer[3] = (uint8_t)(off & 0xFF);       
    buffer[4] = (uint8_t)((off >> 8) & 0xFF); 

    //Send Buffer
       volatile int err = HAL_I2C_Master_Transmit(&hi2c1, PCA9685_I2C_ADDRESS << 1, buffer, 5, HAL_MAX_DELAY);
    if (err == HAL_OK) {
        return HAL_OK;  
    } else {
        // set a breakpoint below and look at err's value and match it against the I2C-specific error codes documented in the HAL. Alternatively, test for those cases if you want to.
        return HAL_ERROR;  
    }

}


   