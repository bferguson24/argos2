#pragma once

#include "mpu6050.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"
#include <math.h>
#include <string.h>
#include <sys/types.h>
#include "utility.h"

extern I2C_HandleTypeDef hi2c1;


// read_fifo(uint16_t* buffer, int n_bytes){

// 	for (int i = 0; i < n_bytes; i++){
// 		HAL_I2C_
// 	} 


// }

void mpu6050_init(I2C_HandleTypeDef *hi2c, mpu6050_t *mpu){

    {
        uint8_t reset_command[2] = {MPU6050_PWR_MGMT_1, 0x80};
        HAL_I2C_Master_Transmit(hi2c, mpu->device_address << 1, reset_command, 2, HAL_MAX_DELAY);
    }

    HAL_Delay(100);
    {

        uint8_t start_command[2] = {MPU6050_PWR_MGMT_1, 0};
        HAL_I2C_Master_Transmit(hi2c, mpu->device_address << 1, start_command, 2, HAL_MAX_DELAY); 
    }

       {
        uint8_t start_command[2] = {MPU6050_INT_ENABLE, 1};   
        HAL_I2C_Master_Transmit(hi2c, mpu->device_address << 1, start_command, 2, HAL_MAX_DELAY); 
    }
}


HAL_StatusTypeDef mpu6050_read_data(I2C_HandleTypeDef *hi2c, mpu6050_t *mpu){
    imu_data_t temp; 

    //Burst fifo data read, store in buffer
    if (HAL_I2C_Mem_Read(hi2c, mpu->device_address << 1, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&temp, sizeof(imu_data_t), HAL_MAX_DELAY)){
        return HAL_ERROR; 
    };

    mpu->data.ax = flip_endianess(temp.ax);
    mpu->data.ay = flip_endianess(temp.ay);
    mpu->data.az = flip_endianess(temp.az);
    mpu->data.temp = flip_endianess(temp.temp);
    mpu->data.gx = flip_endianess(temp.gx);
    mpu->data.gy = flip_endianess(temp.gy);
    mpu->data.gz = flip_endianess(temp.gz);
    return HAL_OK;
}




HAL_StatusTypeDef read_fifo(I2C_HandleTypeDef *hi2c, uint8_t device_address, uint8_t register_address, uint8_t *data, uint16_t size) {
    if (HAL_I2C_Master_Transmit(hi2c, (device_address << 1), &register_address, 1, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_I2C_Master_Receive(hi2c, (device_address << 1), data, size, HAL_MAX_DELAY);
}