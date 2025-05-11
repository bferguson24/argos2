#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

// REGISTER ADDRESSES
#pragma once

// Self-Test Registers
#define MPU6050_SELF_TEST_X        0x0D
#define MPU6050_SELF_TEST_Y        0x0E
#define MPU6050_SELF_TEST_Z        0x0F
#define MPU6050_SELF_TEST_A        0x10

// Offset Registers
#define MPU6050_XG_OFFSET_H        0x13
#define MPU6050_XG_OFFSET_L        0x14
#define MPU6050_YG_OFFSET_H        0x15
#define MPU6050_YG_OFFSET_L        0x16
#define MPU6050_ZG_OFFSET_H        0x17
#define MPU6050_ZG_OFFSET_L        0x18

// Configuration Registers
#define MPU6050_SMPLRT_DIV         0x19
#define MPU6050_CONFIG             0x1A
#define MPU6050_GYRO_CONFIG        0x1B
#define MPU6050_ACCEL_CONFIG       0x1C
#define MPU6050_ACCEL_CONFIG_2     0x1D
#define MPU6050_LP_ACCEL_ODR       0x1E

// Motion Detection
#define MPU6050_WOM_THR            0x1F
#define MPU6050_FIFO_EN            0x23

// I2C Master Control
#define MPU6050_I2C_MST_CTRL       0x24
#define MPU6050_I2C_SLV0_ADDR      0x25
#define MPU6050_I2C_SLV0_REG       0x26
#define MPU6050_I2C_SLV0_CTRL      0x27
#define MPU6050_I2C_SLV1_ADDR      0x28
#define MPU6050_I2C_SLV1_REG       0x29
#define MPU6050_I2C_SLV1_CTRL      0x2A
#define MPU6050_I2C_SLV2_ADDR      0x2B
#define MPU6050_I2C_SLV2_REG       0x2C
#define MPU6050_I2C_SLV2_CTRL      0x2D
#define MPU6050_I2C_SLV3_ADDR      0x2E
#define MPU6050_I2C_SLV3_REG       0x2F
#define MPU6050_I2C_SLV3_CTRL      0x30
#define MPU6050_I2C_SLV4_ADDR      0x31
#define MPU6050_I2C_SLV4_REG       0x32
#define MPU6050_I2C_SLV4_DO        0x33
#define MPU6050_I2C_SLV4_CTRL      0x34
#define MPU6050_I2C_SLV4_DI        0x35

// Sensor Output Registers
#define MPU6050_ACCEL_XOUT_H       0x3B
#define MPU6050_ACCEL_XOUT_L       0x3C
#define MPU6050_ACCEL_YOUT_H       0x3D
#define MPU6050_ACCEL_YOUT_L       0x3E
#define MPU6050_ACCEL_ZOUT_H       0x3F
#define MPU6050_ACCEL_ZOUT_L       0x40
#define MPU6050_TEMP_OUT_H         0x41
#define MPU6050_TEMP_OUT_L         0x42
#define MPU6050_GYRO_XOUT_H        0x43
#define MPU6050_GYRO_XOUT_L        0x44
#define MPU6050_GYRO_YOUT_H        0x45
#define MPU6050_GYRO_YOUT_L        0x46
#define MPU6050_GYRO_ZOUT_H        0x47
#define MPU6050_GYRO_ZOUT_L        0x48

//FIFO
#define MPU6050_FIFO_COUNTH 0x72
#define MPU6050_FIFO_COUNTL 0x73
#define MPU6050_FIFO_R_W    0x74
#define MPU6050_INT_STATUS 0x3A
#define MPU6050_INT_CONFIG 0x37
#define MPU6050_INT_ENABLE 0x38

// Power Management
#define MPU6050_PWR_MGMT_1         0x6B
#define MPU6050_PWR_MGMT_2         0x6C

// Device ID
#define MPU6050_WHO_AM_I           0x75

#define INT_RD_CLEAR_OFFS 4
#define INT_LATCH_EN_OFFS 5


typedef struct __attribute__((packed)){
    int16_t ax; 
    int16_t ay; 
    int16_t az; 
    int16_t temp;
    int16_t gx; 
    int16_t gy; 
    int16_t gz; 

}imu_data_t;


typedef struct {
    uint16_t device_address; 
    imu_data_t data; 

}mpu6050_t; 

HAL_StatusTypeDef mpu6050_read_data(I2C_HandleTypeDef *hi2c, mpu6050_t *mpu); 
void mpu6050_init(I2C_HandleTypeDef *hi2c, mpu6050_t *mpu); 


