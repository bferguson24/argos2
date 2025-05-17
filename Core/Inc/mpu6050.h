#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "filter.h"
#include "timing.h"

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

//Bit Offsets
#define INT_RD_CLEAR_OFFS 4
#define INT_LATCH_EN_OFFS 5
#define FS_SEL_OFFS 3 
#define AFS_SEL_OFFS 3 

//Scaling Constants:
#define MPU6050_TIMER_SCALE_SECONDS 1e-6f //[s]
#define MPU6050_FS_SEL0 250.0f //[deg/s]
#define MPU6050_AFS_SEL 2.0f //[g]
#define MPU6050_GYRO_SCALE 131.0f //[LSB/ (deg/s)]
#define MPU6050_ACCEL_SCALE 16384.0f // [LSB / g]
#define MPU6050_SAMPLE_TIME 0.2f
#define RAD_TO_DEG_SCALE 57.2957795131




typedef struct __attribute__((packed)){
    int16_t ay; 
    int16_t ax; 
    int16_t az; 
    int16_t temp;
    int16_t gy; 
    int16_t gx; 
    int16_t gz; 

}imu_data_t;

typedef enum{
    STATE_IMU_INIT,
    STATE_RESET_OFFSETS, 
    STATE_TIME_START,
    STATE_TIME_WAIT,
    STATE_SAMPLE_OFFSET,
    STATE_SAMPLE_DATA, 
}mpu6050_calibrate_t; 


typedef struct {

    //Info & Status
    I2C_HandleTypeDef *hi2c; 
    uint16_t device_address; 
    bool data_ready_flag; 
    mpu6050_calibrate_t status; 
    uint32_t calibration_start_time; 
    uint32_t curr_cnt;
    uint32_t prev_cnt;

    //Raw Data & Offsets
    imu_data_t data; 
    int16_t sample_count; 
    int16_t sample_idx; 
    int32_t gyro_cal_sum[3]; 
    float accel_cal_sum[3]; // Add this to your struct

    int32_t gyro_offset[3];
    int32_t accel_offset[3];    

    //Dimensioned Data
    timing_t *timer; 
    float alpha_fusion; 
    float dt; 

    float wx;
    float wy;
    float wz;

    float roll;
    float pitch;
    float yaw; 

    float accel_pitch;
    float accel_roll;

    float ax;
    float ay;
    float az; 

    moving_avg_t *ax_filtered;
    moving_avg_t *ay_filtered;
    moving_avg_t *az_filtered;

}mpu6050_t; 

HAL_StatusTypeDef mpu6050_read_data(mpu6050_t *mpu); 
void mpu6050_init(mpu6050_t *imu); 
void mpu6050_task(mpu6050_t *imu); 
float mpu6050_elapsed_time(mpu6050_t *mpu); 
void mpu6050_gyro_to_angle(mpu6050_t *imu); 
void mpu6050_comp_filter(mpu6050_t *imu); 


