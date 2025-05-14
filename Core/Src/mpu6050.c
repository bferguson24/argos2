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

    mpu->imu_calibration_state = STATE_SAMPLE_DATA; 
    mpu->sample_idx = 0; 
    mpu->sample_count = 5; 

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
        uint8_t command[2] = {MPU6050_INT_CONFIG, ((1 << INT_RD_CLEAR_OFFS) | (1 << INT_LATCH_EN_OFFS))};
        HAL_I2C_Master_Transmit(hi2c, mpu->device_address << 1, command, 2, HAL_MAX_DELAY);
    }

    {
        uint8_t start_command[2] = {MPU6050_INT_ENABLE, 1};   
        HAL_I2C_Master_Transmit(hi2c, mpu->device_address << 1, start_command, 2, HAL_MAX_DELAY); 
    }

    {
        uint8_t command[2] = {MPU6050_CONFIG, 1};
        HAL_I2C_Master_Transmit(hi2c, mpu->device_address << 1, command, 2, HAL_MAX_DELAY);
    }

    //Gyro Scale Set: //(250deg/s)
    {
        uint8_t command[2] = {MPU6050_GYRO_CONFIG, (0<< FS_SEL_OFFS)};
        HAL_I2C_Master_Transmit(hi2c, mpu->device_address << 1, command, 2, HAL_MAX_DELAY);
    }

    //Accel Scale Set: (2g)
    {
        uint8_t command[2] = {MPU6050_ACCEL_CONFIG, (0<< AFS_SEL_OFFS)};
        HAL_I2C_Master_Transmit(hi2c, mpu->device_address << 1, command, 2, HAL_MAX_DELAY);
    }
    
}

float mpu6050_elapsed_time(mpu6050_t *mpu){
    //Return the time in [s] between reads; 
    uint32_t delta_count = mpu->curr_cnt - mpu->prev_cnt; 
    float time_s = delta_count * MPU6050_TIMER_SCALE_SECONDS; 
    mpu->dt = time_s; 
    return time_s; 
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

void mpu6050_calc_units(mpu6050_t *imu){





    float dt = mpu6050_elapsed_time(imu); 

    //Calculate Angular Velocities
    float wx = (imu->data.gx - imu->gyro_offset[0]) / MPU6050_GYRO_SCALE; 
    float wy = (imu->data.gy- imu->gyro_offset[1]) / MPU6050_GYRO_SCALE; 
    float wz = (imu->data.gz - imu->gyro_offset[2]) / MPU6050_GYRO_SCALE; 

    //Store
    imu->wx = wx;
    imu->wy = wy;
    imu->wz = wz;

    //Euler Angles
    imu->roll  += wx * dt;
    imu->pitch += wy * dt;
    imu->yaw   += wz * dt;

    //Acceleration
    imu->ax = (imu->data.ax - imu->accel_offset[0]) / MPU6050_ACCEL_SCALE; 
    imu->ay = (imu->data.ay - imu->accel_offset[1]) / MPU6050_ACCEL_SCALE; 
    imu->az = (imu->data.az - imu->accel_offset[2]) / MPU6050_ACCEL_SCALE; 
}

void mpu6050_comp_filter(mpu6050_t *imu, float alpha){
    // float accel_pitch = atan2f(imu->data.ay), float)
}

void mpu6050_task(mpu6050_t *imu, I2C_HandleTypeDef *hi2c){
    if (imu->data_ready_flag != true)
        return; 
    imu->data_ready_flag = false; 

    mpu6050_read_data(hi2c, imu); 
    mpu6050_elapsed_time(imu); 

    switch(imu->imu_calibration_state){
        case(STATE_START_TIME):
            imu->calibration_start_time = TIM2->CNT; 

            imu->sample_idx = 0;
            imu->gyro_cal_sum[0] = 0;
            imu->gyro_cal_sum[1] = 0;
            imu->gyro_cal_sum[2] = 0;

            imu->accel_offset[0] = 0;
            imu->accel_offset[1] = 0;
            imu->accel_offset[2] = 0;

            imu->imu_calibration_state = STATE_WAIT_TIME; 
            break; 
        case(STATE_WAIT_TIME):
        
            float time_elapsed = (TIM2->CNT - imu->calibration_start_time) * MPU6050_TIMER_SCALE_SECONDS; 
            if (time_elapsed > MPU6050_SAMPLE_TIME){
                imu->imu_calibration_state = STATE_SAMPLE_DATA; 
                return; 
            break;
        }
      

        //Get offsets
        case(STATE_SAMPLE_DATA): 

            //Average Offsets once filled
            if (imu->sample_idx >= imu->sample_count){

                imu->gyro_offset[0] = imu->gyro_cal_sum[0] / imu->sample_count; 
                imu->gyro_offset[1] = imu->gyro_cal_sum[1] / imu->sample_count; 
                imu->gyro_offset[2] = imu->gyro_cal_sum[2] / imu->sample_count; 

                imu->accel_offset[0] /= imu->sample_count; 
                imu->accel_offset[1] /= imu->sample_count; 
                imu->accel_offset[2] /= imu->sample_count; 

                imu->imu_calibration_state = STATE_SAMPLE_COMPLETE; 
                return;
            }
            else{
            //Sum offsets
                imu->gyro_cal_sum[0] += imu->data.gx; 
                imu->gyro_cal_sum[1] += imu->data.gy; 
                imu->gyro_cal_sum[2] += imu->data.gz; 

                imu->accel_offset[0] += imu->data.ax; 
                imu->accel_offset[1] += imu->data.ay; 
                imu->accel_offset[2] += imu->data.az; 

                imu->imu_calibration_state = STATE_WAIT_TIME; 
                imu->calibration_start_time = TIM2->CNT;
                imu->sample_idx++; 
            }
            break; 

        case(STATE_SAMPLE_COMPLETE):
    
            mpu6050_calc_units(imu);       
            break;             
            
        default:
            break; 
    }
} 
