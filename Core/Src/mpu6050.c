
#include "mpu6050.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"
#include <math.h>
#include <string.h>
#include <sys/types.h>
#include "timing.h"
#include "utility.h"
#include "filter.h"

extern I2C_HandleTypeDef hi2c1;


void mpu6050_init(mpu6050_t *imu){

    imu->status = STATE_SAMPLE_DATA; 
    imu->sample_idx = 0; 
    imu->sample_count = 5; 
    
    imu->gyro_cal_sum[0] = 0;
    imu->gyro_cal_sum[1] = 0;
    imu->gyro_cal_sum[2] = 0;

    imu->accel_cal_sum[0] = 0;
    imu->accel_cal_sum[1] = 0;
    imu->accel_cal_sum[2] = 0;
    
    imu->gyro_offset[0] = 0;
    imu->gyro_offset[1] = 0;
    imu->gyro_offset[2] = 0;

    imu->accel_offset[0] = 0;
    imu->accel_offset[1] = 0;
    imu->accel_offset[2] = 0;

    {
        uint8_t reset_command[2] = {MPU6050_PWR_MGMT_1, 0x80};
        HAL_I2C_Master_Transmit(imu->hi2c, imu->device_address << 1, reset_command, 2, HAL_MAX_DELAY);
    }

    HAL_Delay(100);
    {

        uint8_t start_command[2] = {MPU6050_PWR_MGMT_1, 0};
        HAL_I2C_Master_Transmit(imu->hi2c, imu->device_address << 1, start_command, 2, HAL_MAX_DELAY); 
    }

    {
        uint8_t command[2] = {MPU6050_INT_CONFIG, ((1 << INT_RD_CLEAR_OFFS) | (1 << INT_LATCH_EN_OFFS))};
        HAL_I2C_Master_Transmit(imu->hi2c, imu->device_address << 1, command, 2, HAL_MAX_DELAY);
    }

    {
        uint8_t start_command[2] = {MPU6050_INT_ENABLE, 1};   
        HAL_I2C_Master_Transmit(imu->hi2c, imu->device_address << 1, start_command, 2, HAL_MAX_DELAY); 
    }

    {
        uint8_t command[2] = {MPU6050_CONFIG, 1};
        HAL_I2C_Master_Transmit(imu->hi2c, imu->device_address << 1, command, 2, HAL_MAX_DELAY);
    }

    //Gyro Scale Set: //(250deg/s)
    {
        uint8_t command[2] = {MPU6050_GYRO_CONFIG, (0<< FS_SEL_OFFS)};
        HAL_I2C_Master_Transmit(imu->hi2c, imu->device_address << 1, command, 2, HAL_MAX_DELAY);
    }

    //Accel Scale Set: (2g)
    {
        uint8_t command[2] = {MPU6050_ACCEL_CONFIG, (0<< AFS_SEL_OFFS)};
        HAL_I2C_Master_Transmit(imu->hi2c, imu->device_address << 1, command, 2, HAL_MAX_DELAY);
    }
    imu->status = STATE_RESET_OFFSETS; 
    
}

// float mpu6050_elapsed_time(mpu6050_t *imu){
//     //Return the time in [s] between reads; 
//     uint32_t delta_count = imu->curr_cnt - imu->prev_cnt; 
//     float time_s = delta_count * MPU6050_TIMER_SCALE_SECONDS; 
//     imu->dt = time_s; 
//     return time_s; 
// }

HAL_StatusTypeDef mpu6050_read_data(mpu6050_t *imu){
    imu_data_t temp; 

    //Burst fifo data read, store in buffer
    if (HAL_I2C_Mem_Read(imu->hi2c, imu->device_address << 1, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&temp, sizeof(imu_data_t), HAL_MAX_DELAY)){
        return HAL_ERROR; 
    };

    imu->data.ax = flip_endianess(temp.ax);
    imu->data.ay = flip_endianess(temp.ay);
    imu->data.az = flip_endianess(temp.az);
    imu->data.temp = flip_endianess(temp.temp);
    imu->data.gx = flip_endianess(temp.gx);
    imu->data.gy = flip_endianess(temp.gy);
    imu->data.gz = flip_endianess(temp.gz);
    return HAL_OK;
}

void mpu6050_calc_units(mpu6050_t *imu){

    float dt = elapsed_time_s(imu->timer); 
    // float dt = mpu6050_elapsed_time(imu); 

    //Calculate Angular Velocities [Deg/s]
    float wx = -(imu->data.gx - imu->gyro_offset[0]) / MPU6050_GYRO_SCALE; 
    float wy = -(imu->data.gy- imu->gyro_offset[1]) / MPU6050_GYRO_SCALE; 
    float wz = -(imu->data.gz - imu->gyro_offset[2]) / MPU6050_GYRO_SCALE; 

    //Store
    imu->wx = wx;
    imu->wy = wy;
    imu->wz = wz;

    //Euler Angles [Deg]
    imu->roll  += wx * dt;
    imu->pitch += wy * dt;
    imu->yaw   += wz * dt;

    //Acceleration [g]
    imu->ax = (imu->data.ax) / MPU6050_ACCEL_SCALE; 
    imu->ay = (imu->data.ay) / MPU6050_ACCEL_SCALE; 
    imu->az = (imu->data.az) / MPU6050_ACCEL_SCALE; 
}

void mpu6050_comp_filter(mpu6050_t *imu){

    //Filter Acceleration Data: 
    float ax_f = moving_avg(imu->ax_filtered, imu->ax);  
    float ay_f = moving_avg(imu->ay_filtered, imu->ay);  
    float az_f = moving_avg(imu->az_filtered, imu->az);  



    //Compute Accel Angles [gyro - Quick changes | accel - Long Term Stability]
    // imu->accel_roll  = atan2f(imu->az, imu->ay) * RAD_TO_DEG_SCALE; 
    imu->accel_pitch = atan2f(az_f, (sqrtf(ay_f*ay_f + az_f*az_f) ) ) * RAD_TO_DEG_SCALE;

    // //Fuse Data 
    // imu->roll  = ((1 - imu->alpha_fusion) * (imu->roll))  + ((imu->alpha_fusion * accel_roll)); 
    // imu->pitch = ((1 - imu->alpha_fusion) * (imu->pitch)) + ((imu->alpha_fusion * accel_pitch)); 
}; 

void mpu6050_task(mpu6050_t *imu){
    //Make sure data is available
    if (imu->data_ready_flag != true)
        return; 
    imu->data_ready_flag = false; 

    //Read data & elapsed time
    mpu6050_read_data(imu); 
    elapsed_time_s(imu->timer);
    // mpu6050_elapsed_time(imu); 

    switch(imu->status){

        case(STATE_IMU_INIT):
            mpu6050_init(imu); 
            imu->status = STATE_RESET_OFFSETS;  
            break; 

        case(STATE_RESET_OFFSETS):
            imu->calibration_start_time = TIM2->CNT; 

            imu->sample_idx = 0; 
            imu->gyro_cal_sum[0] = 0;
            imu->gyro_cal_sum[1] = 0;
            imu->gyro_cal_sum[2] = 0;

            imu->accel_cal_sum[0] = 0;
            imu->accel_cal_sum[1] = 0;
            imu->accel_cal_sum[2] = 0;

            imu->gyro_offset[0] = 0;
            imu->gyro_offset[1] = 0;
            imu->gyro_offset[2] = 0;

            imu->accel_offset[0] = 0;
            imu->accel_offset[1] = 0;
            imu->accel_offset[2] = 0;

            imu->status = STATE_TIME_START; 
            break; 
        
        case(STATE_TIME_START):
            imu->calibration_start_time = TIM2->CNT; 
            imu->status = STATE_TIME_WAIT; 
            break; 

        case(STATE_TIME_WAIT): 
            float time_elapsed = (TIM2->CNT - imu->calibration_start_time) * TIMER_SCALE_SECONDS; 
            if (time_elapsed > MPU6050_SAMPLE_TIME){
                imu->status = STATE_SAMPLE_OFFSET; 
                return; 
            }
            break; 
            
        case(STATE_SAMPLE_OFFSET):
     
            //Average Offsets once filled
            if (imu->sample_idx >= imu->sample_count){

                imu->gyro_offset[0] = imu->gyro_cal_sum[0] / imu->sample_count; 
                imu->gyro_offset[1] = imu->gyro_cal_sum[1] / imu->sample_count; 
                imu->gyro_offset[2] = imu->gyro_cal_sum[2] / imu->sample_count; 

                imu->accel_offset[0] = imu->accel_cal_sum[0] / imu->sample_count; 
                imu->accel_offset[1] = imu->accel_cal_sum[1] / imu->sample_count; 
                imu->accel_offset[2] = imu->accel_cal_sum[2] / imu->sample_count; 
                
                //Beging Full Sampling once Offsets collected
                imu->status = STATE_SAMPLE_DATA; 
                return;
            }
            else{
            //Sum offsets
                imu->gyro_cal_sum[0] += imu->data.gx; 
                imu->gyro_cal_sum[1] += imu->data.gy; 
                imu->gyro_cal_sum[2] += imu->data.gz; 

                imu->accel_cal_sum[0] += imu->data.ax; 
                imu->accel_cal_sum[1] += imu->data.ay; 
                imu->accel_cal_sum[2] += imu->data.az; 

                imu->status = STATE_TIME_START; 
                imu->calibration_start_time = TIM2->CNT;
                imu->sample_idx++; 
            }
            break; 

        case(STATE_SAMPLE_DATA):
    
            mpu6050_calc_units(imu);    
            mpu6050_comp_filter(imu);    
            break;             
            
        default:
            break; 
    }
} 
