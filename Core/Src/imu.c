#include "lsm6dso.h"
#include "pio.h"
#include "proto/nextbot/node.pb.h"
#include "quaternion_math_functions.h"
#include "samg55j19.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "arm_common_tables.h"
#include "arm_math.h"
#include "arm_math_types.h"
#include "dsp/fast_math_functions.h"
#include "dsp/support_functions.h"
#include "dsp/utils.h"
#include "utility.h"
#include "pio_handler.h"

#include "sensor_fusion.h"

#include "imu.h"

extern NodeState state;

// #define MAX_STEPS     (1 << 16)
// #define G_RANGE       4.0
// #define DPS           500.0

const float MAX_STEPS =   65536.0f; // 16-bit
const float G_RANGE =     2.0f * 2.0f;      // +/- 4g
const float DPS =         250.0f * 2.0f;    // +/- 500 dps
float ACCEL_RES =   G_RANGE / MAX_STEPS;
float GYRO_RES =    DPS / MAX_STEPS;

float DT = 1.0f / 833.0f;
// float DT = 1.0f / 416.0f;

// float FUSION_COEFFICIENT = 85.0f;
float FUSION_COEFFICIENT = 0.98f;
float fused_vector[] = {0.0f, 0.0f, -1.0f};

FusionVector gyro = {0, 0, 0};
FusionVector accel = {0, 0, 0};

volatile bool readyToProcess = false;

void IMU_fusion_callback() {
    readyToProcess = true;
}

bool IMU_fusion_task() {
    if(!readyToProcess)
        return false;

    // Get latest IMU data
    imu_data_t* imu_data = LSM6DSO_getLastData();

    // Normalize IMU readings
    accel.axis.x = imu_data->accel_x * ACCEL_RES;
    accel.axis.y = imu_data->accel_y * ACCEL_RES;
    accel.axis.z = imu_data->accel_z * ACCEL_RES * -1.0f;

    gyro.axis.x = imu_data->gyro_x * GYRO_RES * PI / 180.0f;
    gyro.axis.y = imu_data->gyro_y * GYRO_RES * PI / 180.0f;
    gyro.axis.z = imu_data->gyro_z * GYRO_RES * PI / 180.0f * -1.0f;

    readyToProcess = 0;
    // return true;

    // Normalize IMU readings
    state.imu.accel.x = imu_data->accel_x * ACCEL_RES;
    state.imu.accel.y = imu_data->accel_y * ACCEL_RES;
    state.imu.accel.z = imu_data->accel_z * ACCEL_RES * -1.0f;

    state.imu.gyro.x = imu_data->gyro_x * GYRO_RES * PI / 180.0f;
    state.imu.gyro.y = imu_data->gyro_y * GYRO_RES * PI / 180.0f;
    state.imu.gyro.z = imu_data->gyro_z * GYRO_RES * PI / 180.0f * -1.0f;


    // This is necessary because for some reason the second time we get here, fused_vector is NaN
    // This should be fixed in the future
    if( fused_vector[0] != fused_vector[0] || 
        fused_vector[1] != fused_vector[1] || 
        fused_vector[2] != fused_vector[2])
    {
        fused_vector[0] = 0;
        fused_vector[1] = 0;
        fused_vector[2] = -1;
    }
    
    // Perform complementary filtering
    FUSION_update_fused_vector(fused_vector, &state.imu.accel.x, &state.imu.gyro.x, DT, FUSION_COEFFICIENT);
    
    // Update state with current fused vector
    memcpy(&state.imu.orientation.x, fused_vector, sizeof(fused_vector));
}

void IMU_init(void) {
    // Initialize IMU sensor
    LSM6DSO_init();

    // Configure GPIO Pin PA9 as interrupt pin for IMU sensor
    pio_set_input(PIOA, PIO_PA9, 0);

    // Configure interrupt handler for IMU sensor
    pio_handler_set(PIOA, ID_PIOA, PIO_PA9, PIO_IT_RISE_EDGE, LSM6DS0_interrupt_callback);
    
    // Enable interrupt for IMU sensor
    pio_enable_interrupt(PIOA, PIO_PA9);
   	NVIC_SetPriority(PIOA_IRQn, 5);
}