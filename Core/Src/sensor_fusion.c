#include "sensor_fusion.h"
#include "arm_common_tables.h"
#include "fast_math_functions.h"

void vector_3d_normalize(float* v, float* result_3v)
{
    float norm;

    arm_sqrt_f32(v[0]*v[0] + v[1]*v[1] + v[2]*v[2], &norm);
    norm = 1/norm;

    result_3v[0] = v[0] * norm;
    result_3v[1] = v[1] * norm;
    result_3v[2] = v[2] * norm;
}

void quaternion_rotate_vector(float* v, float* q, float* result_q)
{
    float q_mult[4];

    float vector_q[] = {0.0, v[0], v[1], v[2]};
    float q_inverse[] = {q[0], -q[1], -q[2], -q[3]};

    arm_quaternion_product_single_f32(q, vector_q, (float*) &q_mult);
    arm_quaternion_product_single_f32(q_mult, q_inverse, result_q);
}

void quaternion_from_gyro(float* gyro, float dt, float* result_q)
{
    float alpha = 0.5f * dt;
    float w,x,y,z;
    float dx,dy,dz;
    
    // x = alpha * gyro[0];
    // y = alpha * gyro[1];
    // z = alpha * gyro[2];
    // w = 1.0f - 0.5f * (x * x + y * y + z * z);

    // x = dt * gyro[0];
    // y = dt * gyro[1];
    // z = dt * gyro[2];
    // arm_sqrt_f32(1.0f - (x * x + y * y + z * z), &w);

    dx = dt * gyro[0];
    dy = dt * gyro[1];
    dz = dt * gyro[2];
    float angle = 0;
    arm_sqrt_f32(dx * dx + dy * dy + dz * dz, &angle);
    w = arm_cos_f32(angle/2);
    x = arm_sin_f32(angle/2)*dx/angle;
    y = arm_sin_f32(angle/2)*dy/angle;
    z = arm_sin_f32(angle/2)*dz/angle;

    // result_q[0] = w;
    // result_q[1] = -x;
    // result_q[2] = -y;
    // result_q[3] = -z;
    result_q[0] = w;
    result_q[1] = x;
    result_q[2] = y;
    result_q[3] = z;
}

void FUSION_update_fused_vector(float* fused_vector, float* accel, float* gyro, float delta, float fusion_coefficient)
{
    // Turn gyro vector into a quaternion
    float gyro_q[4];
    quaternion_from_gyro(gyro, delta, gyro_q);

    // Update gyro gravity vector
    float gyro_gravity[4];
    float gyro_gravity_norm[3];
    quaternion_rotate_vector(fused_vector, gyro_q, gyro_gravity);
    // memcpy(gyro_gravity_norm, &gyro_gravity[1], 12);
    vector_3d_normalize(&gyro_gravity[1], gyro_gravity_norm);

    // Normalize acceleration
    float accel_norm[3];
    memcpy(accel_norm, accel, 12);
    // vector_3d_normalize(accel, accel_norm);

    // Perform sensor fusion
    float vector_sum[3];
    // vector_sum[0] = gyro_gravity[1] * fusion_coefficient + accel_norm[0];
    // vector_sum[1] = gyro_gravity[2] * fusion_coefficient + accel_norm[1];
    // vector_sum[2] = gyro_gravity[3] * fusion_coefficient + accel_norm[2];
    vector_sum[0] = gyro_gravity_norm[0] * fusion_coefficient + accel_norm[0] * (1.0f - fusion_coefficient);
    vector_sum[1] = gyro_gravity_norm[1] * fusion_coefficient + accel_norm[1] * (1.0f - fusion_coefficient);
    vector_sum[2] = gyro_gravity_norm[2] * fusion_coefficient + accel_norm[2] * (1.0f - fusion_coefficient);

    // Normalize our vector sum to get our filtered gravity estimate
    vector_3d_normalize(vector_sum, fused_vector);
    // memcpy(fused_vector, vector_sum, 12);
}