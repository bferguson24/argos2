#pragma once
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <stdbool.h>




typedef struct{
const int motorPin;
int motorAngle;
}motor_t;

HAL_StatusTypeDef motor_set_angle(motor_t *motor, float angleDeg); 
