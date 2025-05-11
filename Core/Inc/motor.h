#pragma once
#include <stdint.h>
#include "stm32f4xx_hal_def.h"



typedef struct{
const int motorPin;
int motorAngle;
}motor_t;


//Motor Functions

HAL_StatusTypeDef motor_set_angle(motor_t *motor, float angleDeg); 
