#include "pca9685.h"
#include "motor.h"



HAL_StatusTypeDef motor_set_angle(motor_t *motor, float angleDeg) {
    // Clip the angle between 0 and 180
    if (angleDeg < 0) {
        angleDeg = 0;
    } else if (angleDeg > 180) {
        angleDeg = 180;
    }
    //Off Time Calculation
    uint16_t offTime = (((2.0 / 180.0 * angleDeg) + 0.5) * (4096.0 / 20));
    HAL_StatusTypeDef status = setPWM(motor->motorPin, 0, offTime);
    return status; 
}


