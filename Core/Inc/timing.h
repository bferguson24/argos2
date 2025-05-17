#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef struct {
    uint32_t prev_cnt;
    uint32_t curr_cnt; 
    float elapsed_time_s;
}timing_t; 

#define TIMER_SCALE_SECONDS 1e-6

float elapsed_time_s(timing_t *timer); 
