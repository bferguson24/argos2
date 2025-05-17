#include "timing.h"

float elapsed_time_s(timing_t *timer){
    //Return the time in [s] between reads; 
    uint32_t delta_count = timer->curr_cnt - timer->prev_cnt; 

    float time_s = delta_count * TIMER_SCALE_SECONDS; 
    timer->elapsed_time_s = time_s; 
    return time_s; 
}