#include "filter.h"


float moving_avg(moving_avg_t *sample, float newVal){
    float avg = (sample->alpha) * (newVal) + (1 - sample->alpha) * (sample->average); 
    sample->average = avg; 
    return avg; 
}