#pragma once 

typedef struct { 
float alpha; 
float average; 
}moving_avg_t;


float moving_avg(moving_avg_t *sample, float newVal); 
