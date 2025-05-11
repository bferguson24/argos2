#include "utility.h"
#include "stdint.h"

float clip(float val, float min, float max){
    if (val < min) return min;
    if (val > max) return max; 
    return val;
}


int map(int val, int val_min, int val_max, int out_min, int out_max){
    int output = (val - val_min) * (out_max - out_min) / (val_max - val_min) + out_min; 
    return output;
}

float mapf(float val, float val_min, float val_max, float out_min, float out_max) {
    return (val - val_min) * (out_max - out_min) / (val_max - val_min) + out_min;
}

int16_t flip_endianess(int16_t x){
    return (x >>8) | (x <<8); 
}; 