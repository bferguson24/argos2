#pragma once
#include "stdint.h"

float clip(float val, float min, float max);
int map(int val, int val_min, int val_max, int out_min, int out_max); 
float mapf(float val, float val_min, float val_max, float out_min, float out_max);
int16_t flip_endianess(uint16_t x); 
