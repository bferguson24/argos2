#pragma once
#include "leg.h"
#include "argos_parameters.h"


typedef enum {
    STATE_IDLE, 
    STATE_HOME,
    STATE_WALK_PATH, 
    STATE_RPY_CONTROL
}quad_state_t; 

typedef enum {
    WALK_IDLE,
    WALK_CALC_JOINTS,
    WALK_STEP,
    WALK_TIME_DELAY,
}walk_state_t;

typedef struct {
    leg_t *legList[4];
    quad_state_t state; 
    walk_state_t walk_state; 
    
    //Timing
    uint32_t prev_time;
    uint32_t curr_time; 

    float walk_step_delay; 
    float walking_speed; 
}quadruped_t; 

//function prototypes

void quad_home(quadruped_t *quad);
void quadPositionIncrement(quadruped_t*quad, float dx, float dy, float  dz);
void quad_calc_walk_joints(quadruped_t *quad); 
void quadRPYcontrol(quadruped_t *quad, float roll, float pitch, float yaw); 
void quad_move(quadruped_t *quad);
void quad_task(quadruped_t *quad);
void quad_walk_task(quadruped_t *quad); 
