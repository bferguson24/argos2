#include "quadruped.h"
#include "argos_parameters.h"
#include "leg.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"  





void quad_home(quadruped_t *quad){
    for (int i = 0; i < 4; i++){
        home(quad->legList[i]); 
    }
}

void quad_calc_walk_joints(quadruped_t *quad){
    
    for (int i = 0; i < PositionCount ; i++){
    
        //Current leg index:
        int leg_i;


        //Forward Swing:
        leg_i = 0;
            float x_hip_foot_1 = (stepListX[i] - quad->legList[leg_i]->reflectionMap[3] * quad->legList[leg_i]->d_torso_hip[0]);
            float y_hip_foot_1 = stepListY[i] - quad->legList[leg_i]->d_torso_hip[1];
            float z_hip_foot_1 =  (stepListZ[i] - quad->legList[leg_i]->reflectionMap[5] * quad->legList[leg_i]->d_torso_hip[2]);
            IK(quad->legList[leg_i], x_hip_foot_1, y_hip_foot_1, z_hip_foot_1);

        leg_i = 3;
            float x_hip_foot_2 = (stepListX[i] - quad->legList[leg_i]->reflectionMap[3] * quad->legList[leg_i]->d_torso_hip[0]);
            float y_hip_foot_2 = stepListY[i] - quad->legList[leg_i]->d_torso_hip[1];
            float z_hip_foot_2 =  (stepListZ[i] - quad->legList[leg_i]->reflectionMap[5] * quad->legList[leg_i]->d_torso_hip[2]);
            IK(quad->legList[leg_i], x_hip_foot_2, y_hip_foot_2, z_hip_foot_2);

            
        //Return Swing
        leg_i = 1;
            float x_hip_foot_3 = (stepListX[(i + PositionCount/2)%PositionCount]- quad->legList[leg_i]->reflectionMap[3] * quad->legList[leg_i]->d_torso_hip[0]);
            float y_hip_foot_3 = stepListY[(i + PositionCount/2)%PositionCount] - quad->legList[leg_i]->d_torso_hip[1];
            float z_hip_foot_3 =  (stepListZ[(i + PositionCount/2)%PositionCount] - quad->legList[leg_i]->reflectionMap[5] * quad->legList[leg_i]->d_torso_hip[2]);
            IK(quad->legList[leg_i], x_hip_foot_3, y_hip_foot_3, z_hip_foot_3);

        leg_i = 2;
            float x_hip_foot_4 = (stepListX[(i + PositionCount/2)%PositionCount] - quad->legList[leg_i]->reflectionMap[3] * quad->legList[leg_i]->d_torso_hip[0]);
            float y_hip_foot_4 = stepListY[(i + PositionCount/2)%PositionCount] - quad->legList[leg_i]->d_torso_hip[1];
            float z_hip_foot_4 =  (stepListZ[(i + PositionCount/2)%PositionCount] - quad->legList[leg_i]->reflectionMap[5] * quad->legList[leg_i]->d_torso_hip[2]);
            IK(quad->legList[leg_i], x_hip_foot_4, y_hip_foot_4, z_hip_foot_4);

            
        //Publish Movements
        // quadMove(quad);

        //Temporary Speed Parameter (Future Work will improve this)
        // HAL_Delay(100 * (1-quad->walking_speed));
    }
}

void quad_walk_task(quadruped_t *quad){
    switch (quad->walk_state){
        case(WALK_IDLE):
            break;
        
        case (WALK_CALC_JOINTS): 
            quad_calc_walk_joints(quad); 
            quad->walk_state = WALK_STEP; 
            break;
        
        case (WALK_STEP):
            //Publishes Joint Commands to motors;
            quad_move(quad); 
            quad->walk_state = WALK_TIME_DELAY; 
            quad->curr_time = TIM2->CNT;
            break; 

        case (WALK_TIME_DELAY):
            if ((TIM2->CNT - quad->curr_time) >= quad->walk_step_delay){
                quad->walk_state = WALK_CALC_JOINTS; 
                return; 
            }
            break; 
    }
}


void quadPositionIncrement(quadruped_t*quad, float dx, float dy, float  dz){
    for (int i = 0; i<4; i++){
        legPositionIncrement(quad->legList[i], dx, dy,dz, 1000);
        moveLeg(quad->legList[i]);
    }
}

void quadRPYcontrol(quadruped_t *quad, float roll, float pitch, float yaw){
//Roll Pitch Yaw inputs in Degrees; CW+ CCW-
float y = roll * M_PI / 180.0; 
float b = pitch * M_PI / 180.0;
float a = yaw * M_PI / 180.0;

for (int i = 0; i< 4; i++){
    //Calculate Torso -> Hip Position
        float x_hip_rotated = quad->legList[i]->d_torso_hip[0] * (cos(a) * cos(b)) - quad->legList[i]->d_torso_hip[1] * (sin(b)) + quad->legList[i]->d_torso_hip[2] * (cos(b) * sin(a));
        float y_hip_rotated = quad->legList[i]->d_torso_hip[0] * (sin(a)*sin(y) + cos(a)*sin(b)*cos(y)) - quad->legList[i]->d_torso_hip[2] * (cos(a)*sin(y) - sin(a)*sin(b)*cos(y)) + quad->legList[i]->d_torso_hip[1] * (cos(b) * cos(y));
        float z_hip_rotated = quad->legList[i]->d_torso_hip[2] * (cos(a)*cos(y) + sin(a) * sin(b) * sin(y)) - quad->legList[i]->d_torso_hip[0] * (sin(a) * cos(y) - cos(a) * sin(b) * sin(y)) + quad->legList[i]->d_torso_hip[1] * (cos(b) * sin(y));

    //Calculate FINAL Hip - > Foot Position
        float x_hip_foot = quad->legList[i]->reflectionMap[3] * (quad->legList[i]->d_torso_foot[0] - x_hip_rotated);
        float y_hip_foot = quad->legList[i]->d_torso_foot[1] - y_hip_rotated;
        float z_hip_foot = quad->legList[i]->reflectionMap[5] * (quad->legList[i]->d_torso_foot[2] - z_hip_rotated);

    //Set Position
        quad->legList[i]->x =  x_hip_foot;
        quad->legList[i]->y =  y_hip_foot;
        quad->legList[i]->z =  z_hip_foot;

    //Calculate Angles
        IK(quad->legList[i], x_hip_foot, y_hip_foot, z_hip_foot);
    }
}

void quad_move(quadruped_t *quad){
    for (int i = 0; i< 4; i++){
        moveLeg(quad->legList[i]);
    }
}

void quad_task(quadruped_t *quad){ 

    switch(quad->state){
        case (STATE_IDLE):
            break; 
        case (STATE_HOME):
            quad_home(quad);
            quad->state = STATE_IDLE; 
            break; 

        case (STATE_WALK_PATH):
            quad_walk_task(quad); 
            break; 
             
        case (STATE_RPY_CONTROL):
            break; 
    };
}