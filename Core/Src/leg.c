#include "leg.h"
#include <stdlib.h>
#include "utility.h"

//Leg Objects





void IK(leg_t *leg, float x, float y, float z){
//Might want to put parameters here to calculate leg specific angles using l/w/h map;

//Calculate angles
    float theta1_0 = 90 - (atan2((sqrt(z*z + y*y - d1*d1)),d1) - (M_PI/2) + atan2(z,y)) * (180.0/M_PI);
    float theta2_0 = (M_PI - atan2(sqrt(z*z + y*y -d1*d1),x) - acos( (l1*l1 - l2*l2 + (x*x + z*z + y*y -d1*d1))/(-2*l1*sqrt(x*x + z*z + y*y - d1*d1)))) * (180.0/M_PI);
    float theta3_0 = (M_PI - acos((x*x + z*z + y*y -d1*d1 - l1*l1 - l2*l2) / (-2*l1*l2))) * (180.0/M_PI);

//Update angles based on leg specific motor orientation
    // theta1 = leg->reflectionMap[3] * fabs((leg->reflectionMap[0] * 180.0f - theta1));
    // theta2 = fabs((leg->reflectionMap[1] * 180.0f - theta2_0 * leg->reflectionMap[4]));
    // theta3 = fabs((leg->reflectionMap[2] * 180.0f - theta3 - theta2_0*leg->reflectionMap[5])); 

    float theta1 = fabs((leg->reflectionMap[0] * 180.0f - theta1_0));
    float theta2 = fabs((leg->reflectionMap[1] * 180.0f) - theta2_0 * leg->reflectionMap[4]);
    float theta3 = fabs(leg->reflectionMap[5] * ((leg->reflectionMap[2] * 180) - theta3_0 * leg->reflectionMap[5] + theta2_0 * leg->reflectionMap[4]));
//Store angles in leg motor object
    leg->motorList[0]->motorAngle = theta1;
    leg->motorList[1]->motorAngle = theta2;
    leg->motorList[2]->motorAngle = theta3;
} 


void legPositionIncrement(leg_t *leg, float dx, float dy, float dz, float maxDist){

    leg->x = clip(leg->x + dx, homeX - maxDist, homeX + maxDist);
    leg->y = clip(leg->y + dy, homeY - maxDist, homeY + maxDist);
    leg->z = clip(leg->z + dz, homeZ - maxDist, homeZ + maxDist);


    IK(leg,leg->x,leg->y,leg->z);
}

void moveLeg(leg_t *leg){
    for(int i = 0; i < 3; i++){
        motor_set_angle(leg->motorList[i], leg->motorList[i]->motorAngle);
    }
}

void home(leg_t *leg){
    leg->x = homeX;
    leg->y = homeY;
    leg->z = homeZ;
    IK(leg,leg->x,leg->y,leg->z);

    moveLeg(leg);
}


