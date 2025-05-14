#pragma once


#include "math.h"
#include "motor.h"
#include "argos_parameters.h"


typedef struct{
//Motor Objects
motor_t *motorList[3];


//Leg Mirror/Properties
const int reflectionMap[6];

//Kinematic Distances
const float d_torso_hip[3];
const float d_torso_foot[3]; 


//Current Leg Positions
float x;
float y; 
float z;

//Home Position
float home_position[3];

}leg_t; 


//Leg Function Declarations

void legPositionIncrement(leg_t *leg, float dx, float dy, float dz, float maxDist);

//Inverse Kinematics Calculation:
    //Updates internal leg angles given x/y/z
void IK(leg_t *leg, float x, float y, float z); 

void moveLeg(leg_t *leg);

//Formard Kinematics Map:
    //Don't really think we will need since everything is driven by the input x/y/z anyway?
void FK(leg_t *leg, float theta1, float theta2, float theta3);

//Home Leg Position to default
void home(leg_t *leg);


void walkLoop(leg_t *leg, int state);

