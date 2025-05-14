#include "leg.h"
#include "argos_parameters.h"


typedef struct {
leg_t *legList[4];

}quadruped_t; 

//function prototypes

void quadHome(quadruped_t *quad);
void quadPositionIncrement(quadruped_t*quad, float dx, float dy, float  dz);
void quadWalk(quadruped_t *quad, float speed);
void quadRPYcontrol(quadruped_t *quad, float roll, float pitch, float yaw);
void quadMove(quadruped_t *quad);
