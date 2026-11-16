#ifndef ARM_2FREEDOM_H
#define ARM_2FREEDOM_H

#ifdef __cplusplus
#include "dvc_motor_dm.h"

extern Class_Motor_DM_Normal DM4340_Lift;
extern Class_Motor_DM_Normal DM4340_Rotate;

extern "C" {
#endif

void Arm_Start();
void Arm_Lift_Up();
void Arm_Lift_Down();
void Arm_Rotate_To_End();
void Arm_Rotate_To_Start();

#ifdef __cplusplus
}
#endif

#endif
