#include "drv_math.h"
#include "dvc_motor_dm.h"
#include "Arm_2Freedom.h"

Class_Motor_DM_Normal DM4340_Lift;
Class_Motor_DM_Normal DM4340_Rotate;

static const float Start_Angle = -45.0f / 180.0f * PI;

static const float Lift_Down_Angle = 0.0f;
static const float Lift_Up_Angle = -90.0f / 180.0f * PI;

static const float Rotate_Start_Angle = 0.0f;
static const float Rotate_End_Angle = 90.0f / 180.0f * PI;

void Arm_Start()
{
    DM4340_Lift.Set_Control_Angle(Start_Angle);
}
void Arm_Lift_Up()
{
    DM4340_Lift.Set_Control_Angle(Lift_Up_Angle);
}

void Arm_Lift_Down()
{
    DM4340_Lift.Set_Control_Angle(Lift_Down_Angle);
}

void Arm_Rotate_To_End()
{
    DM4340_Rotate.Set_Control_Angle(Rotate_End_Angle);
}

void Arm_Rotate_To_Start()
{
    DM4340_Rotate.Set_Control_Angle(Rotate_Start_Angle);
}

