#include "can.h"
#include "stm32f4xx_hal.h"
extern "C" {
#include "UART_DMA.h"
#include "vofa.h"
}

#include "drv_can.h"
#include "drv_tim.h"
#include "dvc_motor_dm.h"
#include "user_task.h"
#include "Arm_2Freedom.h"
#include <math.h>

enum Enum_Arm_Step
{
    Arm_Step_IDLE = 0,
    Arm_Step_DESCEND,
    Arm_Step_CLAMP,
    Arm_Step_LIFT,
    Arm_Step_ROTATE,
    Arm_Step_LOWER,
    Arm_Step_RELEASE,
    Arm_Step_DONE,
};

Enum_Arm_Step Arm_Step = Arm_Step_IDLE;
uint32_t Arm_Step_Tick = 0;
const uint32_t Gripper_Wait_Ms = 1000;

bool Arm_Motor_Arrived(Class_Motor_DM_Normal *Motor)
{
    float Angle_Error = fabsf(Motor->Get_Now_Angle()
                            - Motor->Get_Control_Angle());

    return (Angle_Error <= 5.0f / 180.0f * PI);
}



void Arm_Set_Step(Enum_Arm_Step Step)
{
    Arm_Step = Step;
    Arm_Step_Tick = HAL_GetTick();

    switch(Arm_Step)
    {
        case (Arm_Step_IDLE):
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
            break;
        }

        case (Arm_Step_DESCEND):
        {
            Arm_Lift_Down();
            break;
        }

        case (Arm_Step_CLAMP):
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
            break;
        }

        case (Arm_Step_LIFT):
        {
            Arm_Lift_Up();
            break;
        }

        case (Arm_Step_ROTATE):
        {
            Arm_Rotate_To_End();
            break;
        }

        case (Arm_Step_LOWER):
        {
            Arm_Lift_Down();
            break;
        }

        case (Arm_Step_RELEASE):
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
            break;
        }

        case (Arm_Step_DONE):
        default:
        {
            break;
        }
    }
}

void Arm_Step_Update()
{
    switch (Arm_Step)
    {
        case (Arm_Step_IDLE):
        {
            if (Arm_Motor_Arrived(&DM4340_Lift) &&
                Arm_Motor_Arrived(&DM4340_Rotate))
            {
                Arm_Set_Step(Arm_Step_DESCEND);
            }
            break;
        }

        case (Arm_Step_DESCEND):
        {
            if (Arm_Motor_Arrived(&DM4340_Lift))
            {
                Arm_Set_Step(Arm_Step_CLAMP);
            }
            break;
        }

        case (Arm_Step_CLAMP):
        {
            if (HAL_GetTick() - Arm_Step_Tick >= Gripper_Wait_Ms)
            {
                Arm_Set_Step(Arm_Step_LIFT);
            }
            break;
        }

        case (Arm_Step_LIFT):
        {
            if (Arm_Motor_Arrived(&DM4340_Lift))
            {
                Arm_Set_Step(Arm_Step_ROTATE);
            }
            break;
        }

        case (Arm_Step_ROTATE):
        {
            if (Arm_Motor_Arrived(&DM4340_Rotate))
            {
                Arm_Set_Step(Arm_Step_LOWER);
            }
            break;
        }

        case (Arm_Step_LOWER):
        {
            if (Arm_Motor_Arrived(&DM4340_Lift))
            {
                Arm_Set_Step(Arm_Step_RELEASE);
            }
            break;
        }

        case (Arm_Step_RELEASE):
        {
            if (HAL_GetTick() - Arm_Step_Tick >= Gripper_Wait_Ms)
            {
                Arm_Set_Step(Arm_Step_DONE);
            }
            break;
        }

        default:
        {
            break;
        }
    }
}

void Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
        case(0x03):
        {
            DM4340_Lift.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }

        case(0x02):
        {
            DM4340_Rotate.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
    }
}

void TASK_1MS_TIM5_Callback()
{
    if(init_finished)
    {
        static int count_1ms = 0;
        count_1ms++;
        if(count_1ms == 1)
        {
            Arm_Step_Update();
            DM4340_Lift.TIM_Send_PeriodElapsedCallback();
            DM4340_Rotate.TIM_Send_PeriodElapsedCallback();

            count_1ms = 0;
        }

        static int count_10ms = 0;
        count_10ms++;
        if(count_10ms == 10)
        {
            Vofa_Transmit(&huart1,8);

            count_10ms = 0;
        }

        static int count_100ms = 0;
        count_100ms++;
        if(count_100ms == 100)
        {
            count_100ms = 0;
        }
    }   
}

void User_Task_Init()
{
    HAL_Delay(100);
    CAN_Init(&hcan1,Device_CAN1_Callback);
    TIM_Init(&htim5, TASK_1MS_TIM5_Callback);
    UART_DMA_Receive_init(&huart1, buffer_receive_1, buffer_receive_length_1);
    HAL_TIM_Base_Start_IT(&htim5);

    DM4340_Lift.Init(&hcan1, 0x03,0x13,Motor_DM_Control_Method_NORMAL_MIT, 25.0f, 45.0f, 28.0f);
    DM4340_Rotate.Init(&hcan1, 0x02,0x12,Motor_DM_Control_Method_NORMAL_MIT,25.0f, 45.0f, 28.0f);

    DM4340_Lift.Set_K_P(120.0f);
    DM4340_Lift.Set_K_D(4.0f);
    DM4340_Lift.Set_Control_Omega(0.0f);
    DM4340_Lift.Set_Control_Torque(0.0f);

    DM4340_Rotate.Set_K_P(120.0f);
    DM4340_Rotate.Set_K_D(4.0f);
    DM4340_Rotate.Set_Control_Omega(0.0f);
    DM4340_Rotate.Set_Control_Torque(0.0f);

    Arm_Set_Step(Arm_Step_IDLE); 
    Arm_Start();
    HAL_Delay(1000);
    Arm_Rotate_To_Start();
    HAL_Delay(2000);

    init_finished = true;
    while(1)
    {

    }
    
}   
