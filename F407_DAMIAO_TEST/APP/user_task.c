#include "can.h"
#include "stm32f4xx_hal.h"
extern "C" {
#include "UART_DMA.h"
#include "vofa_setting.h"
#include "vofa.h"
#include "stdio.h"
#include "stdint.h"
}

#include "drv_can.h"
#include "drv_tim.h"
#include "drv_math.h"
#include "dvc_motor_dm.h"
#include "user_task.h"

static Class_Motor_DM_Normal DM4340;

/**
 * @brief CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
void Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
        case (0x00):
        {
          DM4340.CAN_RxCpltCallback(CAN_RxMessage->Data);
          break;
        }
    }
}

void TASK_1MS_TIM5_Callback()
{
    static int count_1ms = 0;
    count_1ms++;
    if(count_1ms == 1)
    {
        DM4340.TIM_Send_PeriodElapsedCallback();
        count_1ms = 0;
    }

    static int count_10ms = 0;
    count_10ms++;
    if(count_10ms == 10)
    {
        DM4340.Set_Control_Omega(VofaData[0]);

        tempFloat[0] = DM4340.Get_Control_Omega();
        tempFloat[1] = DM4340.Get_Now_Omega();
        tempFloat[2] = DM4340.Get_Control_Angle();
        tempFloat[3] = DM4340.Get_Now_Angle();

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

void User_Task_Init()
{
    HAL_Delay(100);
    CAN_Init(&hcan1,Device_CAN1_Callback);
    TIM_Init(&htim5, TASK_1MS_TIM5_Callback);
    UART_DMA_Receive_init(&huart1, buffer_receive_1, buffer_receive_length_1);

    DM4340.Init(&hcan1, 0x00, 0x01,Motor_DM_Control_Method_NORMAL_MIT,25.0f, 45.0f, 28.0f);

    DM4340.Set_Control_Omega(0.5*PI);
    DM4340.Set_K_P(0.0f);
    DM4340.Set_K_D(0.7f);   

    init_finished = true;
    HAL_TIM_Base_Start_IT(&htim5);
    while(1)
    {
        DM4340.Set_K_D(Vofa_Slider3);
        DM4340.Set_K_P(Vofa_Slider2);
    }
    
}
