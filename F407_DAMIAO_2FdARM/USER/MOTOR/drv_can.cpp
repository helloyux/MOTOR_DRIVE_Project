/**
 * @file drv_can.cpp
 * @brief CAN通信初始化与配置流程
 * @brief 官方6020驱动更新, 适配电压控制与电流控制
 * @brief 适配达妙电机
 * @brief 新增回调函数空指针判定
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_can.h"

/* Private macros ------------------------------------------------------------*/

// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

// 标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
Struct_CAN_Manage_Object CAN2_Manage_Object = {0};

// 本工程只开了 CAN1 (Core/Src/can.c 里只有 hcan1), 但模板代码 drv_can.cpp /
// dvc_motor_dm.cpp 里仍然引用了 hcan2, 不定义它链接会报 undefined reference。
// 这里给一个弱定义: 以后用 CubeMX 启用 CAN2 时, can.c 里的强定义会自动取代它。
__attribute__((weak)) CAN_HandleTypeDef hcan2;

// CAN通信发送缓冲区

// 电机共享区
uint8_t CAN1_0x1fe_Tx_Data[8];
uint8_t CAN1_0x1ff_Tx_Data[8];
uint8_t CAN1_0x200_Tx_Data[8];
uint8_t CAN1_0x2fe_Tx_Data[8];
uint8_t CAN1_0x2ff_Tx_Data[8];
uint8_t CAN1_0x3fe_Tx_Data[8];
uint8_t CAN1_0x4fe_Tx_Data[8];

// 电机共享区
uint8_t CAN2_0x1fe_Tx_Data[8];
uint8_t CAN2_0x1ff_Tx_Data[8];
uint8_t CAN2_0x200_Tx_Data[8];
uint8_t CAN2_0x2fe_Tx_Data[8];
uint8_t CAN2_0x2ff_Tx_Data[8];
uint8_t CAN2_0x3fe_Tx_Data[8];
uint8_t CAN2_0x4fe_Tx_Data[8];


/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 配置CAN的过滤器
 *
 * @param hcan CAN编号
 * @param Object_Para 编号 | FIFOx | ID类型 | 帧类型
 * @param ID ID
 * @param Mask_ID 屏蔽位(0x3ff, 0x1fffffff)
 */
void can_filter_mask_config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
    CAN_FilterTypeDef can_filter_init_structure;

    //检测传参是否正确
    assert_param(hcan != NULL);

    if ((Object_Para & 0x02))
    {
        // 标准帧
        // 掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = ID << 3 >> 16;
        // 掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = ID << 3 | ((Object_Para & 0x03) << 1);
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 3 << 16;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | ((Object_Para & 0x03) << 1);
    }
    else
    {
        // 扩展帧
        // 掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = ID << 5;
        // 掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = ((Object_Para & 0x03) << 1);
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 5;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = ((Object_Para & 0x03) << 1);
    }
    // 滤波器序号, 0-27, 共28个滤波器, 前14个在CAN1, 后14个在CAN2
    can_filter_init_structure.FilterBank = Object_Para >> 3;
    // 滤波器绑定FIFO0
    can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
    // 使能滤波器
    can_filter_init_structure.FilterActivation = ENABLE;
    // 滤波器模式，设置ID掩码模式
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    // 32位滤波
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    // 从机模式选择开始单元
    can_filter_init_structure.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}

/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function)
{
    HAL_CAN_Start(hcan);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

    if (hcan->Instance == CAN1)
    {
        CAN1_Manage_Object.CAN_Handler = &hcan1;
        CAN1_Manage_Object.Callback_Function = Callback_Function;

        can_filter_mask_config(hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        can_filter_mask_config(hcan, CAN_FILTER(1) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
    else if (hcan->Instance == CAN2)
    {
        CAN2_Manage_Object.CAN_Handler = &hcan2;
        CAN2_Manage_Object.Callback_Function = Callback_Function;

        can_filter_mask_config(hcan, CAN_FILTER(14) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        can_filter_mask_config(hcan, CAN_FILTER(15) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
}

/**
 * @brief 发送数据帧
 *
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    //检测传参是否正确
    assert_param(hcan != NULL);

    tx_header.StdId = ID;
    tx_header.ExtId = 0;
    tx_header.IDE = 0;
    tx_header.RTR = 0;
    tx_header.DLC = Length;

    return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox));
}

/**
 * @brief CAN的TIM定时器中断发送回调函数
 *
 */
void TIM_1ms_CAN_PeriodElapsedCallback()
{
    // DJI电机专属

    static int mod2 = 0;
    mod2++;
    if (mod2 == 2)
    {
        mod2 = 0;

        // CAN2半频电机
        // 舵轮底盘舵向电机
        CAN_Send_Data(&hcan2, 0x1fe, CAN2_0x1fe_Tx_Data, 8);
        // 舵轮底盘轮向电机
        CAN_Send_Data(&hcan2, 0x200, CAN2_0x200_Tx_Data, 8);
    }

}

/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 判断程序初始化完成flag
    if (init_finished == false)
    {
        return;
    }

    // 选择回调函数
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
        if(CAN1_Manage_Object.Callback_Function != nullptr)
        {
            CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
        }
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
        if(CAN2_Manage_Object.Callback_Function != nullptr)
        {
            CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
        }
    }
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 判断程序初始化完成
    if (init_finished == false)
    {
        return;
    }

    // 选择回调函数
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
        if(CAN1_Manage_Object.Callback_Function != nullptr)
        {
            CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
        }
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
        if(CAN2_Manage_Object.Callback_Function != nullptr)
        {
            CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
        }
    }
}

