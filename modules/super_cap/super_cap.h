/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:25:46
 */
#ifndef SUPER_CAP_H
#define SUPER_CAP_H

#include "bsp_can.h"
#include "super_cap_sdk.h" // 引入新SDK头文件

// #pragma pack(1)
// typedef struct
// {
//     uint16_t vol; // 电压
//     uint16_t current; // 电流
//     uint16_t power; // 功率
// } SuperCap_Msg_s;
// #pragma pack()

/* 超级电容实例 */
typedef struct
{
    CANInstance *can_ins;
    SuperCap_Feedback_t feedback; // 替换原来的 SuperCap_Msg_s
    SuperCap_Control_t control;   // 新增控制参数缓存
} SuperCapInstance;

/* 超级电容初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;
} SuperCap_Init_Config_s;

// 函数声明更新
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config);
void SuperCapSend(SuperCapInstance *instance, SuperCap_Control_t *control_data);
SuperCap_Feedback_t SuperCapGet(SuperCapInstance *instance);
SuperCapInstance *GetSuperCapInstance(void); 


#endif // !SUPER_CAP_Hd
