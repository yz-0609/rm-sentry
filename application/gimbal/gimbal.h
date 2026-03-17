#ifndef GIMBAL_H
#define GIMBAL_H

#include "ins_task.h"


/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();


attitude_t* get_IMU_data(void);
/**
 * @brief 云台任务
 * 
 */
void GimbalTask();

#endif // GIMBAL_H