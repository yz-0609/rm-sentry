#ifndef SENTRY_STEERING_H
#define SENTRY_STEERING_H

/*
 * 独立的“舵轮底盘 + 双板”候选模块，默认完全不参与运行。
 * 本文件不改变 ONE_BOARD / CHASSIS_BOARD / GIMBAL_BOARD。
 * 后续接入时必须在独立底盘板固件中启用，并用 SteeringInit/SteeringTask
 * 替代原底盘入口；绝对不能与 ChassisInit/ChassisTask 同时运行。
 * CMake 重新配置后会收集 steering.c；Makefile 需要后续单独接入。
 */
#ifndef STEERING_CANDIDATE_ENABLE
#define STEERING_CANDIDATE_ENABLE 0
#endif

#if (STEERING_CANDIDATE_ENABLE != 0) && (STEERING_CANDIDATE_ENABLE != 1)
#error "STEERING_CANDIDATE_ENABLE must be 0 or 1"
#endif

#if STEERING_CANDIDATE_ENABLE

#include <stdint.h>
#include "robot_def.h"
#include "dji_motor.h"
#include "rm_referee.h"

#if !defined(CHASSIS_BOARD) || defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#error "Steering candidate requires an exclusive CHASSIS_BOARD build"
#endif

#define STEERING_WHEEL_COUNT 4u
#define STEERING_DRIVE_RX_BASE 0x201u
#define STEERING_STEER_RX_BASE 0x205u
#define STEERING_DRIVE_TX_ID 0x200u
#define STEERING_STEER_TX_ID 0x1FFu
#define STEERING_COMM_TX_ID 0x311u
#define STEERING_COMM_RX_ID 0x312u

/* wheel[i] 对应硬件 ID i+1，不假设它在车上的位置。
 * CAN1: M3508 ID 1..4 -> 反馈 0x201..0x204，控制 0x200。
 * CAN2: GM6020 ID 1..4 -> 反馈 0x205..0x208，控制 0x1FF。
 * 坐标约定：x 向前、y 向右，正 wz 从 x 转向 y（俯视顺时针）。
 * 6020 候选按舵向直驱、可连续旋转处理；带舵向减速器/限位的机构需另行适配。
 */
typedef struct
{
    float x_mm;
    float y_mm;
    float radius_mm;
    float drive_reduction_ratio;
    float steer_zero_deg;       /* 正前方对应的 6020 单圈角度 [0,360)。 */
    int8_t drive_direction;     /* +1/-1：电机正转是否让轮子沿舵向前进。 */
    int8_t steer_direction;     /* +1/-1：编码器增大是否沿坐标正角方向。 */
    uint8_t zero_calibrated;    /* 标定完成后设为 1；不采用虚构零位。 */
} SteeringWheelConfig_s;

typedef struct
{
    SteeringWheelConfig_s wheel[STEERING_WHEEL_COUNT];
    PID_Init_Config_s drive_speed_pid; /* RPM -> 3508 控制量。 */
    PID_Init_Config_s steer_angle_pid; /* deg -> deg/s。 */
    PID_Init_Config_s steer_speed_pid; /* deg/s -> 6020 控制量。 */
    float max_drive_rpm;
    float max_translation_mm_s;
    float max_rotation_rad_s;
    float zero_speed_mm_s;             /* 此速度以内保持上次舵向，驱动为零。 */
    float command_linear_scale;        /* 现有 vx/vy 数值 -> mm/s，必须实测确认。 */
    float command_angular_scale;       /* 现有 wz/spin 数值 -> rad/s。 */
    float follow_gain;                 /* 保留跟随行为，单位为命令 wz / deg。 */
    float navigation_yaw_offset_deg;   /* 本板 IMU 与导航角度参考之间的标定偏置。 */
    int8_t imu_yaw_direction;          /* 本板 IMU 到原旋转矩阵角度的符号 +1/-1。 */
    uint8_t navigation_frame_calibrated; /* 未标定时拒绝 mode=1；mode=2 不受此项影响。 */
    uint16_t command_timeout_ms;       /* 20..1000ms；从第一帧有效 CANComm 数据开始计时。 */
    CAN_HandleTypeDef *comm_can;       /* &hcan1 或 &hcan2，通常 &hcan2。 */
    referee_info_t *referee;           /* 调用者已完成裁判/UI 初始化，有效期须覆盖运行期。 */
} SteeringConfig_s;

typedef struct
{
    float drive_rpm;
    float steer_motor_deg; /* 与驱动的 total_angle 在同一连续角度空间。 */
} SteeringWheelTarget_s;

/* 只计算，不注册外设，不修改全局功率限制。速度使用 mm/s、rad/s。
 * current_motor_deg 来自反馈；held_motor_deg 来自上次目标（首次可用当前反馈）。
 * 返回 0 时 target 全部清零，调用者必须停止电机，不能将清零舵角作为运动目标。
 */
uint8_t SteeringCalculate(const SteeringConfig_s *config, float vx_mm_s,
                          float vy_mm_s, float wz_rad_s,
                          const float current_motor_deg[STEERING_WHEEL_COUNT],
                          const float held_motor_deg[STEERING_WHEEL_COUNT],
                          SteeringWheelTarget_s target[STEERING_WHEEL_COUNT]);

/* 启动 RTOS 前调用一次；无默认 PID/零位；配置失败不注册任何资源。
 * 裁判/UI、可选超电由调用者初始化，本模块不重复初始化这些共享服务。
 * 注册 4 个 PowerControl 驱动电机、4 个 DJI 舵向电机、1 个 CANComm，并取本板 INS。
 * 确保这些 CAN ID/控制分组在整个物理总线上独占，包括对端云台的电机。
 * 现有 PowerControl 仅覆盖四个 3508；不能视作八电机总功率限制。
 * 返回 1 为成功，重复初始化/无效配置返回 0。
 */
uint8_t SteeringInit(const SteeringConfig_s *config);
void SteeringTask(void); /* 与原 RobotTask 相同的约 5ms 周期，后续显式接入。 */
void SteeringStop(void);
/* 保留原双板反馈结构，real_vx/real_vy 为 m/s，real_wz 为 rad/s。 */
const Chassis_Upload_Data_s *SteeringGetFeedback(void);

#endif /* STEERING_CANDIDATE_ENABLE */
#endif /* SENTRY_STEERING_H */
