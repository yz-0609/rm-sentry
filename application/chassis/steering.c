#include "steering.h"

/* 禁用时是空翻译单元：没有外设注册、任务、全局符号或新增依赖。 */
#if STEERING_CANDIDATE_ENABLE

#include <math.h>
#include <string.h>
#include "can_comm.h"
#include "power_control.h"
#include "bsp_dwt.h"
#include "ins_task.h"

#define STEERING_PI 3.14159265358979323846f
#define STEERING_DEG_TO_RAD (STEERING_PI / 180.0f)
#define STEERING_RAD_TO_DEG (180.0f / STEERING_PI)

static SteeringConfig_s steering_config;
static DJIMotorInstance *steering_drive[STEERING_WHEEL_COUNT];
static DJIMotorInstance *steering_turn[STEERING_WHEEL_COUNT];
static CANCommInstance *steering_comm;
static attitude_t *steering_imu;
static Chassis_Ctrl_Cmd_s steering_command;
static Chassis_Upload_Data_s steering_feedback;
static float steering_held_deg[STEERING_WHEEL_COUNT];
static uint32_t steering_last_command_ms;
static uint8_t steering_initialized;
static uint8_t steering_have_command;
static uint8_t steering_have_angles;
static uint8_t steering_running;

static float SteeringWrapDeg(float angle)
{
    angle = fmodf(angle + 180.0f, 360.0f);
    if (angle < 0.0f)
        angle += 360.0f;
    return angle - 180.0f;
}

static uint8_t SteeringPositive(float value)
{
    return (uint8_t)(isfinite(value) && value > 0.0f);
}

static uint8_t SteeringGeometryValid(const SteeringConfig_s *config)
{
    float center_x = 0.0f, center_y = 0.0f, spread = 0.0f;
    if (config == NULL || !SteeringPositive(config->max_drive_rpm) ||
        !SteeringPositive(config->max_translation_mm_s) ||
        !SteeringPositive(config->max_rotation_rad_s) ||
        !isfinite(config->zero_speed_mm_s) || config->zero_speed_mm_s < 0.0f)
        return 0;

    for (uint32_t i = 0; i < STEERING_WHEEL_COUNT; ++i)
    {
        const SteeringWheelConfig_s *wheel = &config->wheel[i];
        if (!isfinite(wheel->x_mm) || !isfinite(wheel->y_mm) ||
            !SteeringPositive(wheel->radius_mm) ||
            !SteeringPositive(wheel->drive_reduction_ratio) ||
            !isfinite(wheel->steer_zero_deg) || wheel->steer_zero_deg < 0.0f ||
            wheel->steer_zero_deg >= 360.0f || wheel->zero_calibrated != 1 ||
            (wheel->drive_direction != 1 && wheel->drive_direction != -1) ||
            (wheel->steer_direction != 1 && wheel->steer_direction != -1))
            return 0;
        center_x += wheel->x_mm * 0.25f;
        center_y += wheel->y_mm * 0.25f;
    }
    for (uint32_t i = 0; i < STEERING_WHEEL_COUNT; ++i)
    {
        float x = config->wheel[i].x_mm - center_x;
        float y = config->wheel[i].y_mm - center_y;
        spread += x * x + y * y;
    }
    return (uint8_t)(isfinite(spread) && spread > 1.0f);
}

static uint8_t SteeringPIDValid(const PID_Init_Config_s *pid, float max_output)
{
    return (uint8_t)(isfinite(pid->Kp) && pid->Kp >= 0.0f &&
                     isfinite(pid->Ki) && pid->Ki >= 0.0f &&
                     isfinite(pid->Kd) && pid->Kd >= 0.0f &&
                     (pid->Kp > 0.0f || pid->Ki > 0.0f) &&
                     SteeringPositive(pid->MaxOut) && pid->MaxOut <= max_output &&
                     isfinite(pid->IntegralLimit) && pid->IntegralLimit >= 0.0f &&
                     isfinite(pid->DeadBand) && pid->DeadBand >= 0.0f &&
                     isfinite(pid->CoefA) && pid->CoefA >= 0.0f &&
                     isfinite(pid->CoefB) && pid->CoefB >= 0.0f &&
                     isfinite(pid->Output_LPF_RC) && pid->Output_LPF_RC >= 0.0f &&
                     isfinite(pid->Derivative_LPF_RC) && pid->Derivative_LPF_RC >= 0.0f);
}

static uint8_t SteeringConfigValid(const SteeringConfig_s *config)
{
    return (uint8_t)(SteeringGeometryValid(config) &&
        SteeringPIDValid(&config->drive_speed_pid, 16000.0f) &&
        SteeringPIDValid(&config->steer_angle_pid, 3600.0f) &&
        SteeringPIDValid(&config->steer_speed_pid, 30000.0f) &&
        SteeringPositive(config->command_linear_scale) &&
        SteeringPositive(config->command_angular_scale) &&
        isfinite(config->follow_gain) && config->follow_gain >= 0.0f &&
        isfinite(config->navigation_yaw_offset_deg) &&
        (config->imu_yaw_direction == 1 || config->imu_yaw_direction == -1) &&
        config->navigation_frame_calibrated <= 1 &&
        config->command_timeout_ms >= 20 && config->command_timeout_ms <= 1000 &&
        (config->comm_can == &hcan1 || config->comm_can == &hcan2) &&
        config->referee != NULL && config->referee->init_flag);
}

uint8_t SteeringCalculate(const SteeringConfig_s *config, float vx_mm_s,
                          float vy_mm_s, float wz_rad_s,
                          const float current_motor_deg[STEERING_WHEEL_COUNT],
                          const float held_motor_deg[STEERING_WHEEL_COUNT],
                          SteeringWheelTarget_s target[STEERING_WHEEL_COUNT])
{
    SteeringWheelTarget_s calculated[STEERING_WHEEL_COUNT] = {0};
    float max_rpm = 0.0f;
    if (target == NULL)
        return 0;
    if (!SteeringGeometryValid(config) || current_motor_deg == NULL ||
        held_motor_deg == NULL || !isfinite(vx_mm_s) ||
        !isfinite(vy_mm_s) || !isfinite(wz_rad_s))
        goto invalid;

    float translation = hypotf(vx_mm_s, vy_mm_s);
    if (!isfinite(translation))
        goto invalid;
    if (translation > config->max_translation_mm_s)
    {
        float scale = config->max_translation_mm_s / translation;
        vx_mm_s *= scale;
        vy_mm_s *= scale;
    }
    wz_rad_s = fmaxf(-config->max_rotation_rad_s,
                     fminf(wz_rad_s, config->max_rotation_rad_s));

    for (uint32_t i = 0; i < STEERING_WHEEL_COUNT; ++i)
    {
        const SteeringWheelConfig_s *wheel = &config->wheel[i];
        if (!isfinite(current_motor_deg[i]) || !isfinite(held_motor_deg[i]))
            goto invalid;
        float vx = vx_mm_s - wz_rad_s * wheel->y_mm;
        float vy = vy_mm_s + wz_rad_s * wheel->x_mm;
        float speed = hypotf(vx, vy);
        if (!isfinite(speed))
            goto invalid;
        if (speed <= config->zero_speed_mm_s)
        {
            calculated[i].steer_motor_deg = held_motor_deg[i];
            continue;
        }

        float current_deg = wheel->steer_direction *
                            (current_motor_deg[i] - wheel->steer_zero_deg);
        float error = SteeringWrapDeg(atan2f(vy, vx) * STEERING_RAD_TO_DEG -
                                      SteeringWrapDeg(current_deg));
        /* 转向超过 90 度时，反向驱动以缩短舵向行程。 */
        if (fabsf(error) > 90.0f)
        {
            error = SteeringWrapDeg(error + 180.0f);
            speed = -speed;
        }
        calculated[i].steer_motor_deg = current_motor_deg[i] +
                                        wheel->steer_direction * error;
        /* 未转到目标方向时降低驱动，减少侧向拖拽。 */
        float alignment = fmaxf(0.0f, cosf(error * STEERING_DEG_TO_RAD));
        calculated[i].drive_rpm = wheel->drive_direction * speed * alignment *
            wheel->drive_reduction_ratio * 60.0f / (2.0f * STEERING_PI * wheel->radius_mm);
        if (!isfinite(calculated[i].steer_motor_deg) ||
            !isfinite(calculated[i].drive_rpm))
            goto invalid;
        max_rpm = fmaxf(max_rpm, fabsf(calculated[i].drive_rpm));
    }

    /* 按同一比例限制四轮，避免分别截断造成额外运动偏差。 */
    if (max_rpm > config->max_drive_rpm)
    {
        float scale = config->max_drive_rpm / max_rpm;
        for (uint32_t i = 0; i < STEERING_WHEEL_COUNT; ++i)
            calculated[i].drive_rpm *= scale;
    }
    memcpy(target, calculated, sizeof(calculated));
    return 1;

invalid:
    memset(target, 0, sizeof(calculated));
    return 0;
}

uint8_t SteeringInit(const SteeringConfig_s *config)
{
    if (steering_initialized || !SteeringConfigValid(config))
        return 0;
    steering_config = *config;
    steering_imu = INS_Init(); /* 本板 IMU，不能依赖底盘板未初始化的云台实例。 */
    if (steering_imu == NULL)
        return 0;

    Motor_Init_Config_s drive_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config.speed_PID = config->drive_speed_pid,
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    Motor_Init_Config_s turn_config = {
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            .angle_PID = config->steer_angle_pid,
            .speed_PID = config->steer_speed_pid,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = GM6020,
    };
    for (uint32_t i = 0; i < STEERING_WHEEL_COUNT; ++i)
    {
        /* 这里填写的是硬件编号 1..4，不能填写反馈 ID 0x201/0x205。 */
        drive_config.can_init_config.tx_id = i + 1u;
        turn_config.can_init_config.tx_id = i + 1u;
        steering_drive[i] = PowerControlInit(&drive_config);
        DJIMotorStop(steering_drive[i]);
        steering_turn[i] = DJIMotorInit(&turn_config);
        DJIMotorStop(steering_turn[i]);
    }

    CANComm_Init_Config_s comm_config = {
        .can_config = {
            .can_handle = config->comm_can,
            .tx_id = STEERING_COMM_TX_ID,
            .rx_id = STEERING_COMM_RX_ID,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
        .daemon_count = (config->command_timeout_ms + 9u) / 10u,
    };
    steering_comm = CANCommInit(&comm_config);
    SetPowerLimit((float)config->referee->GameRobotState.chassis_power_limit);
    steering_initialized = 1;
    return 1;
}

void SteeringStop(void)
{
    if (!steering_initialized)
        return;
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    for (uint32_t i = 0; i < STEERING_WHEEL_COUNT; ++i)
    {
        DJIMotorSetRef(steering_drive[i], 0.0f);
        DJIMotorStop(steering_drive[i]);
        DJIMotorStop(steering_turn[i]);
    }
    steering_running = 0;
    /* 重启或离线恢复后用实际舵角重新建立零速保持目标。 */
    steering_have_angles = 0;
    __set_PRIMASK(irq_state);
}

static uint8_t SteeringReadMotors(float angle[STEERING_WHEEL_COUNT],
                                 float speed[STEERING_WHEEL_COUNT])
{
    uint8_t online = 1;
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    for (uint32_t i = 0; i < STEERING_WHEEL_COUNT; ++i)
    {
        /* Daemon 的启动计数不等于已收到电机帧；feed_cnt 须已被回调更新。 */
        if (!steering_drive[i]->feed_cnt || !steering_turn[i]->feed_cnt ||
            !DaemonIsOnline(steering_drive[i]->daemon) ||
            !DaemonIsOnline(steering_turn[i]->daemon))
            online = 0;
        angle[i] = steering_turn[i]->measure.total_angle;
        speed[i] = steering_drive[i]->measure.speed_aps;
        if (!isfinite(angle[i]) || !isfinite(speed[i]))
            online = 0;
    }
    __set_PRIMASK(irq_state);
    return online;
}

static void SteeringEstimate(const float angle[STEERING_WHEEL_COUNT],
                             const float speed[STEERING_WHEEL_COUNT])
{
    float mean_x = 0.0f, mean_y = 0.0f;
    float mean_vx = 0.0f, mean_vy = 0.0f, rotation = 0.0f, spread = 0.0f;
    float wheel_vx[STEERING_WHEEL_COUNT], wheel_vy[STEERING_WHEEL_COUNT];
    for (uint32_t i = 0; i < STEERING_WHEEL_COUNT; ++i)
    {
        const SteeringWheelConfig_s *wheel = &steering_config.wheel[i];
        float wheel_angle = SteeringWrapDeg(wheel->steer_direction *
                           (angle[i] - wheel->steer_zero_deg)) * STEERING_DEG_TO_RAD;
        float linear_speed = speed[i] * STEERING_DEG_TO_RAD * wheel->radius_mm *
                             wheel->drive_direction / wheel->drive_reduction_ratio;
        wheel_vx[i] = linear_speed * cosf(wheel_angle);
        wheel_vy[i] = linear_speed * sinf(wheel_angle);
        mean_x += wheel->x_mm * 0.25f;
        mean_y += wheel->y_mm * 0.25f;
        mean_vx += wheel_vx[i] * 0.25f;
        mean_vy += wheel_vy[i] * 0.25f;
    }
    /* 无侧滑假设下的最小二乘速度估计，允许轮组几何中心偏离坐标原点。 */
    for (uint32_t i = 0; i < STEERING_WHEEL_COUNT; ++i)
    {
        float x = steering_config.wheel[i].x_mm - mean_x;
        float y = steering_config.wheel[i].y_mm - mean_y;
        rotation += -y * wheel_vx[i] + x * wheel_vy[i];
        spread += x * x + y * y;
    }
    float wz = rotation / spread;
    float vx = (mean_vx + wz * mean_y) * 0.001f;
    float vy = (mean_vy - wz * mean_x) * 0.001f;
    if (isfinite(vx) && isfinite(vy) && isfinite(wz))
    {
        steering_feedback.real_vx = vx;
        steering_feedback.real_vy = vy;
        steering_feedback.real_wz = wz;
    }
    else
    {
        steering_feedback.real_vx = 0.0f;
        steering_feedback.real_vy = 0.0f;
        steering_feedback.real_wz = 0.0f;
    }
}

static uint8_t SteeringCommandVelocity(float *vx, float *vy, float *wz)
{
    const Chassis_Ctrl_Cmd_s *cmd = &steering_command;
    float rotation, angle;
    if (!isfinite(cmd->vx) || !isfinite(cmd->vy) || !isfinite(cmd->wz) ||
        !isfinite(cmd->offset_angle) || !isfinite(cmd->chassis_spin_speed))
        return 0;
    switch (cmd->chassis_mode)
    {
    case CHASSIS_NO_FOLLOW:
        rotation = 0.0f;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW:
        rotation = steering_config.follow_gain * cmd->offset_angle;
        break;
    case CHASSIS_ROTATE:
        rotation = cmd->chassis_spin_speed;
        break;
    default: /* 零力矩以及未知枚举值均停止。 */
        return 0;
    }

    if (cmd->mode == 2)
        angle = cmd->offset_angle; /* 保留现有云台系旋转矩阵的符号。 */
    else if (cmd->mode == 1 && steering_config.navigation_frame_calibrated)
    {
        /* 导航参考要实测对齐；本板 Yaw 已是底盘角，不再减云台 offset。 */
        angle = steering_config.imu_yaw_direction * steering_imu->Yaw +
                steering_config.navigation_yaw_offset_deg;
    }
    else
        return 0;
    if (!isfinite(angle))
        return 0;

    angle = SteeringWrapDeg(angle) * STEERING_DEG_TO_RAD;
    float cos_angle = cosf(angle), sin_angle = sinf(angle);
    *vx = (cmd->vx * cos_angle - cmd->vy * sin_angle) * steering_config.command_linear_scale;
    *vy = (cmd->vx * sin_angle + cmd->vy * cos_angle) * steering_config.command_linear_scale;
    *wz = rotation * steering_config.command_angular_scale;
    return (uint8_t)(isfinite(*vx) && isfinite(*vy) && isfinite(*wz));
}

static void SteeringApply(const SteeringWheelTarget_s target[STEERING_WHEEL_COUNT])
{
    /* 电机任务也会访问 PID 和参考值，更新期间禁止调度打断。 */
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    for (uint32_t i = 0; i < STEERING_WHEEL_COUNT; ++i)
    {
        if (!steering_running)
        {
            /* 原驱动在 STOP 时仍算 PID，恢复前清除停机期间的积分。 */
            PIDInit(&steering_drive[i]->motor_controller.speed_PID,
                    &steering_config.drive_speed_pid);
            PIDInit(&steering_turn[i]->motor_controller.angle_PID,
                    &steering_config.steer_angle_pid);
            PIDInit(&steering_turn[i]->motor_controller.speed_PID,
                    &steering_config.steer_speed_pid);
        }
        DJIMotorSetRef(steering_drive[i], target[i].drive_rpm);
        DJIMotorSetRef(steering_turn[i], target[i].steer_motor_deg);
        steering_held_deg[i] = target[i].steer_motor_deg;
        DJIMotorEnable(steering_drive[i]);
        DJIMotorEnable(steering_turn[i]);
    }
    steering_running = 1;
    __set_PRIMASK(irq_state);
}

void SteeringTask(void)
{
    float angle[STEERING_WHEEL_COUNT], speed[STEERING_WHEEL_COUNT];
    float vx, vy, wz;
    SteeringWheelTarget_s target[STEERING_WHEEL_COUNT];
    if (!steering_initialized)
        return;

    uint32_t now = (uint32_t)DWT_GetTimeline_ms();
    /* 与接收 ISR 互斥，避免半包控制命令或丢失新帧 update_flag。 */
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    if (steering_comm->update_flag)
    {
        memcpy(&steering_command, CANCommGet(steering_comm), sizeof(steering_command));
        steering_last_command_ms = now;
        steering_have_command = 1;
    }
    __set_PRIMASK(irq_state);

    uint8_t motors_online = SteeringReadMotors(angle, speed);
    if (motors_online)
    {
        SteeringEstimate(angle, speed);
        if (!steering_have_angles)
        {
            memcpy(steering_held_deg, angle, sizeof(angle));
            steering_have_angles = 1;
        }
    }
    else
    {
        steering_feedback.real_vx = 0.0f;
        steering_feedback.real_vy = 0.0f;
        steering_feedback.real_wz = 0.0f;
    }
    SetPowerLimit((float)steering_config.referee->GameRobotState.chassis_power_limit);
    if (!motors_online || !steering_have_command ||
        (uint32_t)(now - steering_last_command_ms) > steering_config.command_timeout_ms ||
        !CANCommIsOnline(steering_comm) || !SteeringCommandVelocity(&vx, &vy, &wz) ||
        !SteeringCalculate(&steering_config, vx, vy, wz, angle, steering_held_deg, target))
        SteeringStop();
    else
        SteeringApply(target);

    /* 继续使用现有结构和 ID；不新增字段、不改远端协议、不伪造裁判信息。 */
    CANCommSend(steering_comm, (uint8_t *)&steering_feedback);
}

const Chassis_Upload_Data_s *SteeringGetFeedback(void)
{
    return steering_initialized ? &steering_feedback : NULL;
}

#endif /* STEERING_CANDIDATE_ENABLE */
