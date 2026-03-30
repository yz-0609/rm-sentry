#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

// static float pitch_angle;

static float target_pitch,target_yaw;

static float yaw_speed_ff = 0; // 定义yaw电机速度前馈变量
static float last_target_yaw;  // 用于保存上一次的目标角度



static float ref_yaw,ref_pitch;

static BMI088Instance *bmi088; // 云台IMU

attitude_t* get_IMU_data(void)
{
    return gimba_IMU_data;
}



void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    //YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,    //后改为CAN2
            .tx_id = 4,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 1.0, // 1.0  
                .Ki = 0,
                .Kd = 1.2, // 1.2
                .DeadBand = 0,//0.1
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement, //,//PID_IMPROVE_NONE//PID_Trapezoid_Intergral |
                .IntegralLimit = 200,

                .MaxOut = 400,
            },
            .speed_PID = {
                .Kp = 1550,  // 1500
                .Ki = 45, // 30  35
                .Kd = 0,
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,//PID_Trapezoid_Intergral |PID_IMPROVE_NONE,// 
                .IntegralLimit = 10000,
                .MaxOut = 30000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,   //YAW电机的角度反馈和速度反馈设为来自IMU

            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
            .speed_feedforward_ptr = &yaw_speed_ff, // 传入速度前馈指针
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,    //编码器速度反馈
            // .feedforward_flag = SPEED_FEEDFORWARD,  // 启用速度前馈
            .outer_loop_type = ANGLE_LOOP,
            //.close_loop_type = SPEED_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            // .close_loop_type = ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = GM6020};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 800, // 2500   2000    2100   1.5
                .Ki = 50,
                .Kd = 150,//120   450   120    0.5
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 200,
                .MaxOut = 30000,
            },
            .speed_PID = {
                .Kp = 1700,  // 6500
                .Ki = 55, // 
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10000,   //2500
                .MaxOut = 30000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[1],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转

            // .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .close_loop_type = ANGLE_LOOP,

            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = GM6020,
    };
    //电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);   
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        // ref_yaw=-gimba_IMU_data->Gyro[2];

        // target_pitch=gimbal_cmd_recv.pitch;
        // ref_pitch=-gimba_IMU_data->Gyro[1];
        // target_pitch=gimbal_cmd_recv.pitch;
        // ref_pitch=-gimba_IMU_data->Pitch;

        target_yaw=gimbal_cmd_recv.yaw;
        ref_yaw=-gimba_IMU_data->YawTotalAngle;       

        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    default:
        break;
    }





    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    yaw_speed_ff =-(gimbal_cmd_recv.yaw - last_target_yaw) * 10.0f;
    last_target_yaw = gimbal_cmd_recv.yaw;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}