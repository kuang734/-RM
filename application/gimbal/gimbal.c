#include "gimbal.h"
#include "robot_def.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "user_lib.h"
#include "bmi088.h"
#include "remote_control.h" // 遥控器: 单环调试yaw用左拨杆启动转动
#include "bsp_log.h"        // 临时调试: LOGINFO 打印

#define GIMBAL_DEG2RAD (PI / 180.0f) // robot_cmd 下发的 yaw/pitch 单位为度, 达妙反馈为 rad
#define GIMBAL_FEEDBACK_GYRO (0) // 速度环反馈源开关: 1=陀螺仪IMU(Gyro[2]yaw/Gyro[1]pitch), 0=达妙编码器; 内环已定死只用电机编码器反馈, 陀螺仪分支保留但不编译

static attitude_t *gimba_IMU_data; // 云台IMU数据
static RC_ctrl_t *rc_data;         // 遥控器数据(单环调试yaw用左拨杆启动转动)
// 达妙 J4310 云台电机: yaw/pitch 均在云台板(CAN1), 使用达妙自身编码器反馈(支持多圈)
static DMMotorInstance *dm_yaw_motor, *dm_pitch_motor;

float g_yaw_torque_ref = 0.3f;   // yaw 开环力矩参考值 N·m, 调试时可在调试器在线修改
float g_yaw_vel_feedback = 0.0f; // yaw 电机反馈速度 rad/s(达妙编码器), 调试时观测
float g_yaw_vel_ref = 3.0f;      // yaw 目标速度 rad/s, 调试器可在线修改(实际speed_PID Kp=0.1f, 目标5.0×0.1=0.5 N·m才推得动)
float g_pitch_vel_ref = -3.0f;      // pitch 目标速度 rad/s, 调试器可在线修改(初始0=阻尼定位, 调pitch时再改非零; 注意调pitch时把g_yaw_vel_ref改0避免yaw同时转)
float g_pitch_vel_feedback = 0.0f; // pitch 电机反馈速度 rad/s(达妙编码器), 调试时观测

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static BMI088Instance *bmi088; // 云台IMU
void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // 以下 yaw 电机初始化已按架构图移除(yaw移到底盘板 chassis.c 中初始化),保留注释以便恢复
    // YAW
    // Motor_Init_Config_s yaw_config = {
    //     .can_init_config = {
    //         .can_handle = &hcan1,
    //         .tx_id = 1,
    //     },
    //     .controller_param_init_config = {
    //         .angle_PID = {
    //             .Kp = 8, // 8
    //             .Ki = 0,
    //             .Kd = 0,
    //             .DeadBand = 0.1,
    //             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //             .IntegralLimit = 100,
    //
    //             .MaxOut = 500,
    //         },
    //         .speed_PID = {
    //             .Kp = 50,  // 50
    //             .Ki = 200, // 200
    //             .Kd = 0,
    //             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //             .IntegralLimit = 3000,
    //             .MaxOut = 20000,
    //         },
    //         .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
    //         // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
    //         .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = OTHER_FEED,
    //         .speed_feedback_source = OTHER_FEED,
    //         .outer_loop_type = ANGLE_LOOP,
    //         .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
    //         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    //     },
    //     .motor_type = GM6020};
    // YAW 达妙J4310: 上位机 CAN_ID=0x1, Master_ID=0x11
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x0001, // = 电机 CAN ID(控制帧)
            .rx_id = 0x0011, // = 电机 Master ID(反馈帧)
        },
        .controller_param_init_config = {
            // 位置环: 输入误差 rad -> 输出 rad/s(作为速度环目标)
            .angle_PID = {
                .Kp = 0.01f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Derivative_On_Measurement | PID_Integral_Limit,
                .IntegralLimit = 0.5f,
                .MaxOut = 8.0f, // 最大速度目标 rad/s
            },
            // 速度环: 输入误差 rad/s -> 输出 N·m(力矩)
            .speed_PID = {
                .Kp = 0.53f,
                .Ki = 0.15f,
                .Kd = 0.0f,
                .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,
                .IntegralLimit = 1.0f,
                .MaxOut = 12.0f, // 最大力矩 N·m, 限幅在 DM_T 内
            },
        },
        .controller_setting_init_config = {
            // 使用达妙自身编码器反馈(非IMU), 支持多圈累计
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE, // 开环实测: +0.3 N·m 时 yaw 朝左转, 与约定正方向(推右=右偏航)相反, 反转
        },
        .motor_type = MOTOR_TYPE_NONE,
    };
    // PITCH 达妙J4310: 上位机 CAN_ID=0x2, Master_ID=0x12
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x0002,
            .rx_id = 0x0012,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.01f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Derivative_On_Measurement | PID_Integral_Limit,
                .IntegralLimit = 0.5f,
                .MaxOut = 8.0f,
            },
            .speed_PID = {
                .Kp = 0.43f,
                .Ki = 0.05f,
                .Kd = 0.0f,
                .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,
                .IntegralLimit = 1.0f,
                .MaxOut = 12.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE, // 开环实测: +0.3 N·m 时 pitch 往下转, 与约定正方向(抬头)相反, 反转
        },
        .motor_type = MOTOR_TYPE_NONE,
    };
    // 达妙上电自动标零(0xFE), 先摆好云台中位再上电
    dm_yaw_motor = DMMotorInit(&yaw_config);
    dm_pitch_motor = DMMotorInit(&pitch_config);

    rc_data = GetRemoteControlData(); // 获取已初始化的遥控器数据(不重复初始化USART)

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // 位置环输出(rad/s), 作为速度环目标
    float yaw_vel_ref, pitch_vel_ref;
    
    // 达妙J4310: MIT力控模式下位置/速度环在主控计算, 单位统一 rad / rad/s / N·m
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止(零力矩)
    case GIMBAL_ZERO_FORCE:
        DMMotorStop(dm_yaw_motor);
        DMMotorStop(dm_pitch_motor);
        DMMotorSetRef(dm_yaw_motor, 0.0f);
        DMMotorSetRef(dm_pitch_motor, 0.0f);
        break;
    // 陀螺仪模式/自由模式: IMU 姿态反馈(替代电机编码器)
    case GIMBAL_GYRO_MODE:
    case GIMBAL_FREE_MODE:
        DMMotorEnable(dm_yaw_motor);
        DMMotorEnable(dm_pitch_motor);
        // yaw: 目标角度(度→rad) -> 位置环(IMU多圈角) -> 速度环(IMU角速度) -> 力矩
        /* 单环调试pitch期间临时停用yaw
        yaw_vel_ref = PIDCalculate(&dm_yaw_motor->angle_PID,
                                   gimba_IMU_data->YawTotalAngle * GIMBAL_DEG2RAD,
                                   gimbal_cmd_recv.yaw * GIMBAL_DEG2RAD);
        yaw_vel_ref = 0.0f; // 单环调试: 目标角速度固定0 rad/s(阻尼测试), 可改固定值验证方向
        DMMotorSetRef(dm_yaw_motor,
                      PIDCalculate(&dm_yaw_motor->speed_PID,
                                   gimba_IMU_data->Gyro[2], yaw_vel_ref));
        */
        /* 单环调试pitch期间临时停用yaw
        DMMotorStop(dm_yaw_motor);
        DMMotorSetRef(dm_yaw_motor, 0.0f); // 单环调试pitch期间临时停用yaw
        */
        /* 开环方向验证: 固定正力矩+0.3 N·m, 不依赖反馈, 观察转向
        DMMotorSetRef(dm_yaw_motor, 0.18f); // 开环方向验证: 固定正力矩+0.3 N·m, 不依赖反馈, 观察转向
        */
        /* 单环调试: 目标角速度固定0 rad/s(阻尼测试), 反馈=IMU Gyro[2]
        yaw_vel_ref = 0.0f; // 单环调试: 目标角速度固定0 rad/s(阻尼测试), 可改固定值验证方向
        DMMotorSetRef(dm_yaw_motor, -PIDCalculate(&dm_yaw_motor->speed_PID, gimba_IMU_data->Gyro[2], yaw_vel_ref)); // REVERSE下输出取反恢复负反馈
        */
        // yaw: 速度环调试, 左拨杆[下]启动, 目标速度g_yaw_vel_ref(调试器可在线修改), 反馈=达妙编码器速度
        // 左拨杆[下]才进入robot_cmd的RemoteControlSet遥控器控制模式并设置gimbal_mode为GYRO/FREE(右拨杆需在[下]或[中]); [上]是键鼠模式不设置gimbal_mode, 云台进不了GYRO/FREE分支
        /* 原开环: 左拨杆[下]固定力矩(调试器在线改g_yaw_torque_ref), 改用速度环后注释保留
        if (rc_data->rc.switch_left == RC_SW_DOWN)
            DMMotorSetRef(dm_yaw_motor, g_yaw_torque_ref); // 开环力矩用全局变量, 调试器可在线修改(初始0.5 N·m, 实测0.18f推不动)
        else
            DMMotorSetRef(dm_yaw_motor, 0.0f); // 其他档位: 零力矩
        */
        if (rc_data->rc.switch_left == RC_SW_DOWN)
        {
#if GIMBAL_FEEDBACK_GYRO
            // yaw+陀螺仪反馈符号测试: Gyro[2]右偏航为正; 输出取负为历史结论, 若方向反去掉负号
            DMMotorSetRef(dm_yaw_motor, -PIDCalculate(&dm_yaw_motor->speed_PID, gimba_IMU_data->Gyro[2], g_yaw_vel_ref));
#else
            DMMotorSetRef(dm_yaw_motor, -PIDCalculate(&dm_yaw_motor->speed_PID, dm_yaw_motor->measure.velocity, g_yaw_vel_ref)); // yaw速度环: 目标g_yaw_vel_ref, 反馈编码器速度(REVERSE下输出取反恢复负反馈)
#endif
        }
        else
        {
            // 左拨杆不在[下]: 清零速度环积分, 保证下次打开时从零开始, 不残留历史误差
            dm_yaw_motor->speed_PID.Iout = 0.0f;
            dm_yaw_motor->speed_PID.ITerm = 0.0f;
            dm_yaw_motor->speed_PID.Output = 0.0f;
            DMMotorSetRef(dm_yaw_motor, 0.0f); // 其他档位: 零力矩
        }
        // pitch 同构: 角度用EKF输出的Pitch(度→rad), 速度用Gyro[1](实测: pitch轴对应Y轴)
        /* 单环调试: 位置环暂注释保留
        pitch_vel_ref = PIDCalculate(&dm_pitch_motor->angle_PID,
                                     gimba_IMU_data->Pitch * GIMBAL_DEG2RAD,
                                     gimbal_cmd_recv.pitch * GIMBAL_DEG2RAD);
        DMMotorStop(dm_pitch_motor);
        DMMotorSetRef(dm_pitch_motor, 0.0f); // 单环调试yaw期间临时停用pitch
        */
        /* 单环调试: 目标角速度固定0 rad/s(阻尼测试), 可改固定值验证方向
        pitch_vel_ref = 0.0f; // 单环调试: 目标角速度固定0 rad/s(阻尼测试), 可改固定值验证方向
        DMMotorSetRef(dm_pitch_motor,
                      PIDCalculate(&dm_pitch_motor->speed_PID,
                                   gimba_IMU_data->Gyro[1], pitch_vel_ref));
        */
        /* 开环方向验证: 固定正力矩+0.3 N·m, 不依赖反馈, 观察转向
        DMMotorSetRef(dm_pitch_motor, 0.2f); // 开环方向验证: 固定正力矩+0.3 N·m, 不依赖反馈, 观察转向
        */
        /* 开环调试yaw期间临时停用pitch
        DMMotorStop(dm_pitch_motor);
        DMMotorSetRef(dm_pitch_motor, 0.0f); // 开环调试yaw期间临时停用pitch
        */
        // /*// /* 单环调试: 目标角速度固定0 rad/s(阻尼测试), 反馈=IMU Gyro[1]
        // // pitch_vel_ref = 0.0f; // 单环调试: 目标角速度固定0 rad/s(阻尼测试), 可改固定值验证方向
        // // DMMotorSetRef(dm_pitch_motor, -PIDCalculate(&dm_pitch_motor->speed_PID, gimba_IMU_data->Gyro[1], pitch_vel_ref)); // pitch轴Gyro[1]符号与yaw相反(抬头为负), 不加负号即为负反馈
        // // 
        // /* 单环调试yaw期间临时停用pitch
        // DMMotorStop(dm_pitch_motor);
        // DMMotorSetRef(dm_pitch_motor, 0.0f); // 单环调试yaw期间临时停用pitch
        // */
        // pitch: 速度环调试, 与yaw共用左拨杆[下]启动, 目标速度g_pitch_vel_ref(调试器在线改), 反馈=达妙编码器速度
        // pitch轴有重力恒扰动, 纯P稳态残差会偏大, 后续需加I(变速积分)或重力前馈
        if (rc_data->rc.switch_left == RC_SW_DOWN)
        {
#if GIMBAL_FEEDBACK_GYRO
            // pitch+陀螺仪反馈符号测试(用户指定形态): 反馈=Gyro[1], 输出取负, 目标用局部pitch_vel_ref(固定0阻尼, 可改值验方向)
            pitch_vel_ref = 0.0f; // 目标角速度固定0 rad/s(阻尼测试), 可改固定值验证方向
            DMMotorSetRef(dm_pitch_motor, -PIDCalculate(&dm_pitch_motor->speed_PID, gimba_IMU_data->Gyro[1], pitch_vel_ref)); // pitch轴Gyro[1]符号与yaw相反(抬头为负), 按用户指定带负号
#else
            DMMotorSetRef(dm_pitch_motor, -PIDCalculate(&dm_pitch_motor->speed_PID, dm_pitch_motor->measure.velocity, -g_pitch_vel_ref)); // pitch速度环: 只取负输出(实测才有负反馈), 目标取负(该轴读数方向与'抬头'约定相反, 负目标=抬头/正目标=低头), 反馈用原读数
#endif
        }
        else
        {
            dm_pitch_motor->speed_PID.Iout = 0.0f;
            dm_pitch_motor->speed_PID.ITerm = 0.0f;
            dm_pitch_motor->speed_PID.Output = 0.0f;
            DMMotorSetRef(dm_pitch_motor, 0.0f); // 非[下]: 零力矩
        }
        break;
    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

#if GIMBAL_FEEDBACK_GYRO
    g_yaw_vel_feedback = gimba_IMU_data->Gyro[2]; // 陀螺仪反馈观测
    g_pitch_vel_feedback = gimba_IMU_data->Gyro[1]; // 陀螺仪反馈观测
#else
    g_yaw_vel_feedback = dm_yaw_motor->measure.velocity; // 更新反馈速度全局变量
    g_pitch_vel_feedback = -dm_pitch_motor->measure.velocity; // 更新pitch反馈速度全局变量
#endif

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    // gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round; // yaw已移底盘板,注释保留

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}