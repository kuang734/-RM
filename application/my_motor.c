#include "my_motor.h"

DJIMotorInstance *my_motor;

void MyMotorInit(void)
{
    Motor_Init_Config_s config = {
        .can_init_config = { .can_handle = &hcan1, .tx_id = 1 },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 4.5, .Ki = 0.0, .Kd = 0.0,
                .MaxOut = 15000, .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral
                         | PID_Integral_Limit
                         | PID_Derivative_On_Measurement,
            },
            .current_PID = {
                .Kp = 1.0, .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };
    my_motor = DJIMotorInit(&config);
}

void MyMotorControl(void)
{
    DJIMotorSetRef(my_motor, 1000.0f);
}
