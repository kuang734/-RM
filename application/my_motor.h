#ifndef MY_MOTOR_H
#define MY_MOTOR_H
#include "dji_motor.h"

extern DJIMotorInstance *my_motor;

void MyMotorInit(void);
void MyMotorControl(void);

#endif
