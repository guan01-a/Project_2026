#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "motor_control.h"

#define E18_PIN   GPIO_PIN_0
#define E18_PORT  GPIOA         //定义E18引脚所在的GPIO端口和引脚号

#define MOVE_TIME_MS  300     // 定义每个状态持续的时间，单位为毫秒
#define MOVE_TIME_MS_2  100 
#define MOVE_TIME_MS_3  300
#define MOVE_TIME_MS_4  300

extern MOTOR_recv data_1;
extern MOTOR_recv data_2;
extern MOTOR_recv data_3;
extern MOTOR_recv data_4;
extern MOTOR_recv data_5;
extern MOTOR_recv motor1_recv_data; // 对应回调函数里用的变量
extern uint8_t motor_power_level;

void MotorControl_Init(void);

void MotorControl_Update(void);

#endif
