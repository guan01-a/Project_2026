#include "MotorControl.h"
MOTOR_send cmd_1 = {
    cmd_1.id   = 0,
    cmd_1.mode = 1,
    cmd_1.T    = 0.0f,
    cmd_1.W    = 0.0f,
    cmd_1.Pos  = 3.46f,
    cmd_1.K_P  = 1.0f,
    cmd_1.K_W  = 0.04f
};
MOTOR_recv data_1 = {0};

void MotorControl_Init(void)
{
   
}

void MotorControl_Update(void)
{
    cmd_1.Pos = 3.46f;
    cmd_1.K_P = 1.0f;
    SERVO_Send_recv_Motor1(&cmd_1, &data_1);
}
