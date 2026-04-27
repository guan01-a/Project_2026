#include "MotorControl.h"
#include <math.h>   // 用于 fabsf

// 三个电机变量（保持原样）
MOTOR_send cmd_1 = {
    cmd_1.id   = 0,
    cmd_1.mode = 1,
    cmd_1.T    = 0.0f,
    cmd_1.W    = 0.0f,
    cmd_1.Pos  = 3.46f,
    cmd_1.K_P  = 1.0f,
    cmd_1.K_W  = 0.04f
};
MOTOR_send cmd_2 = {
    cmd_2.id   = 1,
    cmd_2.mode = 1,
    cmd_2.T    = 0.0f,
    cmd_2.W    = 0.0f,
    cmd_2.Pos  = 5.45f,
    cmd_2.K_P  = 1.0f,
    cmd_2.K_W  = 0.04f
};
MOTOR_send cmd_3 = {
    cmd_3.id   = 2,
    cmd_3.mode = 1,
    cmd_3.T    = 0.0f,
    cmd_3.W    = 0.0f,
    cmd_3.Pos  = 4.54f,
    cmd_3.K_P  = 1.0f,
    cmd_3.K_W  = 0.04f
};
MOTOR_send cmd_4 = {
    cmd_4.id   = 3,
    cmd_4.mode = 1,
    cmd_4.T    = 0.0f,
    cmd_4.W    = 0.0f,
    cmd_4.Pos  = 2.86f,
    cmd_4.K_P  = 1.5f,
    cmd_4.K_W  = 0.04f
};
MOTOR_send cmd_5 = {
    cmd_5.id   = 4,
    cmd_5.mode = 1,
    cmd_5.T    = 0.0f,
    cmd_5.W    = 0.0f,
    cmd_5.Pos  = -1.25f,
    cmd_5.K_P  = 1.0f,
    cmd_5.K_W  = 0.04f
};
MOTOR_recv data_1 = {0};
MOTOR_recv data_2 = {0};
MOTOR_recv data_3 = {0};
MOTOR_recv data_4 = {0};
MOTOR_recv data_5 = {0};
static uint32_t last_check_time = 0;
static uint8_t  motor_state = 0;
static int32_t  state_start_time = 0;
static int32_t  state_start_time_2 = 0;
static int32_t  state_start_time_3 = 0;
static int32_t  state_start_time_4 = 0;
static uint8_t  last_pin_state = 0xFF;
uint8_t count = 0;
uint8_t motor_power_level = 2;  // 默认中等力度

// 位置误差容限（弧度或工程单位）
#define POS_TOLERANCE   4.00f
// 第四电机的固定位置
#define MOTOR4_HOME_POS  2.86f
void MotorControl_Init(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    last_pin_state = HAL_GPIO_ReadPin(E18_PORT, E18_PIN);
    motor_state = 0;
    state_start_time = 0;
}

void MotorControl_Update(void)
{
    // 读取当前引脚状态，上升沿触发（空闲时）
    uint8_t current_pin_state = HAL_GPIO_ReadPin(E18_PORT, E18_PIN);
    if (last_pin_state == GPIO_PIN_RESET && current_pin_state == GPIO_PIN_SET)
    {
        if (motor_state == 0)
        {
            motor_state = 1;
            state_start_time = 0;
        }
    }
    last_pin_state = current_pin_state;

    // 状态机
    switch (motor_state)
    {
        case 0:   // 空闲，等待触发
            cmd_1.Pos = 3.46f;
            cmd_2.Pos = 5.45f;
            cmd_3.Pos = 4.54f;
            cmd_4.Pos = MOTOR4_HOME_POS;
				    cmd_5.Pos  = -1.25f;
            // 空闲时保持位置（KP非零）
            cmd_1.K_P = 1.0f;
            cmd_2.K_P = 1.0f;
            cmd_3.K_P = 1.0f;
            cmd_4.K_P = 1.5f;
				    cmd_5.K_P  = 1.0f;
            break;

        case 1:   // 三个电机同时击打（运动到击打位置）
            cmd_1.Pos = -2.04f;
            cmd_2.Pos = -0.05f;
            cmd_3.Pos = -0.96f;
				    
            // 根据力度等级选择KP
            switch (motor_power_level) {
                case 1:  // 小力度
                    cmd_1.K_P = 0.4f; cmd_2.K_P = 0.4f; cmd_3.K_P = 0.4f;
                    break;
                case 2:  // 中力度
                    cmd_1.K_P = 1.0f; cmd_2.K_P = 1.0f; cmd_3.K_P = 1.0f;
                    break;
                case 3:  // 大力度
                    cmd_1.K_P = 0.8f; cmd_2.K_P = 0.8f; cmd_3.K_P = 0.8f;
                    break;
                default:
                    cmd_1.K_P = 0.6f; cmd_2.K_P = 0.6f; cmd_3.K_P = 0.6f;
                    break;
            }
            cmd_4.K_P = 1.5f; 
            cmd_4.Pos = MOTOR4_HOME_POS;
					cmd_5.Pos  = -1.25f;
				    cmd_5.K_P  = 1.0f; // 第4,5电机保持归位位置不变

            state_start_time++;
						
            if (state_start_time >= MOVE_TIME_MS)   // 击打动作持续时间
            {
                motor_state = 2;
                state_start_time = 0;
            }
            break;

        case 2:   // 电机1、2、3归位 + 延时100ms
    cmd_1.Pos = 3.46f;
    cmd_2.Pos = 5.45f;
    cmd_3.Pos = 4.54f;
    cmd_1.K_P = 0.5f;
    cmd_2.K_P = 0.5f;
    cmd_3.K_P = 0.5f;
    cmd_4.Pos = MOTOR4_HOME_POS;
    cmd_4.K_P = 1.5f;
    cmd_5.Pos = -1.25f;        // 延时阶段保持归位
    cmd_5.K_P = 1.0f;

    state_start_time_2++;
    if (state_start_time_2 >= 350)   // 延时100ms结束
    {
        motor_state = 21;            // 转入击打阶段（新状态）
        state_start_time_2 = 0;
    }
    break;

case 21:  // 电机5击打阶段
    // 电机1、2、3继续保持归位（位置不变）
    cmd_1.Pos = 3.46f;
    cmd_2.Pos = 5.45f;
    cmd_3.Pos = 4.54f;
    cmd_1.K_P = 0.5f;
    cmd_2.K_P = 0.5f;
    cmd_3.K_P = 0.5f;
    cmd_4.Pos = MOTOR4_HOME_POS;
    cmd_4.K_P = 1.5f;
    cmd_5.Pos = 5.51f;          // 击打位置
    cmd_5.K_P = 1.2f;

    state_start_time_3++;
    if (state_start_time_3 >= 300)   // 击打持续时间
    {
        motor_state = 3;        // 转入归位状态
        state_start_time_3 = 0;
    }
    break;

        case 3:   // 第五电机归位
            cmd_1.Pos = 3.46f;
            cmd_2.Pos = 5.45f;
            cmd_3.Pos = 4.54f;
            cmd_1.K_P = 0.5f;
            cmd_2.K_P = 0.5f;
            cmd_3.K_P = 0.5f;
            cmd_4.Pos = MOTOR4_HOME_POS;
            cmd_4.K_P = 1.5f;
				    cmd_5.Pos = -1.25f;
            cmd_5.K_P = 0.5f;
				
				    state_start_time_4++;
            if (state_start_time_4 >= MOVE_TIME_MS_4)   // 击打动作持续时间
            {
                motor_state = 0;
                state_start_time_4 = 0;
            }
            break;

        default:
            motor_state = 0;
            state_start_time_2 = 0;
				    state_start_time_3 = 0;
				    state_start_time_4 = 0;
            break;
    }

    // 轮询发送电机指令（保持原有逻辑，注意修正缩进问题）
    static uint8_t poll_step = 0;
    switch (poll_step)
    {
        case 0:
            if (SERVO_Send_recv_Motor1(&cmd_1, &data_1) == HAL_OK)
                SERVO_Send_recv_Motor3(&cmd_5, &data_5);
            poll_step = 1;
            break;

        case 1:
            if (SERVO_Send_recv_Motor1(&cmd_2, &data_2) == HAL_OK)
                SERVO_Send_recv_Motor3(&cmd_5, &data_5);
            poll_step = 2;
            break;

        case 2:
            if (SERVO_Send_recv_Motor1(&cmd_3, &data_3) == HAL_OK)
                SERVO_Send_recv_Motor3(&cmd_5, &data_5);
            poll_step = 3;
            break;
				case 3:
            if (SERVO_Send_recv_Motor1(&cmd_4, &data_4) == HAL_OK)
                SERVO_Send_recv_Motor3(&cmd_5, &data_5);
            poll_step = 0;
            break;
        default:
            poll_step = 0;
            break;
    }
}
