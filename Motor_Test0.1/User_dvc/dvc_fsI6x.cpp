#include "dvc_fsI6x.h"
#include "tsk_config_and_callback.h"
#include "drv_math.h"

/**
 * @brief 遥控器FSI6X初始化
 *
 * @param huart 指定的UART
 */
void Class_FSI6X::Init(UART_HandleTypeDef *huart_1)
{
    //fsi6x串口
    if (huart_1->Instance == USART1)
    {
        UART_Manage_Object_1 = &UART1_Manage_Object;
    }
    else if (huart_1->Instance == USART2)
    {
        UART_Manage_Object_1 = &UART2_Manage_Object;
    }
    else if (huart_1->Instance == USART3)
    {
        UART_Manage_Object_1 = &UART3_Manage_Object;
    }
    else if (huart_1->Instance == UART4)
    {
        UART_Manage_Object_1 = &UART4_Manage_Object;
    }
    else if (huart_1->Instance == UART5)
    {
        UART_Manage_Object_1 = &UART5_Manage_Object;
    }
    else if (huart_1->Instance == USART6)
    {
        UART_Manage_Object_1 = &UART6_Manage_Object;
    }

    ibus_parser.Init(huart_1);
}

/**
 * @brief 判断拨动开关状态
 *
 */
void Class_FSI6X::Judge_Switch(Enum_FSI6X_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status)
{
     //带触发的判断
    switch (Pre_Status)
    {
    case (SWITCH_UP):
    {
        switch (Status)
        {
        case (SWITCH_UP):
        {
            *Switch = FSI6X_Switch_Status_UP;
        }
        break;
        case (SWITCH_DOWN):
        {
            *Switch = FSI6X_Switch_Status_TRIG_MIDDLE_DOWN;
        }
        break;
        case (SWITCH_MIDDLE):
        {
            *Switch = FSI6X_Switch_Status_TRIG_UP_MIDDLE;
        }
        break;
        }
    }
    break;
    case (SWITCH_DOWN):
    {
        switch (Status)
        {
        case (SWITCH_UP):
        {
            *Switch = FSI6X_Switch_Status_TRIG_MIDDLE_UP;
        }
        break;
        case (SWITCH_DOWN):
        {
            *Switch = FSI6X_Switch_Status_DOWN;
        }
        break;
        case (SWITCH_MIDDLE):
        {
            *Switch = FSI6X_Switch_Status_TRIG_DOWN_MIDDLE;
        }
        break;
        }
    }
    break;
    case (SWITCH_MIDDLE):
    {
        switch (Status)
        {
        case (SWITCH_UP):
        {
            *Switch = FSI6X_Switch_Status_TRIG_MIDDLE_UP;
        }
        break;
        case (SWITCH_DOWN):
        {
            *Switch = FSI6X_Switch_Status_TRIG_MIDDLE_DOWN;
        }
        break;
        case (SWITCH_MIDDLE):
        {
            *Switch = FSI6X_Switch_Status_MIDDLE;
        }
        break;
        }
    }
    break;
    }
   
}


/**
 * @brief 判断遥控器更新状态
 *
 */
void Class_FSI6X::Judge_Updata(Struct_FSI6X_UART_Data __Pre_UART_Rx_Data,Struct_FSI6X_UART_Data __Now_UART_Rx_Data)
{
    if(__Pre_UART_Rx_Data.Channel_1 == __Now_UART_Rx_Data.Channel_1 &&
       __Pre_UART_Rx_Data.Channel_2 == __Now_UART_Rx_Data.Channel_2 &&
       __Pre_UART_Rx_Data.Channel_3 == __Now_UART_Rx_Data.Channel_3 &&
       __Pre_UART_Rx_Data.Channel_4 == __Now_UART_Rx_Data.Channel_4 
      )
        {
            FSI6X_Updata_Status = FSI6X_Status_DisUpdata;
        }
    else
        {
            FSI6X_Updata_Status = FSI6X_Status_Updata;
        }
    
}


/**
 * @brief 数据处理过程
 *
 */
void Class_FSI6X::FSI6X_Data_Process(uint8_t *data, uint16_t length)
{
     ibus_parser.Process_Data(data, length);
    
    // 将IBUS通道数据映射到FSI6X数据结构
    if (ibus_parser.Get_FailsafeStatus() == false) {

        /*获取原始数据*/   
        Now_UART_Rx_Data.Channel_1 = ibus_parser.Get_Channel(0);   // 右摇杆左右
        Now_UART_Rx_Data.Channel_2 = ibus_parser.Get_Channel(1);   // 右摇杆上下
        Now_UART_Rx_Data.Channel_3 = ibus_parser.Get_Channel(2);   // 左摇杆上下
        Now_UART_Rx_Data.Channel_4 = ibus_parser.Get_Channel(3);   // 左摇杆左右
        
      
        //拨杆 
        Now_UART_Rx_Data.SA_Switch = (ibus_parser.Get_Channel(4) > 1500) ? 1 : 2;
        Now_UART_Rx_Data.SB_Switch = (ibus_parser.Get_Channel(5) > 1500) ? 1 : 2;
        Now_UART_Rx_Data.SC_Switch = (ibus_parser.Get_Channel(8) > 1500) ? 1 : ((ibus_parser.Get_Channel(6) > 1000) ? 3 : 2);
        Now_UART_Rx_Data.SD_Switch = (ibus_parser.Get_Channel(9) > 1500) ? 1 : 2;

        //旋钮

        /*数据处理，转换为对外数据 Rocker_Offset   Rocker_Num 未测*/
        Data.Right_X = ( Now_UART_Rx_Data.Channel_1 - Rocker_Offset) / Rocker_Num;
        Data.Right_Y = ( Now_UART_Rx_Data.Channel_2 - Rocker_Offset) / Rocker_Num;
        Data.Left_X = (Now_UART_Rx_Data.Channel_3 - Rocker_Offset) / Rocker_Num;
        Data.Left_Y = ( Now_UART_Rx_Data.Channel_4 - Rocker_Offset) / Rocker_Num;

        /*拨动开关，未完善*/
        Judge_Switch(&Data.Switch_SA, Now_UART_Rx_Data.SA_Switch, Pre_UART_Rx_Data.SA_Switch);
        Judge_Switch(&Data.Switch_SB, Now_UART_Rx_Data.SB_Switch, Pre_UART_Rx_Data.SB_Switch);
        Judge_Switch(&Data.Switch_SC, Now_UART_Rx_Data.SC_Switch, Pre_UART_Rx_Data.SC_Switch);
        Judge_Switch(&Data.Switch_SD, Now_UART_Rx_Data.SD_Switch, Pre_UART_Rx_Data.SD_Switch);

        FSI6X_Status = FSI6X_Status_ENABLE;
    } else {
        // 失控保护状态
        FSI6X_Status = FSI6X_Status_DISABLE;
    }
     Judge_Updata(Pre_UART_Rx_Data,Now_UART_Rx_Data);
   
}





/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 */
void Class_FSI6X::FSI6X_UART_RxCpltCallback(uint8_t *Rx_Data)
{
    //滑动窗口, 判断遥控器是否在线
    FSI6X_Flag += 1;

    FSI6X_Data_Process(Rx_Data ,UART_Manage_Object_1->Rx_Buffer_Length);
    memcpy(&Pre_UART_Rx_Data, &Now_UART_Rx_Data, sizeof(Struct_FSI6X_UART_Data));
   
}



/**
 * @brief TIM定时器中断定期检测遥控器是否存活
 *
 */
void Class_FSI6X::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过遥控器数据
    if (FSI6X_Flag == Pre_FSI6X_Flag)
    {
        //遥控器断开连接
        FSI6X_Status = FSI6X_Status_DISABLE;
        Unline_Cnt++;
    }
    else
    {
        //遥控器保持连接
        FSI6X_Status = FSI6X_Status_ENABLE;
    }

    //IA10B存活状态判断
    if(FSI6X_Flag == Pre_FSI6X_Flag)
        IA10B_Status = IA10B_Status_DISABLE;
    else
        IA10B_Status = IA10B_Status_ENABLE;

    Pre_FSI6X_Flag = FSI6X_Flag;
}
