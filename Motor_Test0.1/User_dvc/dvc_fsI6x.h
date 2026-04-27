//右摇杆左右ch1 上下ch2
//左摇杆上下 ch3
//左摇杆左右 ch4
//5-10自己设定
//从左到右 5-10分别是：SA SB VA VB SC SD

#ifndef __DVC_FSI6X_H
#define __DVC_FSI6X_H

#include <limits.h>
#include <string.h>

#include "drv_uart.h"
#include "mid_ibus.h"

#ifdef	__cplusplus 

//拨动开关位置
#define SWITCH_UP (1)
#define SWITCH_DOWN (2)
#define SWITCH_MIDDLE (3)

/*SA*/
#define FSI6X_SA_DOWN (0)
#define FSI6X_SA_UP (1)

/*SB*/
#define FSI6X_SB_DOWN (0)
#define FSI6X_SB_UP (1)

/*SC :三个状态*/
#define FSI6X_SC_DOWN (0)
#define FSI6X_SC_MIDDLE (1)
#define FSI6X_SC_UP (2)

/*SD*/
#define FSI6X_SD_DOWN (0)
#define FSI6X_SD_UP (1)



/**
 * @brief 遥控器状态
 *
 */
enum Enum_FSI6X_Status
{
    FSI6X_Status_DISABLE = 0,
    FSI6X_Status_ENABLE,
};

/**
 * @brief IA10B存活状态
 *
 */
enum Enum_IA10B_Status
{
    IA10B_Status_DISABLE,
    IA10B_Status_ENABLE,
};


/**
 * @brief 遥控器数据更新状态
 *
 */
enum Enum_FSI6X_Updata_Status
{
    FSI6X_Status_DisUpdata = 0,
    FSI6X_Status_Updata,
};

/**
 * @brief 拨动开关状态
 *
 */
enum Enum_FSI6X_Switch_Status
{
    FSI6X_Switch_Status_UP = 0,           //上状态
    FSI6X_Switch_Status_TRIG_UP_MIDDLE,   //上到中的突变状态
    FSI6X_Switch_Status_TRIG_MIDDLE_UP,   //中到上的突变状态
    FSI6X_Switch_Status_MIDDLE,           //中状态
    FSI6X_Switch_Status_TRIG_MIDDLE_DOWN, //中到下的突变状态
    FSI6X_Switch_Status_TRIG_DOWN_MIDDLE, //下到中的突变状态
    FSI6X_Switch_Status_DOWN,             //下状态
};


struct Struct_FSI6X_UART_Data
{
    /*左右摇杆*/
    uint64_t Channel_1 : 16;
    uint64_t Channel_2 : 16;
    uint64_t Channel_3 : 16;
    uint64_t Channel_4 : 16;

    /*拨杆*/
    uint64_t SA_Switch : 2;
    uint64_t SB_Switch : 2;
    uint64_t SC_Switch : 2;
    uint64_t SD_Switch : 2;
} __attribute__((packed));


struct Struct_FSI6x_Data
{
    //摇杆
    float Right_X;
    float Right_Y;
    float Left_X;
    float Left_Y;

    //拨杆
    Enum_FSI6X_Switch_Status Switch_SA;
    Enum_FSI6X_Switch_Status Switch_SB;
    Enum_FSI6X_Switch_Status Switch_SC;
    Enum_FSI6X_Switch_Status Switch_SD;

    //旋钮


    float Yaw;
};

class Class_FSI6X
{
    public:
        void Init(UART_HandleTypeDef *huart_1);

        inline Enum_FSI6X_Status Get_FSI6X_Status();
        inline Enum_FSI6X_Updata_Status Get_FSI6X_Updata_Status();
   
        inline Enum_IA10B_Status Get_IA10B_Status() {return (IA10B_Status);}

        inline uint16_t Get_Circle_Index(uint16_t index)
        {
            return (index % UART_Manage_Object_2->Rx_Buffer_Length);
        }

        inline float Get_Right_X();
        inline float Get_Right_Y();
        inline float Get_Left_X();
        inline float Get_Left_Y();

        inline Enum_FSI6X_Switch_Status Get_SA_Switch();
        inline Enum_FSI6X_Switch_Status Get_SB_Switch();
        inline Enum_FSI6X_Switch_Status Get_SC_Switch();
        inline Enum_FSI6X_Switch_Status Get_SD_Switch();
       
        inline float Get_Yaw();

        void FSI6X_UART_RxCpltCallback(uint8_t *Rx_Data);
    
        void TIM1msMod50_Alive_PeriodElapsedCallback();

    
    protected:
        //初始化相关常量

        //绑定的UART
        Struct_UART_Manage_Object *UART_Manage_Object_1;
        Struct_UART_Manage_Object *UART_Manage_Object_2;

        Class_IBUS ibus_parser;

        //摇杆偏移量 未测
        float Rocker_Offset = 1000.0f;
        //摇杆总刻度
        float Rocker_Num = 500.0f;

        //内部变量
        //现在时刻的遥控器状态信息
        Struct_FSI6X_UART_Data Now_UART_Rx_Data;
        //前一时刻的遥控器状态信息
        Struct_FSI6X_UART_Data Pre_UART_Rx_Data;


        //当前时刻的遥控器接收flag
        uint32_t FSI6X_Flag = 0;
        //前一时刻的遥控器接收flag
        uint32_t Pre_FSI6X_Flag = 0;

        //遥控器50ms离线次数
        uint16_t Unline_Cnt = 0;
        //遥控器50ms串口错误次数
        uint16_t Error_Cnt = 0;
        //读变量

        //遥控器状态
        Enum_FSI6X_Status FSI6X_Status = FSI6X_Status_DISABLE;
        //遥控器数据更新状态
        Enum_FSI6X_Updata_Status FSI6X_Updata_Status = FSI6X_Status_DisUpdata;
   
        //IA10B存活状态
        Enum_IA10B_Status IA10B_Status = IA10B_Status_DISABLE;

        // FSI6X对外接口信息
        Struct_FSI6x_Data Data;

        //写变量

        //读写变量

        //内部函数

        void Judge_Switch(Enum_FSI6X_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status);
        void Judge_Updata(Struct_FSI6X_UART_Data Pre_UART_Rx_Data,Struct_FSI6X_UART_Data Now_UART_Rx_Data);
        void FSI6X_Get_Raw_IBUS_Data(uint8_t *data, uint16_t length);
        void FSI6X_Data_Process(uint8_t *data, uint16_t length);

};


/**
 * @brief 获取遥控器在线状态
 *
 * @return Enum_DR16_Status 遥控器在线状态
 */
Enum_FSI6X_Status Class_FSI6X::Get_FSI6X_Status()
{
    return (FSI6X_Status);
}

/**
 * @brief 获取遥控器数据更新状态
 *
 * @return Enum_DR16_Updata_Status 遥控器在线状态
 */
Enum_FSI6X_Updata_Status Class_FSI6X::Get_FSI6X_Updata_Status()
{
    return (FSI6X_Updata_Status);
}

/**
 * @brief 获取遥控器右侧x轴摇杆状态
 *
 * @return float 遥控器右侧x轴摇杆状态
 */
float Class_FSI6X::Get_Right_X()
{
    return (Data.Right_X);
}

/**
 * @brief 获取遥控器右侧y轴摇杆状态
 *
 * @return float 遥控器右侧y轴摇杆状态
 */
float Class_FSI6X::Get_Right_Y()
{
    return (Data.Right_Y);
}

/**
 * @brief 获取遥控器左侧x轴摇杆状态
 *
 * @return float 遥控器左侧x轴摇杆状态
 */
float Class_FSI6X::Get_Left_X()
{
    return (Data.Left_X);
}

/**
 * @brief 获取遥控器左侧y轴摇杆状态
 *
 * @return float 遥控器左侧y轴摇杆状态
 */
float Class_FSI6X::Get_Left_Y()
{
    return (Data.Left_Y);
}


Enum_FSI6X_Switch_Status Class_FSI6X::Get_SA_Switch()
{
    return (Data.Switch_SA);
}

Enum_FSI6X_Switch_Status Class_FSI6X::Get_SB_Switch()
{
    return (Data.Switch_SB);
}

Enum_FSI6X_Switch_Status Class_FSI6X::Get_SC_Switch()
{
    return (Data.Switch_SC);
}

Enum_FSI6X_Switch_Status Class_FSI6X::Get_SD_Switch()
{
    return (Data.Switch_SD);
}

float Class_FSI6X::Get_Yaw()
{
    return (Data.Yaw);
}
#endif
#endif
