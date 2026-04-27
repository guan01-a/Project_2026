#ifndef MID_IBUS_H
#define MID_IBUS_H

#include "stm32h7xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"

#include "alg_fsm.h"

//IBUS头帧
#define IBUS_HEADER_1         0x20
#define IBUS_HEADER_2         0x40

//校验常量
#define TELM_CHECKSUM_CONST    0xFFFF


#define IBUS_PACKET_SIZE        32     //IBUS数据包大小
#define IBUS_CHANNEL_COUNT      14      //IBUS通道数量
#define IBUS_FAILSAFE_THRESHOLD 1975    //失控阈值

class Class_IBUS;
/*IBUS状态机*/
typedef enum
{
    IBUS_STATE_SYNC0,       
    IBUS_STATE_SYNC1,       
    IBUS_STATE_DATA,       
    IBUS_STATE_CHECKSUM      
} Enum_IBUS_State;

/*IBUS通道*/
typedef struct
{
    uint16_t channels[IBUS_CHANNEL_COUNT];
}Struct_IBUS_Data;

/*IBUS状态标志*/
typedef struct{
    uint8_t fail_safe : 1;
    uint8_t sync : 1;
}Struct_IBUS_Flags;


class Class_FSM_IBUS : public Class_FSM
{
    Class_IBUS *ibus;
    Enum_IBUS_State IBUS_State =  IBUS_STATE_SYNC0;
};

class Class_IBUS
{
    public:
    Class_IBUS()
    {
        Reset();
    }

    friend class Class_FSM_IBUS;

    void Init (UART_HandleTypeDef * huart);
    
    void Process_Data(uint8_t *data, uint16_t length);
    
    uint16_t Get_Channel(uint8_t channel);
    
    Struct_IBUS_Data Get_AllChannels();
    
    bool Get_FailsafeStatus();
    
    void Reset();

    private:

    UART_HandleTypeDef * huart;

    Enum_IBUS_State state;//IBUS状态

    uint8_t rxbuffer [IBUS_PACKET_SIZE];//接收数据缓冲区

    uint8_t buffer_index;//接收数据索引

    Struct_IBUS_Data ibus_data;//IBUS通道数据

    Struct_IBUS_Flags flags;//IBUS状态标志

    uint16_t Caculate_Check(uint8_t * data ,uint8_t length);//校验和
    void Parse_Packet(uint8_t *data);  //解析数据包
};

#endif