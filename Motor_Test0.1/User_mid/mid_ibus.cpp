#include "mid_ibus.h"

/** @brief 初始化IBUS状态机
 * @param huart UART句柄指针
 * @return void 
 * @note 初始化IBUS状态机, 清除所有数据
 */
void Class_IBUS::Init(UART_HandleTypeDef * huart)
{
    this->huart = huart;
    this->state = IBUS_STATE_SYNC0;
    this->buffer_index = 0;
    this->flags.fail_safe = 0;
    this->flags.sync = 0;

    memset(this->rxbuffer ,0 ,sizeof(this->rxbuffer));
    memset(this->ibus_data.channels , 0 ,sizeof(this->ibus_data.channels));
}
/**
 * @brief 数据处理过程
 * @param data 接收数据指针
 * @param length 数据长度
 * @return void 
 * @note 处理IBUS数据, 包括同步帧, 数据帧, 校验帧
 */
void Class_IBUS::Process_Data(uint8_t * data , uint16_t length)
{
    for (uint16_t i = 0; i < length ; ++i)
    {
        uint8_t byte = data[i];
        switch(this->state)
     {
        case IBUS_STATE_SYNC0 : 
        {
            if(byte == IBUS_HEADER_1)//等待0X20
            {
                this->rxbuffer[0] = byte;
                this->buffer_index = 1;
                this->state = IBUS_STATE_SYNC1;
            }
        }break;

        case IBUS_STATE_SYNC1 :
        {
            if(byte == IBUS_HEADER_2)//等待0X40
            {
                this->rxbuffer[1] = byte;
                this->buffer_index = 2;
                this->state = IBUS_STATE_DATA;
            }
            else{

            }
        }break;

        case IBUS_STATE_DATA:
        {
            //接收数据
            this->rxbuffer[this->buffer_index++]=byte;
            if(this->buffer_index >= IBUS_PACKET_SIZE - 2)
            {
                this->state = IBUS_STATE_CHECKSUM;//接收完成，进行CRC校验
            }
        }break;

        case IBUS_STATE_CHECKSUM:
        {
            /*接收校验位*/
            this->rxbuffer[this->buffer_index++] = byte;
            if(this->buffer_index>=IBUS_PACKET_SIZE)
            {
                uint16_t caculate_checksum = this->Caculate_Check(this->rxbuffer , IBUS_PACKET_SIZE-2);//计算校验和
                uint16_t received_checksum = (this->rxbuffer[IBUS_PACKET_SIZE - 1] << 8) 
                | this->rxbuffer[IBUS_PACKET_SIZE - 2];//接收校验和

                if(caculate_checksum == received_checksum)
                {
                    /*校验正确，开始解析数据包*/
                    Parse_Packet(this->rxbuffer);
                    this->flags.sync = 1;//同步标志位设置
                }
                else{
                    this->flags.sync = 0;//同步标志位清除
                }

            }
            /*校验完成，重置状态机*/
            this->state = IBUS_STATE_SYNC0;
            this->buffer_index = 0;
        }break;

        default :
        {
            this->state = IBUS_STATE_SYNC0;
            this->buffer_index = 0;
        }break;
     }
    }
}
/** @brief 计算校验和
 * @param data 数据指针
 * @param length 数据长度
 * @return uint16_t 校验和
 * @param length 数据长度
 * @note 计算校验和, 校验和为0xFFFF - 数据
 */
uint16_t Class_IBUS::Caculate_Check(uint8_t * data ,uint8_t length)
{
    /*计算校验和*/
    uint16_t checksum=0xFFF;
    for(uint8_t i = 0; i<length; i++)
    {
        checksum -= data[i];
    }
    return checksum;
}
/** @brief 解析数据包
 * @param data 数据指针
 * @param void 
 * @note 解析数据包, 拼接数据到ibus_data.channels
 */
void Class_IBUS::  Parse_Packet(uint8_t * data)
{
    /*for (uint8_t i = 0; i < IBUS_CHANNEL_COUNT; i++) {
        this->ibus_data.channels[i] = (data[2 + i * 2] | (data[3 + i * 2] << 8));
    } */

    /*将数据包中的数据拼接到ibus_data.channels，通道数据12位：8+4*/
    this->ibus_data.channels[0] = (data[2] | ((data[3] & 0x0F) << 8));       // CH1
    this->ibus_data.channels[1] = (data[4] | ((data[5] & 0x0F) << 8));       // CH2
    this->ibus_data.channels[2] = (data[6] | ((data[7] & 0x0F) << 8));       // CH3
    this->ibus_data.channels[3] = (data[8] | ((data[9] & 0x0F) << 8));       // CH4
    this->ibus_data.channels[4] = (data[10] | ((data[11] & 0x0F) << 8));     // CH5
    this->ibus_data.channels[5] = (data[12] | ((data[13] & 0x0F) << 8));     // CH6
    this->ibus_data.channels[6] = (data[14] | ((data[15] & 0x0F) << 8));     // CH7
    this->ibus_data.channels[7] = (data[16] | ((data[17] & 0x0F) << 8));     // CH8
    this->ibus_data.channels[8] = (data[18] | ((data[19] & 0x0F) << 8));     // CH9
    this->ibus_data.channels[9] = (data[20] | ((data[21] & 0x0F) << 8));     // CH10
    this->ibus_data.channels[10] = (data[22] | ((data[23] & 0x0F) << 8));    // CH11
    this->ibus_data.channels[11] = (data[24] | ((data[25] & 0x0F) << 8));    // CH12
    this->ibus_data.channels[12] = (data[26] | ((data[27] & 0x0F) << 8));    // CH13
    this->ibus_data.channels[13] = (data[28] | ((data[29] & 0x0F) << 8));    // CH14

    this->flags.fail_safe = (this->ibus_data.channels[10] > IBUS_FAILSAFE_THRESHOLD);
}

/** @brief 获取单个通道数据
 * @param channel 通道号
 * @return uint16_t 通道数据    
 */
uint16_t Class_IBUS::Get_Channel(uint8_t channel) {
    if (channel < IBUS_CHANNEL_COUNT) {
        return this->ibus_data.channels[channel];
    }
    return 0;
}
/** @brief 获取所有通道数据
 * @return Struct_IBUS_Data 所有通道数据
 */
Struct_IBUS_Data Class_IBUS::Get_AllChannels() {
    return this->ibus_data;
}
/** @brief 获取是否失控状态
 * @return bool 是否失控
 */
bool Class_IBUS::Get_FailsafeStatus() {
    return this->flags.fail_safe;
}
/** @brief 重置IBUS状态机
 * @param void 
 * @note 重置IBUS状态机, 清除所有数据
 */
void Class_IBUS::Reset() {
    this->state = IBUS_STATE_SYNC0;
    this->buffer_index = 0;
    this->flags.fail_safe = 0;
    this->flags.sync = 0;
    memset(this->rxbuffer ,0 ,sizeof(this->rxbuffer));
    memset(this->ibus_data.channels , 0 ,sizeof(this->ibus_data.channels));
}