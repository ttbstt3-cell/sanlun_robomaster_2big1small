#include "sbus.h"
#include "usart.h"
#include <string.h>

uint8_t rx_byte;        // 单字节接收兵
uint8_t sbus_buf[25];   // 真正的 25 字节容器
uint8_t sbus_idx = 0;   // 计数器

uint32_t sbus_last_frame_time = 0;
static SBUS_Data_t sbus_data = {0};

void SBUS_Init(void)
{
    sbus_last_frame_time = HAL_GetTick(); 
    memset(&sbus_data, 0, sizeof(SBUS_Data_t));
    sbus_idx = 0;
    
    // ！！！终极绝招：彻底抛弃复杂的 DMA，开启单字节中断接收 ！！！
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
}

void SBUS_ParseFrame(uint8_t *frame)
{
    sbus_data.channels[0]  = ((frame[1]    | frame[2]  << 8)                        & 0x07FF);
    sbus_data.channels[1]  = ((frame[2]>>3 | frame[3]  << 5)                        & 0x07FF);
    sbus_data.channels[2]  = ((frame[3]>>6 | frame[4]  << 2 | frame[5] << 10)       & 0x07FF);
    sbus_data.channels[3]  = ((frame[5]>>1 | frame[6]  << 7)                        & 0x07FF);
    sbus_data.channels[4]  = ((frame[6]>>4 | frame[7]  << 4)                        & 0x07FF);
    sbus_data.channels[5]  = ((frame[7]>>7 | frame[8]  << 1 | frame[9] << 9)        & 0x07FF);
    sbus_data.channels[6]  = ((frame[9]>>2 | frame[10] << 6)                        & 0x07FF);
    sbus_data.channels[7]  = ((frame[10]>>5| frame[11] << 3)                        & 0x07FF);
    sbus_data.channels[8]  = ((frame[12]   | frame[13] << 8)                        & 0x07FF);
    sbus_data.channels[9]  = ((frame[13]>>3| frame[14] << 5)                        & 0x07FF);
    sbus_data.channels[10] = ((frame[14]>>6| frame[15] << 2 | frame[16] << 10)      & 0x07FF);
    sbus_data.channels[11] = ((frame[16]>>1| frame[17] << 7)                        & 0x07FF);
    sbus_data.channels[12] = ((frame[17]>>4| frame[18] << 4)                        & 0x07FF);
    sbus_data.channels[13] = ((frame[18]>>7| frame[19] << 1 | frame[20] << 9)       & 0x07FF);
    sbus_data.channels[14] = ((frame[20]>>2| frame[21] << 6)                        & 0x07FF);
    sbus_data.channels[15] = ((frame[21]>>5| frame[22] << 3)                        & 0x07FF);
    
    // 提取失控标志位
    sbus_data.frame_lost = (frame[23] >> 2) & 0x01;
    sbus_data.failsafe = (frame[23] >> 3) & 0x01;
    
    // 只有标志位说没失控，这包数据才算有效！
    sbus_data.valid = !sbus_data.failsafe && !sbus_data.frame_lost;
    if(sbus_data.valid) {
        sbus_last_frame_time = HAL_GetTick(); // 喂狗
    }
}

// ====== 完美吸纳视频作者思路，重写底层回调 ======
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (sbus_idx == 0) {
            if (rx_byte == 0x0F) { // 强力狙击：必须是 0x0F 才能成为第0位
                sbus_buf[sbus_idx++] = rx_byte;
            }
        }
        else {
            sbus_buf[sbus_idx++] = rx_byte;
            
            if (sbus_idx == 25) { // 稳稳当当数到 25 个
                sbus_idx = 0;
                
                // ！！！视频作者思路的乐迪进化版：同时兼容 0x00, 0x04, 0x08 ！！！
                if (sbus_buf[0] == 0x0F && 
                   (sbus_buf[24] == 0x00 || sbus_buf[24] == 0x04 || sbus_buf[24] == 0x08))
                {
                    SBUS_ParseFrame(sbus_buf);
                }
            }
        }
        // 继续开启下一个单字节狙击
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1); // 报错也立刻重启
    }
}

void SBUS_Process(void)
{
    if (HAL_GetTick() - sbus_last_frame_time > 50)
    {
        sbus_data.valid = 0;
    }
}

uint16_t SBUS_GetChannel(uint8_t channel)
{
    if (channel >= 1 && channel <= 16) return sbus_data.channels[channel - 1];
    return 1024;
}

uint8_t SBUS_IsValid(void)
{
    return sbus_data.valid;
}
