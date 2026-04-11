/**
 * @file    sbus.h
 * @brief   SBUS协议解析头文件 - RoboMaster C板版本
 */

#ifndef __SBUS_H
#define __SBUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"

/* Exported types ------------------------------------------------------------*/
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_FRAME_SIZE 25
#define SBUS_START_BYTE 0x0F
#define SBUS_END_BYTE 0x00

/* SBUS数据结构 */
typedef struct {
    uint16_t channels[SBUS_CHANNEL_NUMBER];  // 16个通道值 (172-1811)
    uint8_t failsafe;                        // 故障保护标志
    uint8_t frame_lost;                      // 帧丢失标志
    uint8_t valid;                           // 数据有效标志
} SBUS_Data_t;

/* Exported variables --------------------------------------------------------*/
extern uint8_t sbus_rx_buffer[];
extern uint32_t sbus_last_frame_time;

/* Exported functions prototypes ---------------------------------------------*/
void SBUS_Init(void);
void SBUS_FeedChunk(uint8_t *buf, uint16_t len);
void SBUS_Process(void);
uint16_t SBUS_GetChannel(uint8_t channel);
uint8_t SBUS_IsValid(void);

#ifdef __cplusplus
}
#endif

#endif /* __SBUS_H */
