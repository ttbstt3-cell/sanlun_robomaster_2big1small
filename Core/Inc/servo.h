/**
 * @file    servo.h
 * @brief   舵机控制头文件 - RoboMaster C板版本
 */

#ifndef __SERVO_H
#define __SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* Exported constants --------------------------------------------------------*/
// PWM周期为20000 (20ms, 50Hz)
// 注意：TIM1实际时钟为2x，所以所有脉宽值需要x2
#define SERVO_PWM_PERIOD 40000  // 原20000 * 2

// 180度舵机脉宽范围：500-2500us (对应0.5ms-2.5ms)
#define SERVO_180_MIN_PULSE 500    // 500us
#define SERVO_180_MAX_PULSE 2500    // 2500us
#define SERVO_180_MID_PULSE 1500   // 1500us

// 360度连续旋转舵机脉宽范围：500-2500us
// 1500 = 停止, <1500 = 反转, >1500 = 正转
#define SERVO_360_STOP_PULSE 1500  // 停止
#define SERVO_360_MIN_PULSE 500    // 最大反转速度
#define SERVO_360_MAX_PULSE 2500   // 最大正转速度

/* Exported functions prototypes ---------------------------------------------*/
void Servo_Init(void);
void Servo_Set180(uint8_t servo_num, uint16_t pulse_us);
void Servo_Set360(uint8_t servo_num, int16_t speed);  // speed: -1000到+1000
void Servo_SetPulse(uint8_t channel, uint16_t pulse_us);  // 直接设置PWM脉宽
void Servo_Stop360(uint8_t servo_num);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */
