/**
 * @file    servo.c
 * @brief   舵机控制实现文件 - RoboMaster C板版本
 */

/* Includes ------------------------------------------------------------------*/
#include "servo.h"
#include "tim.h"

/* Private variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;

/* Exported functions --------------------------------------------------------*/
void Servo_SetPulse(uint8_t channel, uint16_t pulse_us);

#include "stm32f4xx_hal.h"

/**
 * @brief  舵机初始化
 * @retval None
 */
void Servo_Init(void)
{
    // ！！！究极物理修复：强行把 PWM 信号拽回 C 板真实的舵机引脚 (PE9/11/13/14) ！！！
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1; // 强制映射给 TIM1
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    // 启用TIM1主输出（高级定时器需要此设置）
    __HAL_TIM_MOE_ENABLE(&htim1);
    
    // 启动所有PWM通道 (TIM1, 对应C板的 C4, C3, C2, C1 接口)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  
    
    // 强制发送 1500 的标准锁死中位！
    Servo_SetPulse(TIM_CHANNEL_1, 1500);  
    Servo_SetPulse(TIM_CHANNEL_2, 1500);  
    Servo_SetPulse(TIM_CHANNEL_3, 1500);  
    Servo_SetPulse(TIM_CHANNEL_4, 1500);  
}

/**
 * @brief  设置PWM脉宽（内部函数）
 * @param  channel: 定时器通道
 * @param  pulse_us: 脉宽（微秒）
 * @retval None
 */
void Servo_SetPulse(uint8_t channel, uint16_t pulse_us)
{
    uint32_t pulse_ticks;
    
    // 将微秒转换为定时器计数值
    // APB2时钟 = 84MHz, APB2分频 = 2, 所以TIM1时钟 = 84MHz * 2 = 168MHz
    // Prescaler = 167, 所以定时器频率 = 168MHz / (167 + 1) = 1MHz
    // 因此 1us = 1个计数
    pulse_ticks = pulse_us;
    
    // 限制脉宽范围
    if (pulse_ticks > SERVO_PWM_PERIOD)
    {
        pulse_ticks = SERVO_PWM_PERIOD;
    }
    
    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(&htim1, channel, pulse_ticks);
}

/**
 * @brief  设置180度舵机角度
 * @param  servo_num: 舵机编号 (2)
 * @param  pulse_us: 脉宽（500-2500微秒）
 * @retval None
 */
void Servo_Set180(uint8_t servo_num, uint16_t pulse_us)
{
    uint8_t channel;
    
    // 限制脉宽范围
    if (pulse_us < SERVO_180_MIN_PULSE)
        pulse_us = SERVO_180_MIN_PULSE;
    if (pulse_us > SERVO_180_MAX_PULSE)
        pulse_us = SERVO_180_MAX_PULSE;
    
    // 根据舵机编号选择通道
    switch(servo_num)
    {
        case 2:
            channel = TIM_CHANNEL_2;
            break;
        default:
            return;  // 无效的舵机编号
    }
    
    Servo_SetPulse(channel, pulse_us);
}

/**
 * @brief  设置360度连续旋转舵机速度
 * @param  servo_num: 舵机编号 (1, 3, 4)
 * @param  speed: 速度值 (-1000到+1000, 0=停止)
 * @retval None
 */
void Servo_Set360(uint8_t servo_num, int16_t speed)
{
    uint8_t channel;
    uint16_t pulse_us;
    
    // 限制速度范围
    if (speed < -1000) speed = -1000;
    if (speed > 1000) speed = 1000;
    
    // 根据舵机编号选择通道
    switch(servo_num)
    {
        case 1:
            channel = TIM_CHANNEL_1;
            break;
        case 3:
            channel = TIM_CHANNEL_3;
            break;
        case 4:
            channel = TIM_CHANNEL_4;
            break;
        default:
            return;  // 无效的舵机编号
    }
    
    // 将速度值转换为脉宽
    // speed范围: -1000到+1000
    // 脉宽范围: 500到2500us
    // 1500us = 停止
    if (speed == 0)
    {
        pulse_us = SERVO_360_STOP_PULSE;
    }
    else if (speed > 0)
    {
        // 正转: 1500-2500us
        pulse_us = SERVO_360_STOP_PULSE + (speed * (SERVO_360_MAX_PULSE - SERVO_360_STOP_PULSE) / 1000);
    }
    else
    {
        // 反转: 500-1500us
        pulse_us = SERVO_360_STOP_PULSE + (speed * (SERVO_360_STOP_PULSE - SERVO_360_MIN_PULSE) / 1000);
    }
    
    Servo_SetPulse(channel, pulse_us);
}

/**
 * @brief  停止360度连续旋转舵机
 * @param  servo_num: 舵机编号 (1, 3, 4)
 * @retval None
 */
void Servo_Stop360(uint8_t servo_num)
{
    Servo_Set360(servo_num, 0);
}
