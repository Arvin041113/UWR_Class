#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

/**
 * @brief 水下机器人课设 - TB6612FNG 双路推进器电机驱动头文件
 * 硬件连接 (参考 水下课设原理图&BOM.pdf 中的 U3 TB6612 芯片):
 * -------------------------------------------------------------
 * 左电机 (Motor A, 接口 U5):
 *   PWMA -> PA8  (STM32 TIM1_CH1 PWM调速, 占空比 0~1000)
 *   AIN1 -> PB12 (方向引脚 1)
 *   AIN2 -> PB13 (方向引脚 2)
 *
 * 右电机 (Motor B, 接口 U6):
 *   PWMB -> PA9  (STM32 TIM1_CH2 PWM调速, 占空比 0~1000)
 *   BIN1 -> PB14 (方向引脚 1)
 *   BIN2 -> PB15 (方向引脚 2)
 *
 * 芯片使能:
 *   STBY -> +5V  (硬件已固定接高电平使能)
 * -------------------------------------------------------------
 */

#define MOTOR_SPEED_MAX             1000
#define MOTOR_SPEED_MIN             (-1000)

/* 方向控制 GPIO 引脚定义 (GPIOB) */
#define MOTOR_DIR_GPIO_CLK          RCC_APB2Periph_GPIOB
#define MOTOR_DIR_PORT              GPIOB
#define MOTOR_AIN1_PIN              GPIO_Pin_12
#define MOTOR_AIN2_PIN              GPIO_Pin_13
#define MOTOR_BIN1_PIN              GPIO_Pin_14
#define MOTOR_BIN2_PIN              GPIO_Pin_15

/* PWM 控制 GPIO 引脚定义 (GPIOA, TIM1) */
#define MOTOR_PWM_GPIO_CLK          RCC_APB2Periph_GPIOA
#define MOTOR_PWM_PORT              GPIOA
#define MOTOR_PWMA_PIN              GPIO_Pin_8
#define MOTOR_PWMB_PIN              GPIO_Pin_9

#define MOTOR_TIM                   TIM1
#define MOTOR_TIM_CLK               RCC_APB2Periph_TIM1

/* 初始化函数 */
void Motor_Init(void);

/* 电机速度控制接口 (speed 取值范围: -1000 ~ +1000) */
void Motor_SetSpeedA(int16_t speed);
void Motor_SetSpeedB(int16_t speed);
void Motor_SetSpeed(int16_t speedA, int16_t speedB);

/* 停止与刹车 */
void Motor_Stop(void);
void Motor_Brake(void);

/* 水下机器人常用差速动作控制 (0 ~ 1000) */
void Motor_Forward(int16_t speed);
void Motor_Backward(int16_t speed);
void Motor_TurnLeft(int16_t speed);
void Motor_TurnRight(int16_t speed);

#endif /* __MOTOR_H */
