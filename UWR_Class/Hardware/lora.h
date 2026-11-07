#ifndef __LORA_H
#define __LORA_H

#include "stm32f10x.h"
#include <stdio.h>

/**
 * @brief 水下机器人课设 - LoRa 无线通信模块驱动头文件
 * 硬件连接 (参考 水下课设原理图&BOM.pdf 中的 H8 排母插座):
 * -------------------------------------------------------------
 * LoRa H8 Pin 1: NC (悬空)
 * LoRa H8 Pin 2: NC (悬空)
 * LoRa H8 Pin 3: PA2 (STM32 USART2_TX)
 * LoRa H8 Pin 4: PA3 (STM32 USART2_RX)
 * LoRa H8 Pin 5: NC (悬空)
 * LoRa H8 Pin 6: +5V 电源
 * LoRa H8 Pin 7: GND 地
 * -------------------------------------------------------------
 */

#define LORA_DEFAULT_BAUDRATE       9600

#define LORA_USART                  USART2
#define LORA_USART_CLK              RCC_APB1Periph_USART2
#define LORA_GPIO_CLK               RCC_APB2Periph_GPIOA
#define LORA_GPIO_PORT              GPIOA
#define LORA_TX_PIN                 GPIO_Pin_2
#define LORA_RX_PIN                 GPIO_Pin_3
#define LORA_IRQn                   USART2_IRQn
#define LORA_IRQHandler             USART2_IRQHandler

/* 接收环形缓冲区大小 */
#define LORA_RX_BUFFER_SIZE         128

/**
 * @brief 遥控通信数据包协议定义 (用于遥控器控制水下两路电机)
 * 帧格式: [0x55] [0xAA] [CMD] [SpeedLeft_H] [SpeedLeft_L] [SpeedRight_H] [SpeedRight_L] [Checksum] [0x0D] [0x0A]
 * 长度: 10字节
 */
#define LORA_FRAME_HEAD1            0x55
#define LORA_FRAME_HEAD2            0xAA
#define LORA_FRAME_TAIL1            0x0D
#define LORA_FRAME_TAIL2            0x0A

/* 控制命令字定义 */
#define LORA_CMD_STOP               0x00    // 停止/待机
#define LORA_CMD_DIRECT_PWM         0x01    // 直接给定左右电机转速 (-1000 ~ +1000)
#define LORA_CMD_FORWARD            0x02    // 前进
#define LORA_CMD_BACKWARD           0x03    // 后退
#define LORA_CMD_TURN_LEFT          0x04    // 左转
#define LORA_CMD_TURN_RIGHT         0x05    // 右转
#define LORA_CMD_BRAKE              0x06    // 紧急刹车

/* 遥控数据包结构体 */
typedef struct {
    uint8_t cmd;                // 命令字
    int16_t left_speed;         // 左电机转速 (-1000 ~ +1000)
    int16_t right_speed;        // 右电机转速 (-1000 ~ +1000)
} LoRa_Packet_t;

/* 函数声明 */
void LoRa_Init(uint32_t baudrate);
void LoRa_SendByte(uint8_t byte);
void LoRa_SendArray(const uint8_t *array, uint16_t len);
void LoRa_SendString(const char *str);

/* 基础接收接口 (非阻塞轮询 / 查询) */
uint16_t LoRa_Available(void);
uint8_t LoRa_Read(void);
uint16_t LoRa_ReadBytes(uint8_t *buffer, uint16_t max_len);
void LoRa_ClearRxBuffer(void);

/* 指令级解析接口 (适合字符控制与协议包控制) */
uint8_t LoRa_GetChar(char *ch);
uint8_t LoRa_GetPacket(LoRa_Packet_t *packet);
uint8_t LoRa_GetCommand(LoRa_Packet_t *packet);

#endif /* __LORA_H */
