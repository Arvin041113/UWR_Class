#include "stm32f10x.h"
#include "lora.h"
#include "motor.h"

/* 全局毫秒时间戳 (由 SysTick_Handler 每 1ms 递增) */
volatile uint32_t g_tick_ms = 0;

/* 指令超时时间阈值: 若超过 600ms 未收到任何有效运动指令，自动停机保护 */
#define CMD_TIMEOUT_MS          600

/**
 * @brief 板载状态指示灯初始化 (PC13, 低电平点亮)
 */
static void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_SetBits(GPIOC, GPIO_Pin_13); // 初始熄灭 (高电平灭)
}

/**
 * @brief 设置板载LED状态 (1: 点亮, 0: 熄灭)
 */
static void LED_Set(uint8_t state)
{
    if (state)
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13); // 低电平点亮
    }
    else
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_13);   // 高电平熄灭
    }
}

/**
 * @brief 精确毫秒延时 (基于SysTick计数)
 */
static void Delay_ms(uint32_t ms)
{
    uint32_t start = g_tick_ms;
    while ((g_tick_ms - start) < ms);
}

int main(void)
{
    LoRa_Packet_t rx_packet;
    uint32_t last_cmd_tick = 0;
    uint8_t is_moving = 0;

    /* 1. 中断优先级分组配置 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* 2. 配置 SysTick 为 1ms 滴答中断 (系统主频 72MHz) */
    SysTick_Config(SystemCoreClock / 1000);

    /* 3. 硬件模块初始化 */
    LED_Init();                 // PC13 LED
    Motor_Init();               // TB6612 电机驱动 (TIM1_CH1: PA8, TIM1_CH2: PA9, 方向: PB12~PB15)
    LoRa_Init(9600);            // LoRa 串口通信 (USART2: PA2-TX, PA3-RX, 9600bps)

    /* 开机指示: 闪烁三次确认系统与外设初始化完毕 */
    LED_Set(1); Delay_ms(120); LED_Set(0); Delay_ms(120);
    LED_Set(1); Delay_ms(120); LED_Set(0); Delay_ms(120);
    LED_Set(1); Delay_ms(120); LED_Set(0);

    /* 主循环: 实时监听 LoRa 接收信号并控制双路电机 */
    while (1)
    {
        /* -------------------------------------------------------------
         * 统一命令获取: 自动兼容上位机10字节协议包与单字符指令，杜绝拆包冲突
         * ------------------------------------------------------------- */
        if (LoRa_GetCommand(&rx_packet))
        {
            switch (rx_packet.cmd)
            {
                case LORA_CMD_DIRECT_PWM:   // 左右电机直接差速控制
                    Motor_SetSpeed(rx_packet.left_speed, rx_packet.right_speed);
                    last_cmd_tick = g_tick_ms;
                    is_moving = (rx_packet.left_speed != 0 || rx_packet.right_speed != 0);
                    break;

                case LORA_CMD_FORWARD:      // 前进
                    Motor_Forward(rx_packet.left_speed);
                    last_cmd_tick = g_tick_ms;
                    is_moving = 1;
                    break;

                case LORA_CMD_BACKWARD:     // 后退
                    Motor_Backward(rx_packet.left_speed);
                    last_cmd_tick = g_tick_ms;
                    is_moving = 1;
                    break;

                case LORA_CMD_TURN_LEFT:    // 差速左转
                    Motor_TurnLeft(rx_packet.left_speed);
                    last_cmd_tick = g_tick_ms;
                    is_moving = 1;
                    break;

                case LORA_CMD_TURN_RIGHT:   // 差速右转
                    Motor_TurnRight(rx_packet.right_speed);
                    last_cmd_tick = g_tick_ms;
                    is_moving = 1;
                    break;

                case LORA_CMD_BRAKE:        // 刹车
                    Motor_Brake();
                    is_moving = 0;
                    break;

                case LORA_CMD_STOP:         // 正常停机
                default:
                    Motor_Stop();
                    is_moving = 0;
                    break;
            }
        }

        /* -------------------------------------------------------------
         * 安全防失控看门狗保护机制:
         * 只要处于航行中，若连续超过 600ms 未收到任何有效指令，立即自动停机！
         * ------------------------------------------------------------- */
        if (is_moving)
        {
            if ((g_tick_ms - last_cmd_tick) > CMD_TIMEOUT_MS)
            {
                Motor_Stop();
                is_moving = 0;
                LED_Set(0); // 超时停机指示灯灭
            }
            else
            {
                LED_Set(1); // 持续航行中指示灯保持常亮
            }
        }
        else
        {
            LED_Set(0);
        }
    }
}
