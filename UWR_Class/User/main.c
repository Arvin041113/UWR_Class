#include "stm32f10x.h"
#include "lora.h"
#include "motor.h"

/* 全局毫秒时间戳 (由 SysTick_Handler 每 1ms 递增) */
volatile uint32_t g_tick_ms = 0;

/* 指令超时时间阈值: 若超过 350ms 未收到任何有效运动指令，自动停机保护 */
#define CMD_TIMEOUT_MS          350

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
    GPIO_SetBits(GPIOC, GPIO_Pin_13); // 初始熄灭
}

/**
 * @brief 板载LED翻转
 */
static void LED_Toggle(void)
{
    GPIOC->ODR ^= GPIO_Pin_13;
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
    char cmd_char = 0;
    LoRa_Packet_t rx_packet;
    uint32_t last_cmd_tick = 0;
    uint8_t is_moving = 0;

    /* 1. 中断优先级分组配置 (主程序初始化时统一配置) */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* 2. 配置 SysTick 为 1ms 滴答中断 (系统主频 72MHz) */
    SysTick_Config(SystemCoreClock / 1000);

    /* 3. 硬件模块初始化 */
    LED_Init();                 // PC13 LED
    Motor_Init();               // TB6612 电机驱动 (TIM1_CH1: PA8, TIM1_CH2: PA9, 方向: PB12~PB15)
    LoRa_Init(9600);            // LoRa 串口通信 (USART2: PA2-TX, PA3-RX, 9600bps)

    /* 开机指示: 闪烁两次 */
    LED_Toggle(); Delay_ms(150); LED_Toggle(); Delay_ms(150);
    LED_Toggle(); Delay_ms(150); LED_Toggle();

    /* 主循环: 实时监听 LoRa 接收信号并控制双路电机 */
    while (1)
    {
        /* -------------------------------------------------------------
         * 方式 1: 结构化数据包协议 (推荐: 上位机控制、动态变频调速)
         * 协议帧格式: 0x55 0xAA [CMD] [L_H] [L_L] [R_H] [R_L] [SUM] 0x0D 0x0A
         * ------------------------------------------------------------- */
        if (LoRa_GetPacket(&rx_packet))
        {
            LED_Toggle(); // 接收成功指示灯翻转

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

                case LORA_CMD_STOP:         // 停止
                default:
                    Motor_Stop();
                    is_moving = 0;
                    break;
            }
        }

        /* -------------------------------------------------------------
         * 方式 2: 单字符简易控制 (兼容串口助手直接发字符调试)
         * 'F'/'f': 前进    'B'/'b': 后退
         * 'L'/'l': 左转    'R'/'r': 右转
         * 'S'/'s': 停止    'P'/'p': 刹车
         * ------------------------------------------------------------- */
        if (LoRa_GetChar(&cmd_char))
        {
            LED_Toggle(); // 接收成功指示灯翻转

            switch (cmd_char)
            {
                case 'F':
                case 'f':
                    Motor_Forward(600);     // 默认60%功率前进
                    last_cmd_tick = g_tick_ms;
                    is_moving = 1;
                    break;

                case 'B':
                case 'b':
                    Motor_Backward(600);    // 默认60%功率后退
                    last_cmd_tick = g_tick_ms;
                    is_moving = 1;
                    break;

                case 'L':
                case 'l':
                    Motor_TurnLeft(600);     // 左转
                    last_cmd_tick = g_tick_ms;
                    is_moving = 1;
                    break;

                case 'R':
                case 'r':
                    Motor_TurnRight(600);    // 右转
                    last_cmd_tick = g_tick_ms;
                    is_moving = 1;
                    break;

                case 'P':
                case 'p':
                    Motor_Brake();          // 刹车
                    is_moving = 0;
                    break;

                case 'S':
                case 's':
                default:
                    Motor_Stop();           // 停止
                    is_moving = 0;
                    break;
            }
        }

        /* -------------------------------------------------------------
         * 安全防失控看门狗保护机制:
         * 只有当按键按下时上位机才持续发送运动指令包；
         * 一旦按键松开(停止发送)或无线信号丢包超过阈值，立即自动停止电机！
         * ------------------------------------------------------------- */
        if (is_moving && ((g_tick_ms - last_cmd_tick) > CMD_TIMEOUT_MS))
        {
            Motor_Stop();
            is_moving = 0;
        }
    }
}
