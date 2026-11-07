#include "lora.h"

/* 环形缓冲区变量 */
static volatile uint8_t s_rx_buffer[LORA_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head = 0; // 写指针
static volatile uint16_t s_rx_tail = 0; // 读指针

/**
 * @brief 初始化LoRa串口接口 (USART2, PA2-TX, PA3-RX)
 * @param baudrate 串口通信波特率 (推荐 9600 或 115200)
 */
void LoRa_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 开启外设时钟 (USART2位于APB1总线，GPIOA位于APB2总线) */
    RCC_APB1PeriphClockCmd(LORA_USART_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(LORA_GPIO_CLK, ENABLE);

    /* 配置 PA2 为复用推挽输出 (TX) */
    GPIO_InitStructure.GPIO_Pin = LORA_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LORA_GPIO_PORT, &GPIO_InitStructure);

    /* 配置 PA3 为浮空输入 (RX) */
    GPIO_InitStructure.GPIO_Pin = LORA_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(LORA_GPIO_PORT, &GPIO_InitStructure);

    /* USART2 参数配置 */
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(LORA_USART, &USART_InitStructure);

    /* 使能 USART2 接收中断 */
    USART_ITConfig(LORA_USART, USART_IT_RXNE, ENABLE);

    /* 配置 NVIC 中断向量 */
    NVIC_InitStructure.NVIC_IRQChannel = LORA_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 使能 USART2 */
    USART_Cmd(LORA_USART, ENABLE);

    /* 清空接收缓冲区 */
    LoRa_ClearRxBuffer();
}

/**
 * @brief USART2 中断服务函数 - 接收数据写入环形缓冲区
 */
void LORA_IRQHandler(void)
{
    if (USART_GetITStatus(LORA_USART, USART_IT_RXNE) != RESET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(LORA_USART);
        uint16_t next_head = (s_rx_head + 1) % LORA_RX_BUFFER_SIZE;

        /* 若缓冲区未满则存入 */
        if (next_head != s_rx_tail)
        {
            s_rx_buffer[s_rx_head] = data;
            s_rx_head = next_head;
        }
        /* 读寄存器已自动清除RXNE标志位，安全起见显式清除 */
        USART_ClearITPendingBit(LORA_USART, USART_IT_RXNE);
    }
}

/**
 * @brief 获取当前接收缓冲区中未读的字节数
 * @return 未读字节数
 */
uint16_t LoRa_Available(void)
{
    if (s_rx_head >= s_rx_tail)
    {
        return (s_rx_head - s_rx_tail);
    }
    else
    {
        return (LORA_RX_BUFFER_SIZE - s_rx_tail + s_rx_head);
    }
}

/**
 * @brief 从接收缓冲区读取一个字节
 * @return 读取到的字节，若无数据返回0
 */
uint8_t LoRa_Read(void)
{
    uint8_t data = 0;
    if (s_rx_head != s_rx_tail)
    {
        data = s_rx_buffer[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1) % LORA_RX_BUFFER_SIZE;
    }
    return data;
}

/**
 * @brief 从接收缓冲区读取指定长度字节
 * @param buffer 目标存储数组
 * @param max_len 最大读取长度
 * @return 实际读取到的字节数
 */
uint16_t LoRa_ReadBytes(uint8_t *buffer, uint16_t max_len)
{
    uint16_t count = 0;
    while (count < max_len && LoRa_Available() > 0)
    {
        buffer[count++] = LoRa_Read();
    }
    return count;
}

/**
 * @brief 清空接收缓冲区
 */
void LoRa_ClearRxBuffer(void)
{
    s_rx_head = 0;
    s_rx_tail = 0;
}

/**
 * @brief 发送单个字节
 * @param byte 要发送的字节
 */
void LoRa_SendByte(uint8_t byte)
{
    USART_SendData(LORA_USART, byte);
    while (USART_GetFlagStatus(LORA_USART, USART_FLAG_TXE) == RESET);
}

/**
 * @brief 发送字节数组
 * @param array 数组指针
 * @param len 数组长度
 */
void LoRa_SendArray(const uint8_t *array, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        LoRa_SendByte(array[i]);
    }
}

/**
 * @brief 发送以 '\0' 结尾的字符串
 * @param str 字符串指针
 */
void LoRa_SendString(const char *str)
{
    while (*str)
    {
        LoRa_SendByte((uint8_t)(*str));
        str++;
    }
}

/**
 * @brief 读取单个控制字符 (用于简单的单键遥控，如 'F','B','L','R','S')
 * @param ch 存储字符指针
 * @return 1: 成功获取到一个字符; 0: 暂无新字符
 */
uint8_t LoRa_GetChar(char *ch)
{
    if (LoRa_Available() > 0)
    {
        *ch = (char)LoRa_Read();
        return 1;
    }
    return 0;
}

/**
 * @brief 环形缓冲区偷看第 offset 个字节 (不移动读指针)
 */
static uint8_t LoRa_Peek(uint16_t offset)
{
    uint16_t idx = (s_rx_tail + offset) % LORA_RX_BUFFER_SIZE;
    return s_rx_buffer[idx];
}

/**
 * @brief 解析遥控器发来的控制数据包
 * 格式: [0x55] [0xAA] [CMD] [Left_H] [Left_L] [Right_H] [Right_L] [Checksum] [0x0D] [0x0A]
 * 长度: 10 字节
 * Checksum: CMD + Left_H + Left_L + Right_H + Right_L 的低8位
 * @param packet 解析成功后写入的数据包结构体指针
 * @return 1: 成功解析出一包有效数据; 0: 未接收到完整或有效数据包
 */
uint8_t LoRa_GetPacket(LoRa_Packet_t *packet)
{
    while (LoRa_Available() >= 10)
    {
        /* 寻找帧头 0x55 0xAA */
        if (LoRa_Peek(0) == LORA_FRAME_HEAD1 && LoRa_Peek(1) == LORA_FRAME_HEAD2)
        {
            /* 检查帧尾 0x0D 0x0A */
            if (LoRa_Peek(8) == LORA_FRAME_TAIL1 && LoRa_Peek(9) == LORA_FRAME_TAIL2)
            {
                uint8_t cmd     = LoRa_Peek(2);
                uint8_t left_h  = LoRa_Peek(3);
                uint8_t left_l  = LoRa_Peek(4);
                uint8_t right_h = LoRa_Peek(5);
                uint8_t right_l = LoRa_Peek(6);
                uint8_t sum     = LoRa_Peek(7);

                /* 计算校验和 */
                uint8_t calc_sum = (uint8_t)(cmd + left_h + left_l + right_h + right_l);
                if (calc_sum == sum)
                {
                    /* 校验成功，取出并赋值 */
                    packet->cmd = cmd;
                    packet->left_speed = (int16_t)((left_h << 8) | left_l);
                    packet->right_speed = (int16_t)((right_h << 8) | right_l);

                    /* 丢弃这10个字节 */
                    s_rx_tail = (s_rx_tail + 10) % LORA_RX_BUFFER_SIZE;
                    return 1;
                }
            }
        }
        /* 若帧头不匹配或帧尾有误，丢弃队头1字节，滑动窗口继续寻找 */
        s_rx_tail = (s_rx_tail + 1) % LORA_RX_BUFFER_SIZE;
    }
    return 0;
}

/**
 * @brief 综合命令获取接口 (彻底解决数据包与字符冲突问题)
 * 优先解析10字节结构化数据包；若不足10字节且队首为0x55，安全保留等待；
 * 仅当队首不是0x55且匹配字符命令时，才作为字符解析，未知字符直接丢弃绝不关停电机！
 * @param packet 输出的控制包结构体指针
 * @return 1: 成功获取到一条控制指令; 0: 暂无新指令
 */
uint8_t LoRa_GetCommand(LoRa_Packet_t *packet)
{
    /* 1. 首先尝试解析 10 字节数据包 */
    if (LoRa_GetPacket(packet))
    {
        return 1;
    }

    /* 2. 检查是否有单字符控制指令 */
    if (LoRa_Available() > 0)
    {
        uint8_t head_byte = LoRa_Peek(0);

        /* 关键保护: 如果队首是 0x55，说明正在接收数据包的前半部分，绝对不能读走！ */
        if (head_byte == LORA_FRAME_HEAD1)
        {
            return 0;
        }

        /* 队首不是 0x55，安全出队 1 个字节 */
        char ch = (char)LoRa_Read();
        switch (ch)
        {
            case 'F':
            case 'f':
                packet->cmd = LORA_CMD_FORWARD;
                packet->left_speed = 600;
                packet->right_speed = 600;
                return 1;

            case 'B':
            case 'b':
                packet->cmd = LORA_CMD_BACKWARD;
                packet->left_speed = 600;
                packet->right_speed = 600;
                return 1;

            case 'L':
            case 'l':
                packet->cmd = LORA_CMD_TURN_LEFT;
                packet->left_speed = 600;
                packet->right_speed = 600;
                return 1;

            case 'R':
            case 'r':
                packet->cmd = LORA_CMD_TURN_RIGHT;
                packet->left_speed = 600;
                packet->right_speed = 600;
                return 1;

            case 'P':
            case 'p':
                packet->cmd = LORA_CMD_BRAKE;
                packet->left_speed = 0;
                packet->right_speed = 0;
                return 1;

            case 'S':
            case 's':
                packet->cmd = LORA_CMD_STOP;
                packet->left_speed = 0;
                packet->right_speed = 0;
                return 1;

            default:
                /* 收到换行、空格等杂波，直接忽略丢弃，绝不能关闭电机！ */
                return 0;
        }
    }

    return 0;
}
