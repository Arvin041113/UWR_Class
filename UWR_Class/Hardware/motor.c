#include "motor.h"

/**
 * @brief 初始化双路电机驱动硬件外设 (GPIO与TIM1 PWM)
 */
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

    /* 1. 开启外设时钟: GPIOB, GPIOA, TIM1, AFIO */
    RCC_APB2PeriphClockCmd(MOTOR_DIR_GPIO_CLK | MOTOR_PWM_GPIO_CLK | MOTOR_TIM_CLK | RCC_APB2Periph_AFIO, ENABLE);

    /* 2. 配置方向控制引脚为通用推挽输出 (PB12, PB13, PB14, PB15) */
    GPIO_InitStructure.GPIO_Pin = MOTOR_AIN1_PIN | MOTOR_AIN2_PIN | MOTOR_BIN1_PIN | MOTOR_BIN2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_DIR_PORT, &GPIO_InitStructure);

    /* 默认方向全部拉低 (滑行停止状态) */
    GPIO_ResetBits(MOTOR_DIR_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN | MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);

    /* 3. 配置PWM引脚为复用推挽输出 (PA8: TIM1_CH1, PA9: TIM1_CH2) */
    GPIO_InitStructure.GPIO_Pin = MOTOR_PWMA_PIN | MOTOR_PWMB_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_PWM_PORT, &GPIO_InitStructure);

    /* 4. 配置 TIM1 时基单元: 计数频率 1MHz, PWM 频率 1kHz (ARR=999, PSC=71) */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;                // 自动重装载值 ARR (0~999 对应 1000级调速)
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;               // 预分频器 PSC (72MHz / 72 = 1MHz)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(MOTOR_TIM, &TIM_TimeBaseStructure);

    /* 5. 配置 TIM1 输出比较通道 1 (PWMA, PA8) 与 通道 2 (PWMB, PA9) */
    TIM_OCStructInit(&TIM_OCInitStructure);                     // 彻底清空结构体未初始化脏数据
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;           // PWM 模式 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;// 主通道输出使能
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; // 禁用互补通道
    TIM_OCInitStructure.TIM_Pulse = 0;                          // 初始占空比 0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   // 高电平有效
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;

    /* 配置通道 1 (PA8) */
    TIM_OC1Init(MOTOR_TIM, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);

    /* 配置通道 2 (PA9) */
    TIM_OC2Init(MOTOR_TIM, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(MOTOR_TIM, TIM_OCPreload_Enable);

    /* 使能自动重装载寄存器预装载 */
    TIM_ARRPreloadConfig(MOTOR_TIM, ENABLE);

    /* 6. 高级定时器刹车与死区控制 (BDTR) 极其关键配置:
     * 注意: TIM1 的 BKIN 硬件刹车脚恰好是 PB12！
     * 原理图中 PB12 被用作 AIN1，一旦前进拉高 PB12，若不关闭刹车功能，
     * 硬件刹车会瞬间锁死 MOE 并关闭 PWM 输出！因此必须显式禁用刹车！
     */
    TIM_BDTRStructInit(&TIM_BDTRInitStructure);
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStructure.TIM_DeadTime = 0;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;        // 严禁开启刹车功能！防止PB12触发PWM切断
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable; // 自动输出使能
    TIM_BDTRConfig(MOTOR_TIM, &TIM_BDTRInitStructure);

    /* 7. 使能高级定时器主输出 (MOE) 并启动计数器 */
    TIM_CtrlPWMOutputs(MOTOR_TIM, ENABLE);
    TIM_Cmd(MOTOR_TIM, ENABLE);

    /* 初始电机全停 */
    Motor_Stop();
}

/**
 * @brief 设置左电机 (Motor A) 的运行速度与方向
 * @param speed 取值范围 -1000 ~ +1000 (>0: 正转, <0: 反转, 0: 停止)
 */
void Motor_SetSpeedA(int16_t speed)
{
    /* 幅值限制 */
    if (speed > MOTOR_SPEED_MAX)  speed = MOTOR_SPEED_MAX;
    if (speed < MOTOR_SPEED_MIN)  speed = MOTOR_SPEED_MIN;

    if (speed > 0)
    {
        /* 正转: AIN1=1, AIN2=0 */
        GPIO_SetBits(MOTOR_DIR_PORT, MOTOR_AIN1_PIN);
        GPIO_ResetBits(MOTOR_DIR_PORT, MOTOR_AIN2_PIN);
        TIM_SetCompare1(MOTOR_TIM, (uint16_t)speed);
        TIM_CtrlPWMOutputs(MOTOR_TIM, ENABLE); // 确保主输出开启
    }
    else if (speed < 0)
    {
        /* 反转: AIN1=0, AIN2=1 */
        GPIO_ResetBits(MOTOR_DIR_PORT, MOTOR_AIN1_PIN);
        GPIO_SetBits(MOTOR_DIR_PORT, MOTOR_AIN2_PIN);
        TIM_SetCompare1(MOTOR_TIM, (uint16_t)(-speed));
        TIM_CtrlPWMOutputs(MOTOR_TIM, ENABLE); // 确保主输出开启
    }
    else
    {
        /* 停止: AIN1=0, AIN2=0 */
        GPIO_ResetBits(MOTOR_DIR_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN);
        TIM_SetCompare1(MOTOR_TIM, 0);
    }
}

/**
 * @brief 设置右电机 (Motor B) 的运行速度与方向
 * @param speed 取值范围 -1000 ~ +1000 (>0: 正转, <0: 反转, 0: 停止)
 */
void Motor_SetSpeedB(int16_t speed)
{
    /* 幅值限制 */
    if (speed > MOTOR_SPEED_MAX)  speed = MOTOR_SPEED_MAX;
    if (speed < MOTOR_SPEED_MIN)  speed = MOTOR_SPEED_MIN;

    if (speed > 0)
    {
        /* 正转: BIN1=1, BIN2=0 */
        GPIO_SetBits(MOTOR_DIR_PORT, MOTOR_BIN1_PIN);
        GPIO_ResetBits(MOTOR_DIR_PORT, MOTOR_BIN2_PIN);
        TIM_SetCompare2(MOTOR_TIM, (uint16_t)speed);
        TIM_CtrlPWMOutputs(MOTOR_TIM, ENABLE); // 确保主输出开启
    }
    else if (speed < 0)
    {
        /* 反转: BIN1=0, BIN2=1 */
        GPIO_ResetBits(MOTOR_DIR_PORT, MOTOR_BIN1_PIN);
        GPIO_SetBits(MOTOR_DIR_PORT, MOTOR_BIN2_PIN);
        TIM_SetCompare2(MOTOR_TIM, (uint16_t)(-speed));
        TIM_CtrlPWMOutputs(MOTOR_TIM, ENABLE); // 确保主输出开启
    }
    else
    {
        /* 停止: BIN1=0, BIN2=0 */
        GPIO_ResetBits(MOTOR_DIR_PORT, MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
        TIM_SetCompare2(MOTOR_TIM, 0);
    }
}

/**
 * @brief 同时设置左右双路电机速度
 * @param speedA 左电机速度 (-1000 ~ +1000)
 * @param speedB 右电机速度 (-1000 ~ +1000)
 */
void Motor_SetSpeed(int16_t speedA, int16_t speedB)
{
    Motor_SetSpeedA(speedA);
    Motor_SetSpeedB(speedB);
}

/**
 * @brief 双电机停止 (滑行停止)
 */
void Motor_Stop(void)
{
    GPIO_ResetBits(MOTOR_DIR_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN | MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
    TIM_SetCompare1(MOTOR_TIM, 0);
    TIM_SetCompare2(MOTOR_TIM, 0);
}

/**
 * @brief 双电机紧急刹车 (短路制动)
 */
void Motor_Brake(void)
{
    /* TB6612制动模式: IN1=1, IN2=1 */
    GPIO_SetBits(MOTOR_DIR_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN | MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
    TIM_SetCompare1(MOTOR_TIM, MOTOR_SPEED_MAX);
    TIM_SetCompare2(MOTOR_TIM, MOTOR_SPEED_MAX);
    TIM_CtrlPWMOutputs(MOTOR_TIM, ENABLE);
}

/**
 * @brief 水下机器人前进
 * @param speed 航行速度 (0 ~ 1000)
 */
void Motor_Forward(int16_t speed)
{
    if (speed < 0) speed = 0;
    Motor_SetSpeed(speed, speed);
}

/**
 * @brief 水下机器人后退
 * @param speed 航行速度 (0 ~ 1000)
 */
void Motor_Backward(int16_t speed)
{
    if (speed < 0) speed = 0;
    Motor_SetSpeed(-speed, -speed);
}

/**
 * @brief 水下机器人原地/差速左转 (左后退，右前进)
 * @param speed 转向速度 (0 ~ 1000)
 */
void Motor_TurnLeft(int16_t speed)
{
    if (speed < 0) speed = 0;
    Motor_SetSpeed(-speed, speed);
}

/**
 * @brief 水下机器人原地/差速右转 (左前进，右后退)
 * @param speed 转向速度 (0 ~ 1000)
 */
void Motor_TurnRight(int16_t speed)
{
    if (speed < 0) speed = 0;
    Motor_SetSpeed(speed, -speed);
}
