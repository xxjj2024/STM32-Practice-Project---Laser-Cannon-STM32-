#include "stm32f10x.h"                  // Device header
 
/* TIM2 初始化为 PWM 输出 */
void PWM_Init(void)
{
    /* 1. 开启时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   // TIM2 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // GPIOA 时钟
 
    /* 2. 配置 GPIO：PA1->TIM2_CH2, PA2->TIM2_CH3 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        // 复用推挽
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;   // PA1 -> 通道2
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;   // PA2 -> 通道3
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    /* 3. TIM2 基本配置 */
    TIM_InternalClockConfig(TIM2);  // 使用内部时钟
 
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;  
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;   // ARR = 20000 → 周期 = 20ms
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;   // 72MHz / 72 = 1MHz
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
 
    /* 4. PWM 配置 */
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;         // PWM 模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 高电平有效
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
 
    TIM_OCInitStructure.TIM_Pulse = 1500;   // 初始脉宽 = 1.5ms（舵机中位）
    TIM_OC2Init(TIM2, &TIM_OCInitStructure); // 通道2 → PA1
 
    TIM_OCInitStructure.TIM_Pulse = 1500;
    TIM_OC3Init(TIM2, &TIM_OCInitStructure); // 通道3 → PA2
 
    /* 5. 使能定时器 */
    TIM_Cmd(TIM2, ENABLE);
}
 
/* 设置 PWM 占空比 → 控制舵机角度 */
void PWM_SetCompare2(uint16_t Compare) { TIM_SetCompare2(TIM2, Compare); } // PA1
void PWM_SetCompare3(uint16_t Compare) { TIM_SetCompare3(TIM2, Compare); } // PA2
