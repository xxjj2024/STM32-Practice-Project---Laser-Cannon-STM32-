#include "stm32f10x.h"                  // Device header
 
/* -------- 编码器1 相关变量 -------- */
int16_t Encoder1_Count;   // 保存累计脉冲数
int16_t Encoder1_Get(void)
{
    int16_t Temp = Encoder1_Count;
    Encoder1_Count = 0;   // 读取后清零（返回增量）
    return Temp;
}
 
/* -------- 编码器2 相关变量 -------- */
int16_t Encoder2_Count;
int16_t Encoder2_Get(void)
{
    int16_t Temp = Encoder2_Count;
    Encoder2_Count = 0;
    return Temp;
}
 
/* -------- 初始化编码器 GPIO + EXTI -------- */
void Encoder_Init(void)
{
    /* 1. 开启 GPIOB/A 时钟 & AFIO */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
 
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 
    /* --- 编码器1：PB0, PB1 --- */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
 
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
 
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿触发
    EXTI_Init(&EXTI_InitStructure);
 
    /* NVIC 配置 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
 
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;  // PB0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
 
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;  // PB1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);
 
    /* --- 编码器2：PA6, PA7 --- */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource7);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6);
 
    EXTI_InitStructure.EXTI_Line = EXTI_Line7 | EXTI_Line6;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_Init(&EXTI_InitStructure);
 
    NVIC_InitStructure.NVIC_IRQChannel =  EXTI9_5_IRQn;  // PA6/PA7
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_Init(&NVIC_InitStructure);
}
 
/* -------- 编码器1 中断处理 -------- */
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) == SET) // PB0 下降沿
    {
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0) // 读取相位差
            Encoder1_Count--;  // 逆时针
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
void EXTI1_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line1) == SET) // PB1 下降沿
    {
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0)
            Encoder1_Count++;  // 顺时针
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}
 
/* -------- 编码器2 中断处理 -------- */
void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line6) == SET) // PA6
    {
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == 0)
            Encoder2_Count--;  // 逆时针
        EXTI_ClearITPendingBit(EXTI_Line6);
    }
    if(EXTI_GetITStatus(EXTI_Line7) == SET) // PA7
    {
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == 0)
            Encoder2_Count++;  // 顺时针
        EXTI_ClearITPendingBit(EXTI_Line7);
    }
}
