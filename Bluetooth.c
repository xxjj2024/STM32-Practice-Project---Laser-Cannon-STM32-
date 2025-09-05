#include "Bluetooth.h"
#include <stdio.h>    // 提供 sscanf 格式化解析
#include <string.h>   // 提供字符串操作
 
/* ----------------- 接收缓冲区 ----------------- */
static char rxBuf[64];     // 存储接收到的一帧数据（最大63字节 + '\0'）
static uint8_t idx = 0;    // 当前写入位置索引
 
/* ----------------- 外部可用的接收数据 ----------------- */
float BT_TargetAngle1 = 0;  // 从蓝牙接收的舵机1目标角度
float BT_Angle2 = 90;       // 从蓝牙接收的舵机2目标角度（初值90度）
 
/* ----------------- 初始化蓝牙串口（USART1） ----------------- */
void Bluetooth_Init(uint32_t baud)
{
    // 打开USART1和GPIOA的时钟（因为USART1的TX/RX用的是PA9/PA10）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
 
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;   // 设置端口速率（输出翻转速度）
 
    // 配置PA9 (USART1 TX) 为复用推挽输出
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin  = GPIO_Pin_9;
    GPIO_Init(GPIOA, &gpio);
 
    // 配置PA10 (USART1 RX) 为浮空输入
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Pin  = GPIO_Pin_10;
    GPIO_Init(GPIOA, &gpio);
 
    // 配置USART1参数
    USART_InitTypeDef usart;
    usart.USART_BaudRate = baud;                             // 波特率
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;        // 允许收发
    usart.USART_Parity = USART_Parity_No;                    // 无校验
    usart.USART_StopBits = USART_StopBits_1;                 // 1个停止位
    usart.USART_WordLength = USART_WordLength_8b;            // 8位数据
    USART_Init(USART1, &usart);                              // 初始化USART1
 
    USART_Cmd(USART1, ENABLE);                               // 使能USART1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);           // 使能接收中断（RXNE）
 
    NVIC_EnableIRQ(USART1_IRQn);                             // 使能USART1中断向量
}
 
/* ----------------- 串口发送字符串 ----------------- */
void Bluetooth_SendString(char *s)
{
    while(*s)  // 循环直到字符串结尾
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // 等待发送缓冲区空
        USART_SendData(USART1, *s++); // 将字符写入数据寄存器（自动发送出去）
    }
}
 
/* ----------------- USART1中断服务函数 ----------------- */
/* 处理蓝牙接收到的数据，解析为 A1/A2 角度 ----------------- */
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  // 判断是否接收到数据
    {
        char c = USART_ReceiveData(USART1);  // 读取接收到的一个字节（读出后标志位自动清零）
 
        if(c == '\n')  // 如果收到换行符，说明一帧数据结束
        {
            rxBuf[idx] = '\0';  // 给字符串添加结束符
            idx = 0;            // 重置写入索引，准备下一帧
 
            float a1=0, a2=0;
            // 按固定格式解析，例如 "A1:123.4,A2:90.0"
            sscanf(rxBuf, "A1:%f,A2:%f", &a1, &a2);
 
            // 更新全局变量，供主循环或其他模块使用
            BT_TargetAngle1 = a1;
            BT_Angle2       = a2;
        }
        else  // 如果不是换行符，就继续存入缓冲区
        {
            rxBuf[idx++] = c;   // 存储当前字节
            if(idx >= sizeof(rxBuf)) idx = 0;  // 防止溢出（超长时清零）
        }
    }
}
