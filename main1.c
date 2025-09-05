#include "stm32f10x.h"   // STM32 标准外设库核心头文件，包含寄存器定义、外设初始化函数
#include "OLED.h"        // OLED 显示屏驱动头文件
#include "Encoder.h"     // 编码器驱动头文件，用于读取旋转增量
#include "Bluetooth.h"   // 蓝牙串口模块驱动头文件
#include <stdio.h>       // C 标准输入输出库，用于 sprintf 格式化字符串
 
/* ----------------- 宏定义 ----------------- */
#define ENCODER_STEPS_PER_REV 20   // 编码器一圈的步数（脉冲数），决定角度分辨率
#define SERVO2_RANGE_DEG     180   // 舵机2的最大旋转角度（0~180度）
#define SERVO1_RANGE_DEG     360   // 舵机1的角度范围（模拟成0~360度一圈）
 
/* ----------------- 全局变量 ----------------- */
float TargetAngle1 = 0;   // 舵机1目标角度，初值设为0度
float Angle2 = 90;        // 舵机2角度，初值设为90度（居中）
char txBuf[64];           // 串口发送缓存区，存放要通过蓝牙发出去的字符串
 
/* ----------------- 主函数 ----------------- */
int main(void)
{
    OLED_Init();           // 初始化OLED显示屏
    Encoder_Init();        // 初始化两个旋转编码器
    Bluetooth_Init(9600);  // 初始化蓝牙串口模块，波特率9600bps
 
    OLED_ShowString(1,1,"SendA1:");  // OLED第1行显示固定标签 "SendA1:"
    OLED_ShowString(2,1,"SendA2:");  // OLED第2行显示固定标签 "SendA2:"
 
    while(1)  // 主循环，持续运行
    {
        /* ---------- 读取编码器1 ---------- */
        int16_t val1 = Encoder1_Get();   // 获取编码器1的增量（可能为 -1,0,+1 ...）
        if(val1 != 0)  // 如果有旋转动作
        {
            // 累加角度：每步对应 360/20 = 18 度
            TargetAngle1 += val1 * (SERVO1_RANGE_DEG / ENCODER_STEPS_PER_REV);
 
            // 角度环绕处理（保持在 0~359 范围内）
            if(TargetAngle1 < 0) TargetAngle1 += 360;
            if(TargetAngle1 >= 360) TargetAngle1 -= 360;
        }
 
        /* ---------- 读取编码器2 ---------- */
        int16_t val2 = Encoder2_Get();   // 获取编码器2的增量
        if(val2 != 0)  // 如果有旋转动作
        {
            // 累加角度：每步对应 180/20 = 9 度
            Angle2 += val2 * (SERVO2_RANGE_DEG / ENCODER_STEPS_PER_REV);
 
            // 限幅处理（限制在0~180度之间）
            if(Angle2 < 0) Angle2 = 0;
            if(Angle2 > 180) Angle2 = 180;
        }
 
        /* ---------- 通过蓝牙发送数据 ---------- */
        // 格式化为字符串，例如 "A1:45.0,A2:90.0\n"
        sprintf(txBuf, "A1:%.1f,A2:%.1f\n", TargetAngle1, Angle2);
        Bluetooth_SendString(txBuf);  // 通过串口逐字节发送字符串
 
        /* ---------- OLED 显示 ---------- */
        // 在第1行第8列显示舵机1角度（整数部分，3位宽度）
        OLED_ShowNum(1, 8, (uint16_t)TargetAngle1, 3);
        // 在第2行第8列显示舵机2角度（整数部分，3位宽度）
        OLED_ShowNum(2, 8, (uint16_t)Angle2, 3);
 
        /* ---------- 简单延时 ---------- */
        for(volatile int i=0; i<500000; i++);  // 忙等延时，防止刷屏过快
    }
}
