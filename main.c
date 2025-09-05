#define ENCODER_STEPS_PER_REV 20   // 定义：旋转编码器一圈产生 20 个脉冲
#define SERVO2_RANGE_DEG     180   // 定义：舵机2 的角度范围为 180°
#define SERVO1_RANGE_DEG     360   // 定义：舵机1 的虚拟角度范围为 360°（连续旋转舵机，软件模拟角度）
 
#include "stm32f10x.h"                  // STM32 标准外设库头文件
#include "Delay.h"                      // 延时函数
#include "OLED.h"                       // OLED 显示驱动
#include "PWM.h"                        // PWM 驱动
//#include "Key.h"                      // 按键驱动（此处未使用）
#include "Servo.h"                      // 舵机控制
#include "Encoder.h"                    // 编码器驱动
#include <math.h>                       // 数学库（用于计算）
 
/* 舵机1角度变量（软件虚拟角度，实际为连续旋转舵机） */
float TargetAngle1 = 0;     // 目标角度（由编码器1控制）
float CurrentAngle1 = 0;    // 当前角度（软件模拟，非真实传感器反馈）
 
/* 舵机2角度变量（实际角度，可控 0~180°） */
float Angle2 = 90;          // 初始角度设为 90°（中位）
 
/* OLED 显示缓存（用于减少重复刷新） */
float lastAngle1 = -1;      // 上一次显示的舵机1角度
float lastAngle2 = -1;      // 上一次显示的舵机2角度
 
/* PID 控制参数（用于舵机1的速度控制） */
float Kp = 0.02f;   // 比例系数（放大误差）
float Ki = 0.0005f; // 积分系数（累积误差，消除静差）
float Kd = 0.01f;   // 微分系数（预测误差变化，防止震荡）
 
float integral = 0;   // 积分项
float lastError = 0;  // 上一次误差（用于微分项）
 
int main(void)
{
    /* 初始化外设 */
    OLED_Init();         // OLED 显示初始化
    PWM_Init();          // TIM2 PWM 初始化
    Encoder_Init();      // 外部中断方式初始化两个旋转编码器
 
    Servo1_SetSpeed(0);   // 舵机1初始化时停止（速度 = 0）
    Servo2_SetAngle(Angle2);  // 舵机2转到 90° 初始位置
 
    /* 在 OLED 上固定显示标签 */
    OLED_ShowString(1,1,"Servo1Ang:");  // 第1行显示 "Servo1Ang:"
    OLED_ShowString(2,1,"Servo2Ang:");  // 第2行显示 "Servo2Ang:"
 
    while(1)
    {
        /* ================= 舵机1：由编码器1控制目标角度 ================= */
        int16_t val1 = Encoder1_Get();   // 读取编码器1脉冲变化量（每次调用后清零）
        if(val1 != 0)                    // 如果有旋转
        {
            // 将编码器脉冲转换为角度变化：一圈 360°，编码器一圈 20 步
            TargetAngle1 += val1 * (SERVO1_RANGE_DEG / ENCODER_STEPS_PER_REV);
 
            // 角度循环在 [0,360) 范围内
            if(TargetAngle1 < 0) TargetAngle1 += 360;
            if(TargetAngle1 >= 360) TargetAngle1 -= 360;
        }
 
        /* 计算误差，保证在 -180° ~ +180° 范围内（最小旋转路径） */
        float error = TargetAngle1 - CurrentAngle1;
        if(error > 180) error -= 360;
        if(error < -180) error += 360;
 
        /* PID 控制 */
        integral += error;                        // 积分项
        float derivative = error - lastError;     // 微分项
        lastError = error;                        // 保存误差
 
        float output = Kp*error + Ki*integral + Kd*derivative;  // PID公式
 
        /* 限制输出范围（速度范围 -1.0 ~ 1.0） */
        if(output > 1.0f) output = 1.0f;
        if(output < -1.0f) output = -1.0f;
 
        Servo1_SetSpeed(output);  // 设置舵机1速度（PWM 1500±500us）
 
        /* 模拟舵机1的角度变化（假设 output 与角度变化成正比） */
        CurrentAngle1 += output * 2;  // “2”为速度因子，可调整快慢
        if(CurrentAngle1 < 0) CurrentAngle1 += 360;
        if(CurrentAngle1 >= 360) CurrentAngle1 -= 360;
 
        /* OLED 更新舵机1角度（避免重复刷新） */
        if((int)CurrentAngle1 != (int)lastAngle1)
        {
            OLED_ShowNum(1, 11, (uint16_t)CurrentAngle1, 3);
            lastAngle1 = CurrentAngle1;
        }
 
        /* ================= 舵机2：由编码器2控制实际角度 ================= */
        int16_t val2 = Encoder2_Get();   // 读取编码器2脉冲变化量
        if(val2 != 0)
        {
            Angle2 += val2 * (SERVO2_RANGE_DEG / ENCODER_STEPS_PER_REV);
            if(Angle2 < 0) Angle2 = 0;       // 限制最小角度
            if(Angle2 > 180) Angle2 = 180;   // 限制最大角度
            Servo2_SetAngle(Angle2);         // 设置舵机2角度
        }
 
        /* OLED 更新舵机2角度 */
        if(Angle2 != lastAngle2)
        {
            OLED_ShowNum(2, 11, (uint16_t)Angle2, 3);
            lastAngle2 = Angle2;
        }
    }
}
