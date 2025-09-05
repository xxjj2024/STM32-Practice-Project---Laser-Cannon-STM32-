#include "Servo.h"
#include "PWM.h"
 
/* 舵机初始化 */
void Servo_Init(void)
{
    PWM_Init();
}
 
/* 舵机1（连续旋转舵机），设置速度 -1.0~1.0 */
void Servo1_SetSpeed(float Speed)
{
    if(Speed < -1.0f) Speed = -1.0f;
    if(Speed >  1.0f) Speed =  1.0f;
 
    uint16_t compare = 1500 + (int)(Speed * 500); // 1.0ms~2.0ms 控制速度
    PWM_SetCompare2(compare);  // 输出到 PA1
}
 
/* 舵机2（标准角度舵机），设置角度 0~180° */
void Servo2_SetAngle(float Angle)
{
    if(Angle < 0) Angle = 0;
    if(Angle > 180) Angle = 180;
 
    uint16_t compare = 500 + (Angle / 180.0f) * 2000; // 0.5ms~2.5ms → 0~180°
    PWM_SetCompare3(compare);  // 输出到 PA2
}
