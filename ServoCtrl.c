#include "ServoCtrl.h"
#include "Servo.h"
#include "OLED.h"
#include "Bluetooth.h"
#include <math.h>
 
/* ======================== PID 参数区 ======================== */
/* 
 * 对于连续旋转舵机，需要控制转到目标角度。
 * 我们使用 PID 算法来控制舵机速度，使其逐渐逼近目标角度。
 * Kp：比例系数，误差越大，输出越大（反应速度快，但可能震荡）
 * Ki：积分系数，误差累积补偿（消除静差，但太大会震荡）
 * Kd：微分系数，预测误差变化趋势（抑制超调，但太大噪声敏感）
 */
static float Kp = 0.02f;     // 比例增益
static float Ki = 0.0005f;   // 积分增益
static float Kd = 0.01f;     // 微分增益
 
/* PID 中的中间变量 */
static float integral = 0;   // 误差的积分（累积误差）
static float lastError = 0;  // 上一次的误差（用于微分计算）
 
/* ======================== 舵机角度状态 ======================== */
/*
 * CurrentAngle1：记录连续舵机（360°舵机）的当前位置角度
 * lastAngle1   ：上一次显示到 OLED 的角度（避免反复刷新屏幕）
 * lastAngle2   ：同理，针对 180° 舵机
 */
static float CurrentAngle1 = 0;  
static float lastAngle1 = -1;    
static float lastAngle2 = -1;    
 
/* ======================== 初始化函数 ======================== */
void ServoCtrl_Init(void)
{
    Servo1_SetSpeed(0);           // 初始化连续舵机速度为 0（停止）
    Servo2_SetAngle(BT_Angle2);   // 初始化普通舵机，设置为蓝牙传来的角度值
 
    /* 在 OLED 上显示标题文字 */
    OLED_ShowString(1,1,"Servo1Ang:"); // 第一行显示连续舵机角度
    OLED_ShowString(2,1,"Servo2Ang:"); // 第二行显示普通舵机角度
}
 
/* ======================== 更新函数 ======================== */
void ServoCtrl_Update(void)
{
    /* -------- 舵机1: 连续舵机（带 PID 控制） -------- */
    float error = BT_TargetAngle1 - CurrentAngle1;  // 计算目标角度与当前角度的误差
 
    /* 防止旋转路径过长，选择最短旋转方向 */
    if(error > 180)  error -= 360;
    if(error < -180) error += 360;
 
    /* ===== PID 三部分 ===== */
    integral += error;                      // 积分项：累积误差
    float derivative = error - lastError;   // 微分项：误差变化率
    lastError = error;                      // 更新误差记录，供下次使用
 
    /* PID 控制公式：输出值（控制舵机速度） */
    float output = Kp*error + Ki*integral + Kd*derivative;
 
    /* 限制输出范围在 -1.0 ~ 1.0 （舵机 API 要求速度范围） */
    if(output > 1.0f)  output = 1.0f;
    if(output < -1.0f) output = -1.0f;
 
    /* 将 PID 输出转换为舵机转速 */
    Servo1_SetSpeed(output);
 
    /* 模拟连续舵机的角度变化（实际硬件需编码器反馈，这里软件虚拟） */
    CurrentAngle1 += output * 2;  // 根据速度推算角度变化（2为比例因子，调节响应速度）
 
    /* 保证角度范围在 0 ~ 360 之间 */
    if(CurrentAngle1 < 0)     CurrentAngle1 += 360;
    if(CurrentAngle1 >= 360)  CurrentAngle1 -= 360;
 
    /* OLED 显示舵机1角度（避免频繁刷新，只有角度变化才更新） */
    if((int)CurrentAngle1 != (int)lastAngle1)
    {
        OLED_ShowNum(1, 11, (uint16_t)CurrentAngle1, 3); // 显示整数角度值
        lastAngle1 = CurrentAngle1;
    }
 
    /* -------- 舵机2: 普通 180° 舵机 -------- */
    Servo2_SetAngle(BT_Angle2);   // 直接设置角度（蓝牙接收到的目标值）
 
    /* OLED 显示舵机2角度（同样避免频繁刷新） */
    if((int)BT_Angle2 != (int)lastAngle2)
    {
        OLED_ShowNum(2, 11, (uint16_t)BT_Angle2, 3);
        lastAngle2 = BT_Angle2;
    }
}
