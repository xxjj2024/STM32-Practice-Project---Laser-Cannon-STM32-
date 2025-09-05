# STM32-Practice-Project---Laser-Cannon-STM32-
本文设计了一种基于STM32的双舵机蓝牙激光炮台控制系统，由一代到四代不断替代转化，采用180°限位舵机以及360°连续舵机构建双自由度运动机构，并且在蓝牙模块下以旋转编码器控制，使其可以远程控制并且增加了其灵活性。在此详细说明制作以及升级过程和设想。
刚开始设想做一个上半部分可以上下180°移动,下半部分底座360°移动的激光炮台。于是便开始了实践。(This paper designs a dual-servo Bluetooth laser turret control system based on STM32. It has been continuously replaced and transformed from the first generation to the fourth generation. A 180° limit servo and a 360° continuous servo are adopted to build a dual-degree-of-freedom motion mechanism, and it is controlled by a rotary encoder under the Bluetooth module, which enables remote control and increases its flexibility. Here is a detailed description of the production and upgrade process and ideas.
At the beginning, the idea was to make a laser cannon platform where the upper part could move up and down 180° and the lower part's base could move 360°. So the practice began.)![95e0382dc64e2d198a594b37d3c9aa2b](https://github.com/user-attachments/assets/e23097f8-2110-47fa-a8cd-ddafa44af828)
所需材料清单：

序号	名称	数量	备注说明
1	面包板（Breadboard）	2	用于电路搭建和模块连接
2	杜邦线（公对公、公对母等）	若干	建议准备 30~50 根，方便连接
3	MB-102 电源模块	2	插在面包板上，提供 3.3V / 5V 电源
4	电池（适配 MB-102）	2	建议 9V 方块电池或 7.4V 锂电池
5	SG90 舵机（180° 限位舵机）	1	控制角度在 0°~180°
6	SG90 舵机（360° 连续舵机）	1	可连续旋转，用作角度模拟+PID控制
7	STM32F103C8T6 开发板	2	最小系统板（Blue Pill）
8	KY-008 激光模块	1	激光发射模块（带限流电阻）
9	HC-05 蓝牙模块	2	一发一收，用于无线通信
10	旋转编码器（KY-040 或同类）	2	用于输入角度，连接 STM32 编码器接口
(List of required materials
Serial number name quantity remarks
1. Breadboard (Breadboard) 2 is used for circuit construction and module connection
It is recommended to prepare 30 to 50 DuPont wires (male-to-male, male-to-female, etc.) for easy connection
The 3 MB-102 power module 2 is plugged into the breadboard and provides 3.3V / 5V power supply
4 Battery (compatible with MB-102) 2 It is recommended to use a 9V cube battery or a 7.4V lithium battery
5 SG90 servo (180° limit servo) 1 controls the Angle between 0° and 180°
6 SG90 servo (360° continuous servo) 1 can rotate continuously and is used for Angle simulation +PID control
7 STM32F103C8T6 Development board 2 Minimum System board (Blue Pill)
8 KY-008 Laser Module 1 Laser Emission Module (with Current-limiting Resistor)
9 HC-05 Bluetooth Module 2 transmits and receives, used for wireless communication
10 Rotary encoder (KY-040 or similar) 2 for input Angle, connected to the STM32 encoder interface)

主要过程
起初设想用简单的按钮控制，而单凭if语句只能实现按钮按一下舵机角度变化一下，无法实现舵机角度连续性变化。
(Main process
At first, it was envisioned to use simple button control. However, relying solely on the if statement could only achieve a change in the servo Angle when the button was pressed, and it was impossible to achieve continuous changes in the servo Angle.

)
//uint8_t KeyNum;
//float Angle;
 
//uint8_t i;
 
//int main(void)
//{
//	OLED_Init();
//	Servo_Init();
//	Key_Init();
//	
//	OLED_ShowString(1,1,"Angle:");
//    
//    while (1)
//    {
//       KeyNum=Key_GetNum();
//		if(KeyNum==1)
//		{
//			Angle+=30;
//			if(Angle>180)
//			{
//				Angle=0;
//			}
//		}
//		Servo_SetAngle(Angle);
//		OLED_ShowNum(1,7,Angle,3);
//    }
//}

因此,完善key.c代码，在原代码中添加按住一直返回低电平来实现。(Therefore, to improve the key.c code, add in the original code that holding it down always returns a low level to achieve this.

)

// 新增的实时检测（按住时一直返回）
uint8_t Key_IsPressed(uint8_t keyID)
{
    if (keyID == 1)
    {
        return (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0);
    }
    else if (keyID == 2)
    {
        return (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0);
    }
    return 0;
}

而此时又发现新的问题，如何很好的控制转速。一开始采用的是改变延时速率来提升转速。但如此一来舵机的稳定性便出现的问题。会发生抖动。
(At this point, a new problem emerged: how to control the rotational speed well. At the beginning, the delay rate was changed to increase the rotational speed. But in this way, problems arise with the stability of the servo. Shaking will occur.

)

//int main(void)
//{
//    OLED_Init();
//    Servo_Init();
//    Key_Init();
//    
//    OLED_ShowString(1, 1, "Angle:");
//	OLED_ShowString(2, 1, "Angle:");
//    while (1)
//{
//    if (Key_IsPressed(1)) // 如果按键1被按住
//    {
//        Angle += 1;
//        if (Angle > 180) Angle = 180;
 
//        Servo_SetAngle(Angle);
//        OLED_ShowNum(1, 7, Angle, 3);
//        Delay_ms(1); // 匀速控制
//    }
//	if (Key_IsPressed(2)) // 如果按键1被按住
//    {
//        Angle -= 1;
//        if (Angle < 0) Angle = 0;
 
//        Servo_SetAngle(Angle);
//        OLED_ShowNum(2, 7, Angle, 3);
//        Delay_ms(1); // 匀速控制
//    }
//}



于是加大每一次增加的角度，而延迟秒数不变。
(So the Angle of each increase is increased, while the delay in seconds remains unchanged.)



float Angle = 90;     // 初始角度
float lastAngle = -1; // 用于减少 OLED 刷新频率
 
int main(void)
{
    OLED_Init();
    Servo_Init();
    Key_Init();
    
    OLED_ShowString(1, 1, "Angle+ :");
 
    Servo_SetAngle(Angle); // 舵机先转到中位
 
    while (1)
    {
        // 按键 1：增加角度（快速）
        if (Key_IsPressed(1))
        {
            Angle += 5;               // 每次增加 5°
            if (Angle > 180) Angle = 180;
            Servo_SetAngle(Angle);
            Delay_ms(15);             // 匀速快速控制
        }
 
        // 按键 2：减少角度（快速）
        if (Key_IsPressed(2))
        {
            Angle -= 5;               // 每次减少 5°
            if (Angle < 0) Angle = 0;
            Servo_SetAngle(Angle);
            Delay_ms(15);
        }
 
        // 角度变化才刷新显示
        if (Angle != lastAngle)
        {
            OLED_ShowNum(1, 9, (uint16_t)Angle, 3);
            lastAngle = Angle;
        }
    }
}



的确这样能让舵机较好的连续变化，但是按键寿命有限，频繁操作容易损坏。于是换成更加方便顺手的旋转编码器。其优点也是比较突出的。

优点

操作更直观：想让舵机转多少，就拧多少；比按键舒服很多。

分辨率可调：可以设置每格 1° / 5° / 10°，灵活性高。

响应更快：旋钮快速转几格，舵机就能快速到位。

支持连续调节：不像按钮那样要一直按着，旋钮转动一圈就能从 0° 到 180°。

耐用性更好：旋转编码器机械寿命通常比按键长。
(Indeed, this can enable the servo to change continuously and well, but the lifespan of the buttons is limited and frequent operation is prone to damage. So it was replaced with a more convenient and user-friendly rotary encoder. Its advantages are also quite prominent.
Advantages
More intuitive operation: Just turn the servo as much as you want it to rotate. It's much more comfortable than pressing buttons.
Adjustable resolution: It can be set to 1° / 5° / 10° per grid, offering high flexibility.
Faster response: Just a few quick turns of the knob and the servo can be in place quickly.
Supports continuous adjustment: Unlike buttons that need to be held down all the time, a knob can be turned once to adjust from 0° to 180°.
Better durability: The mechanical life of a rotary encoder is usually longer than that of a key.


)


//main函数中
#include "Encoder.h"
float Angle = 90;     // 初始角度
float lastAngle = -1; // 上一次显示的角度
int main(void)
{
    OLED_Init();
    PWM_Init();
    Encoder_Init();
    
    Servo_SetAngle(Angle); // 初始角度
    
    OLED_ShowString(1,1,"Angle:");
    
    while(1)
    {
        int16_t val = Encoder_Get();   // 获取旋转增量
        if(val != 0)
        {
            Angle += val*5;              // 编码器每跳一下 → 改变 5°
            if(Angle < 0) Angle = 0;
            if(Angle > 180) Angle = 180;
            Servo_SetAngle(Angle);
        }
        
        if(Angle != lastAngle)
        {
            OLED_ShowNum(1, 8, (uint16_t)Angle, 3);
            lastAngle = Angle;
        }
    }
}
//编码器函数
#include "stm32f10x.h"                  // Device header
/*================= 编码器初始化 =================*/
 
int16_t Encoder_Count; 
void Encoder_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
    
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource1);
    
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line=EXTI_Line0|EXTI_Line1;
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
    EXTI_Init(&EXTI_InitStructure);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel=EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel=EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
    NVIC_Init(&NVIC_InitStructure);
}
 
int16_t Encoder_Get(void)
{
    int16_t Temp;
    Temp=Encoder_Count;
    Encoder_Count=0;
    return Temp;
}
 
/*================= 编码器中断服务函数 =================*/
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0)==SET)
    {
        if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==0)
        {
            Encoder_Count--;
        }
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
void EXTI1_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line1)==SET)
    {
        if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==0)
        {
            Encoder_Count++;
        }
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}





但如此一来旋转编码器需要旋转720°舵机才能旋转180°，如果没有显示屏不太好控制，并且旋转两圈也比较难操作，就想着能否对应编码器旋转360°舵机旋转180。

1.确定编码器的分辨率
绝大多数常见机械旋转编码器是 20 格/圈（detents），有的高分辨率型号可能是 24、30、32 格。

如果是 20 格/圈：旋转 360° → 20 次脉冲。

中断服务函数里 Encoder_Count++ / -- 正好就是在数这些脉冲。

2. 计算换算关系
目标是：

20 格（1 圈） → 180°

那么 每格对应 = 180 ÷ 20 = 9°

如果是 24 格：

24 格（1 圈） → 180°

每格 = 180 ÷ 24 = 7.5°

于是在原有主函数上改遍（我这里的旋转编码器是20格的）


(However, in this case, the rotary encoder needs to rotate 720° for the servo to rotate 180°. Without a display screen, it is not easy to control, and rotating two turns is also rather difficult to operate. So, I was thinking about whether the encoder could rotate 360° while the servo could rotate 180°.
1. Determine the resolution of the encoder
The vast majority of common mechanical rotary encoders are 20 detents per turn, while some high-resolution models may have 24, 30, or 32 detents.
If it is 20 grids per circle: Rotate 360° → 20 pulses.
In the interrupt service function, Encoder_Count++ / -- is precisely counting these pulses.
2. Calculate the conversion relationship
The goal is:
20 squares (1 circle) → 180°
Then each grid corresponds to 180 ÷ 20 = 9°
If it is 24 grids:
24 squares (1 circle) → 180°
Each grid = 180 ÷ 24 = 7.5°
So, I modified the original main function (the rotary encoder I have here is 20 bars).

)


#define ENCODER_STEPS_PER_REV 20   // 编码器分辨率（根据实际修改）
#define SERVO_RANGE_DEG       180  // 舵机可动角度范围
 
float Angle = 90;     // 初始角度
float lastAngle = -1;
 
while (1)
{
    int16_t val = Encoder_Get();   // 获取旋转的脉冲数
    if (val != 0)
    {
        Angle += val * (SERVO_RANGE_DEG / ENCODER_STEPS_PER_REV);
        if (Angle < 0) Angle = 0;
        if (Angle > 180) Angle = 180;
        Servo_SetAngle(Angle);
    }
 
    if (Angle != lastAngle)
    {
        OLED_ShowNum(1, 8, (uint16_t)Angle, 3);
        lastAngle = Angle;
    }
}



3.同理，再加上一个舵机
//#define ENCODER_STEPS_PER_REV 20   // 编码器一圈脉冲数
//#define SERVO_RANGE_DEG       180  // 舵机行程角度
 
//#include "Encoder.h"
//#include "OLED.h"
//#include "PWM.h"
//#include "Servo.h"
 
//float Angle1 = 90;  // 舵机1初始角度
//float Angle2 = 90;  // 舵机2初始角度
 
//float lastAngle1 = -1;
//float lastAngle2 = -1;
 
//int main(void)
//{
//    OLED_Init();
//    PWM_Init();
//    Encoder_Init();
 
//    Servo1_SetAngle(Angle1); // 初始角度
//    Servo2_SetAngle(Angle2);
 
//    OLED_ShowString(1,1,"Servo1:");
//    OLED_ShowString(2,1,"Servo2:");
 
//    while(1)
//    {
//        /* ---------- 编码器1控制舵机1 ---------- */
//        int16_t val1 = Encoder1_Get();
//        if(val1 != 0)
//        {
//            Angle1 += val1 * (SERVO_RANGE_DEG / ENCODER_STEPS_PER_REV);
//            if(Angle1 < 0) Angle1 = 0;
//            if(Angle1 > 180) Angle1 = 180;
//            Servo1_SetAngle(Angle1);
//        }
//        if(Angle1 != lastAngle1)
//        {
//            OLED_ShowNum(1, 8, (uint16_t)Angle1, 3);
//            lastAngle1 = Angle1;
//        }
 
//        /* ---------- 编码器2控制舵机2 ---------- */
//        int16_t val2 = Encoder2_Get();
//        if(val2 != 0)
//        {
//            Angle2 += val2 * (SERVO_RANGE_DEG / ENCODER_STEPS_PER_REV);
//            if(Angle2 < 0) Angle2 = 0;
//            if(Angle2 > 180) Angle2 = 180;
//            Servo2_SetAngle(Angle2);
//        }
//        if(Angle2 != lastAngle2)
//        {
//            OLED_ShowNum(2, 8, (uint16_t)Angle2, 3);
//            lastAngle2 = Angle2;
//        }
//    }
//}



此时这样已经差不多了，用此代码可以稳定操控2个有限位180°舵机，但我设想把底座换成360°连续舵机，如此一来更加还原炮台的半球体的运行范围。(At this point, it's almost done. With this code, two finite 180° servos can be stably controlled. However, I plan to replace the base with a 360° continuous servo, which would better restore the hemispherical operating range of the battery.

)
//#include "Encoder.h"
//#include "OLED.h"
//#include "PWM.h"
//#include "Servo.h"
 
//#define ENCODER_STEPS_PER_REV 20   // 编码器一圈脉冲数
//#define SERVO_RANGE_DEG       180  // 舵机2行程角度
 
//float Speed1 = 0;    // 舵机1速度（-1.0 ~ 1.0）
//float Angle2 = 90;   // 舵机2初始角度
 
//float lastSpeed1 = -2;  // 上次速度（用于刷新显示）
//float lastAngle2 = -1;  // 上次角度
 
//int main(void)
//{
//    /* 初始化外设 */
//    OLED_Init();
//    PWM_Init();
//    Encoder_Init();
 
//    /* 初始化舵机 */
//    Servo1_SetSpeed(Speed1);   // 舵机1连续旋转型
//    Servo2_SetAngle(Angle2);   // 舵机2标准角度型
 
//    /* OLED 初始显示 */
//    OLED_ShowString(1, 1, "Servo1Spd:");
//    OLED_ShowString(2, 1, "Servo2Ang:");
//    OLED_ShowNum(1, 11, (int16_t)(Speed1 * 100), 4);  // -100 ~ 100
//    OLED_ShowNum(2, 11, (uint16_t)Angle2, 3);        // 0 ~ 180
 
//    while(1)
//    {
//        /* ---------- 编码器1控制舵机1速度 ---------- */
//        int16_t val1 = Encoder1_Get();
//        if(val1 != 0)
//        {
//            Speed1 += val1 * 0.1f;   // 每次调节 0.1
//            if(Speed1 >  1.0f) Speed1 =  1.0f;
//            if(Speed1 < -1.0f) Speed1 = -1.0f;
//            Servo1_SetSpeed(Speed1);
//        }
//        if(Speed1 != lastSpeed1)
//        {
//            OLED_ShowNum(1, 11, (int16_t)(Speed1 * 100), 4);
//            lastSpeed1 = Speed1;
//        }
 
//        /* ---------- 编码器2控制舵机2角度 ---------- */
//        int16_t val2 = Encoder2_Get();
//        if(val2 != 0)
//        {
//            Angle2 += val2 * (SERVO_RANGE_DEG / ENCODER_STEPS_PER_REV);
//            if(Angle2 < 0) Angle2 = 0;
//            if(Angle2 > 180) Angle2 = 180;
//            Servo2_SetAngle(Angle2);
//        }
//        if(Angle2 != lastAngle2)
//        {
//            OLED_ShowNum(2, 11, (uint16_t)Angle2, 3);
//            lastAngle2 = Angle2;
//        }
//    }
//}





而换成连续旋转舵机与普通舵机又有了很大的区别，无法继续继承普通舵机的简单角度控制。于是采取速度控制，根据编码器旋转的角度，舵机会以某一速度旋转。（比如编码器向右边旋转一格，那么舵机便会以某一速度向右旋转，此时编码器继续旋转一格，舵机加快速度旋转。编码器反向旋转2格，舵机停止旋转。向左同理。）

4.为什么连续舵机会有这样的不同
连续旋转舵机（360° 舵机）
内部结构：直流电机 + 齿轮减速 + 控制电路，但电位器反馈功能被移除或改造。

工作原理：

控制电路只用 PWM 脉宽来决定 旋转方向和速度，而不是角度。

典型规律：

1.5 ms → 停止（中立点）。

1.0 ms → 以最大速度向一个方向转。

2.0 ms → 以最大速度向反方向转。

特点：

不会停在某个角度，只能一直转。

PWM 脉宽表示的是“速度和方向”，而不是角度。

但我想要的是编码器控制舵机同步动态变化，并不在我的预期范围内。

5.如何让连续旋转舵机同步与编码器共同变化
此时我还是想用角度控制连续旋转舵机，但不能用简单的角度控制，而是运用连续角度的控制。

(However, when it comes to continuous rotation servos, there is a significant difference from ordinary servos, and they cannot continue to inherit the simple Angle control of ordinary servos. So speed control is adopted. According to the rotation Angle of the encoder, the servo will rotate at a certain speed. For instance, if the encoder rotates one division to the right, the servo will rotate to the right at a certain speed. At this point, if the encoder continues to rotate one division, the servo will accelerate its rotation speed. When the encoder rotates in the opposite direction by 2 divisions, the servo stops rotating. The same applies to the left."
4. Why are there such differences in continuous servos
Continuous rotating servo (360° servo)
Internal structure: DC motor + gear reduction + control circuit, but the potentiometer feedback function has been removed or modified.
Working principle
The control circuit only uses the PWM pulse width to determine the rotation direction and speed, not the Angle.
Typical rule
1.5ms → Stop (neutral point).
1.0ms → Rotate in one direction at maximum speed.
2.0 ms → Rotate in the opposite direction at maximum speed.
Features
It won't stop at a certain Angle; it can only keep turning.
The PWM pulse width represents "speed and direction", not Angle.
But what I want is for the encoder to control the servo to change synchronously and dynamically, which is not within my expected range.
5. How to make the continuous rotating servo change synchronously with the encoder
At this point, I still want to control the continuously rotating servo by Angle, but I can't use simple Angle control. Instead, I should apply continuous Angle control.


)

#define ENCODER_STEPS_PER_REV 20   // 编码器一圈脉冲数
#define SERVO2_RANGE_DEG     180   // 舵机2行程角度
#define SERVO1_RANGE_DEG     360   // 舵机1模拟行程角度
 
#include "Encoder.h"
#include "OLED.h"
#include "PWM.h"
#include "Servo.h"
#include <math.h>
 
/* 舵机1角度变量（虚拟角度） */
float TargetAngle1 = 0;
float CurrentAngle1 = 0;
 
/* 舵机2实际角度 */
float Angle2 = 90;
 
/* 显示缓存 */
float lastAngle1 = -1;
float lastAngle2 = -1;
 
/* PID参数 */
float Kp = 0.02f;   // 比例系数
float Ki = 0.0005f; // 积分系数
float Kd = 0.01f;   // 微分系数
 
float integral = 0;
float lastError = 0;
 
int main(void)
{
    OLED_Init();
    PWM_Init();
    Encoder_Init();
 
    Servo1_SetSpeed(0);   // 舵机1初始化停止
    Servo2_SetAngle(Angle2);
 
    OLED_ShowString(1,1,"Servo1Ang:");
    OLED_ShowString(2,1,"Servo2Ang:");
 
    while(1)
    {
        /* ---------- 编码器1控制舵机1目标角度 ---------- */
        int16_t val1 = Encoder1_Get();
        if(val1 != 0)
        {
            TargetAngle1 += val1 * (SERVO1_RANGE_DEG / ENCODER_STEPS_PER_REV);
            if(TargetAngle1 < 0) TargetAngle1 += 360;
            if(TargetAngle1 >= 360) TargetAngle1 -= 360;
        }
 
        /* 计算角度误差，确保在 -180~180 之间 */
        float error = TargetAngle1 - CurrentAngle1;
        if(error > 180) error -= 360;
        if(error < -180) error += 360;
 
        /* PID控制器 */
        integral += error;
        float derivative = error - lastError;
        lastError = error;
 
        float output = Kp*error + Ki*integral + Kd*derivative;
 
        /* 限制速度输出 */
        if(output > 1.0f) output = 1.0f;
        if(output < -1.0f) output = -1.0f;
 
        Servo1_SetSpeed(output);
 
        /* 软件模拟角度变化（假设速度对应角度变化） */
        CurrentAngle1 += output * 2;  // 数字 2 = 转速因子，可调
        if(CurrentAngle1 < 0) CurrentAngle1 += 360;
        if(CurrentAngle1 >= 360) CurrentAngle1 -= 360;
 
        if((int)CurrentAngle1 != (int)lastAngle1)
        {
            OLED_ShowNum(1, 11, (uint16_t)CurrentAngle1, 3);
            lastAngle1 = CurrentAngle1;
        }
 
        /* ---------- 编码器2控制舵机2角度 ---------- */
        int16_t val2 = Encoder2_Get();
        if(val2 != 0)
        {
            Angle2 += val2 * (SERVO2_RANGE_DEG / ENCODER_STEPS_PER_REV);
            if(Angle2 < 0) Angle2 = 0;
            if(Angle2 > 180) Angle2 = 180;
            Servo2_SetAngle(Angle2);
        }
        if(Angle2 != lastAngle2)
        {
            OLED_ShowNum(2, 11, (uint16_t)Angle2, 3);
            lastAngle2 = Angle2;
        }
    }
}
以上主函数可看到整体的控制舵机形式，舵机1以普通角度控制，舵机2用模拟角度控制。最终能单个STM32丝滑动态控制两个舵机。
(The above main function shows the overall control form of the servo. Servo 1 is controlled at a normal Angle, while servo 2 is controlled at an analog Angle. Ultimately, a single STM32 can smoothly and dynamically control two servos.

)


6.最终完整代码,详细见文件
main.c核心思路是：

用 GPIOB8 (SCL), GPIOB9 (SDA) 模拟 I2C

通过 OLED_WriteCommand / OLED_WriteData 写命令或数据

提供了 OLED_ShowChar / OLED_ShowString / OLED_ShowNum 等高级显示函数

7.为什么需要“软件虚拟角度”？
依据之前解释，连续旋转舵机本质上不是标准 0°~180° 舵机，而是一个“带电机 + 驱动”的模块，它只能无限转圈。

编码器能读到 当前绝对角度（比如 0~359°）。

但如果舵机转了一圈，编码器会从 359° → 0° 突然跳变。

这样就有两个问题：

角度会回绕，比如想要 720°，编码器永远只能给 0~359°。

PID 控制需要“连续的目标 - 实际”，不能用跳变的角度。

所以，引入一个 软件虚拟角度（VirtualAngle），把编码器的跳变展开，得到一个连续角度值。

软件虚拟角度的计算过程
设编码器能输出 CurrentAngle (0~360)，我们定义 VirtualAngle：

第一次读取，记下 LastAngle = CurrentAngle。

每次更新时：
int delta = CurrentAngle - LastAngle;
if(delta > 180) delta -= 360;   // 跨过0点的情况（逆时针）
if(delta < -180) delta += 360;  // 跨过0点的情况（顺时针）
VirtualAngle += delta;
LastAngle = CurrentAngle;
(6. The final complete code, for details, please refer to the file
The core idea of main.c is:
I2C was simulated using GPIOB8 (SCL) and GPIOB9 (SDA)
Write commands or data through OLED_WriteCommand/OLED_WriteData
It provides advanced display functions such as OLED_ShowChar/OLED_ShowString/OLED_ShowNum
7. Why is "Software virtual Angle" needed?
According to previous explanations, a continuously rotating servo is essentially not a standard 0° to 180° servo, but rather a "motor + drive" module that can only rotate infinitely.
The encoder can read the current absolute Angle (for example, 0 to 359°).
However, if the servo rotates one full circle, the encoder will suddenly jump from 359° to 0°.
This brings up two problems:
The Angle will be rewound. For instance, if you want 720°, the encoder can only provide 0 to 359°.
PID control requires "continuous target-reality" and cannot use a jumping Angle.
Therefore, a software virtual Angle (VirtualAngle) is introduced to expand the jump of the encoder and obtain a continuous Angle value.
The calculation process of the software's virtual Angle
Assuming the encoder can output CurrentAngle (0 to 360), we define VirtualAngle:
For the first read, note "LastAngle = CurrentAngle".
Each time it is updated:
int delta = CurrentAngle - LastAngle;
if(delta > 180) delta -= 360; //


)

这样 VirtualAngle 就能变成 …, -360, -180, 0, 180, 360, 720 … 这样的连续值。

例如：

你把旋钮转了两圈，编码器报 0~360；

软件虚拟角度就能输出 0 → 360 → 720；

这就能正确表示“转了两圈”。

8.PID 控制为什么需要虚拟角度？
PID 的基本公式是：
error = TargetAngle - VirtualAngle
Output = Kp*error + Ki*∫error + Kd*d(error)/dt

如果用原始编码器角度（0~360°），会出现问题：

目标 350°，实际 10°，误差应该是 -20°，但直接相减得到 340°，舵机会绕远路转一圈。

或者跨过 0° 时误差突变，PID 输出会乱跳。

用虚拟角度，误差始终是“最近的一条路径”，PID 才能稳定收敛。

PID 参数怎么选？
PID 有三个参数：

Kp（比例系数）：放大误差。

Kp 太小 → 舵机反应迟钝。

Kp 太大 → 抖动、来回震荡。

Ki（积分系数）：消除静差。

Ki 太小 → 最终可能差一点到不了目标。

Ki 太大 → 会积累过多，系统“超调”，甚至振荡。

在舵机控制里，Ki 通常 很小甚至不用。

Kd（微分系数）：抑制快速变化。

Kd 太小 → 转到目标时会冲过头。

Kd 太大 → 动作僵硬甚至震荡。

PID 调参过程
调试时可以按这个顺序：

先调 P

把 Ki、Kd 都设 0；

慢慢加大 Kp，直到舵机能跟随目标，但开始出现轻微振荡 → 稍微调小一点，就是合适的 Kp。

再加 D

在有 Kp 的基础上，加一点 Kd；

目的是让舵机接近目标时不要冲过头。

调到舵机“快到目标时刹住脚”。

最后考虑 I

如果发现舵机能到目标附近，但总是差一点（比如 5°~10°），再加小量 Ki。

加完 Ki 后要观察有没有明显震荡。

举个实际例子
假设目标角度是 720°：

旋钮转两圈，虚拟角度变成 720°；

PID 算误差： error = 720 - VirtualAngle；

舵机就会转动直到 VirtualAngle ≈ 720。

参数举例（在你这个项目里）：

Kp = 2.0

Ki = 0.01

Kd = 0.5

(In this way, VirtualAngle can become... -360, -180, 0, 180, 360, 720... Such continuous values.
You turned the knob twice, and the encoder reported 0 to 360.
The software's virtual Angle can output 0 → 360 → 720.
This can correctly indicate "two turns".
8. Why does PID control require virtual angles?
The basic formula of PID is:
error = TargetAngle - VirtualAngle
Output = Kp*error + Ki*∫error + Kd*d(error)/dt
If the original encoder Angle (0~360°) is used, problems will occur:
The target is 350°, but the actual temperature is 10°. The error should be -20°, but directly subtracting it gives 340°. The servo will take a long detour and make a full circle.
Or when crossing 0°, the error will suddenly change, and the PID output will jump randomly.
From a virtual perspective, the error is always the "nearest path" for the PID to converge stably.
How to select PID parameters?
PID has three parameters:
Kp (Proportional Coefficient) : Amplification error.
The Kp is too small → the servo responds slowly.
If Kp is too large, it will shake and oscillate back and forth.
Ki (integral coefficient) : Eliminate static aberration.
Ki is too small → It might just fall short of reaching the goal in the end.
If Ki is too large, it will accumulate excessively, causing the system to "overshoot" and even oscillate.
In servo control, Ki is usually very small or even not used.
Kd (Differential coefficient) : Suppresses rapid changes.
Kd is too small → it will overcharge when turning to the target.
A too large Kd → stiff movements or even oscillations.
PID parameter tuning process
When debugging, you can follow this sequence


)
