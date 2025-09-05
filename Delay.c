#include "stm32f10x.h"
#include <stdint.h>
 
/* 微秒级延时函数 */
void Delay_us(uint32_t xus)
{
    SysTick->LOAD = 72 * xus;             // SysTick 重装值 = us × 72 (72MHz 主频)
    SysTick->VAL = 0x00;                  // 清空当前计数
    SysTick->CTRL = 0x00000005;           // 使用 HCLK，启动计数器
    while(!(SysTick->CTRL & 0x00010000)); // 等待计数到 0
    SysTick->CTRL = 0x00000004;           // 关闭 SysTick
}
 
/* 毫秒延时函数 */
void Delay_ms(uint32_t xms)
{
    while(xms--)
    {
        Delay_us(1000);   // 1ms = 1000us
    }
}
 
/* 秒级延时函数 */
void Delay_s(uint32_t xs)
{
    while(xs--)
    {
        Delay_ms(1000);   // 1s = 1000ms
    }
}
