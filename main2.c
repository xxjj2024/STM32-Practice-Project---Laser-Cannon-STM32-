#include "stm32f10x.h"
#include "OLED.h"
#include "PWM.h"
#include "ServoCtrl.h"
#include "Bluetooth.h"
 
int main(void)
{
    OLED_Init();
    PWM_Init();
    Bluetooth_Init(9600);
    ServoCtrl_Init();
 
    while(1)
    {
        ServoCtrl_Update();
    }
}
