#ifndef __S_SYSTICK_H
#define __S_SYSTICK_H


#include "stm32f4xx_hal.h"
#include "stdint.h"

typedef struct 
{
    uint32_t period;
    void (*fun)(void);
    int16_t n;
    uint32_t TargetTick;
}Period_t;


void Systick_init(void);//滴答定时器初始化

uint32_t Get_Systick(void);//获取系统运行毫秒数

// void SysTick_Handler(void);//滴答定时器中断

void delay_ms(uint32_t ms);//延时

void do_sth_period(Period_t* pstruct);//定时操作,放在主循环就能跑



#endif