#include "S_SYSTICK.h"

void Systick_init(void)
{
    if (SysTick_Config(SystemCoreClock / 1000))
        ;
    NVIC_SetPriority(SysTick_IRQn, 0);
}

uint32_t Get_Systick(void)
{
    return HAL_GetTick();
}
//延时
void delay_ms(uint32_t ms)
{
    uint32_t timeout = Get_Systick() + ms;
    while (Get_Systick() - timeout > UINT32_MAX / 2)
        ;
}
//定时
void do_sth_period(Period_t *pstruct)
{
    uint32_t CurTick = Get_Systick();
    if (CurTick - pstruct->TargetTick > UINT32_MAX / 2)//判断条件还挺有意思的
        return;
    pstruct->fun();
    pstruct->TargetTick = CurTick + pstruct->period;
}