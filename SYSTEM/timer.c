#include "timer.h"
void TIM2_init(u16 auto_data, u16 fractional)
{

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = auto_data;
    TIM_TimeBaseInitStructure.TIM_Prescaler = fractional;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    return;
}
void TIM_init(u16 auto_data, u16 fractional, u8 timer_id)
{
    switch (timer_id)
    {
    case 1:
    {
        break;
    }
    case 2:
    {
        TIM2_init(auto_data, fractional);
        break;
    }
    case 3:
    {
        break;
    }
    case 4:
    {
        break;
    }
    default:
    {
        break;
    }
    }
    return;
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        if (total_motor_number != 4)
        {
//todo, lost motor
#ifdef DEBUG_OUTPUT
            printf("lost motor , check motor number or link\n");
#endif
        }
        else
        {
            int i = 0;
            for (i = 1; i < 5; i++)
            {
                getPosition(i, Unblock);
            }
        }
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}
