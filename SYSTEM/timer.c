#include "timer.h"
char timeout_flag = 0;
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

void TIM3_init(u16 auto_data, u16 fractional)
{

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = auto_data;
    TIM_TimeBaseInitStructure.TIM_Prescaler = fractional;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    //TIM_Cmd(TIM3, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    return;
}

void TIM4_init(u16 auto_data, u16 fractional)
{

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = auto_data;
    TIM_TimeBaseInitStructure.TIM_Prescaler = fractional;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    //TIM_Cmd(TIM4, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    return;
}

void TIM13_init(u16 auto_data, u16 fractional)
{

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = auto_data;
    TIM_TimeBaseInitStructure.TIM_Prescaler = fractional;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM13, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM13, TIM_IT_Update, ENABLE);
    //TIM_Cmd(TIM13, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    return;
}

void TIM14_init(u16 auto_data, u16 fractional)
{

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = auto_data;
    TIM_TimeBaseInitStructure.TIM_Prescaler = fractional;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM14, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
    //TIM_Cmd(TIM14, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM8_TRG_COM_TIM14_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    return;
}

void TIM6_init(u16 auto_data, u16 fractional)
{

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = auto_data;
    TIM_TimeBaseInitStructure.TIM_Prescaler = fractional;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    //TIM_Cmd(TIM14, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
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
        TIM3_init(auto_data, fractional);
        break;
    }
    case 4:
    {
        TIM4_init(auto_data, fractional);
        break;
    }
    case 5:
    {
        break;
    }
    case 6:
    {
        TIM6_init(auto_data, fractional);
        break;
    }
    case 7:
    {
        break;
    }
    case 8:
    {
        break;
    }
    case 9:
    {
        break;
    }
    case 10:
    {
        break;
    }
    case 11:
    {
        break;
    }
    case 12:
    {
        break;
    }
    case 13:
    {
        TIM13_init(auto_data, fractional);
        break;
    }
    case 14:
    {
        TIM14_init(auto_data, fractional);
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
            if_need_lookup_monitor = 1;
#ifdef DEBUG_OUTPUT
            printf("lost motor , check motor number or link\n");
#endif
        }
        else
        {
            if_need_lookup_monitor = 0;
            timeout_flag = 1;
        }
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {
        SCA_Handler_t *pSCA = NULL;
        pSCA = getInstance(1);
        if (pSCA != NULL)
        {
            if (__fabs(mp[1].target - pSCA->Position_Real) > (mp[1].step/2))//
            {
                motor_act(1, mp[1].step, mp[1].target - pSCA->Position_Real > 0 ? 0 : 1);
                tim3_counter++;
            }
            else
            {
                TIM_Cmd(TIM3, DISABLE);
							  setPosition(1, mp[1].target);
            }
        }
    }
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
    {
        SCA_Handler_t *pSCA = NULL;
        pSCA = getInstance(2);
        if (pSCA != NULL)
        {
            if (__fabs(mp[2].target - pSCA->Position_Real) > (mp[2].step/2))//
            {
                motor_act(2, mp[2].step, mp[2].target - pSCA->Position_Real > 0 ? 0 : 1);
                tim4_counter++;
            }
            else
            {
                TIM_Cmd(TIM4, DISABLE);
							  setPosition(2, mp[2].target);
            }
        }
    }
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}

void TIM6_DAC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)
    {
        //if_error = update_status();
    }
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
}

void TIM8_UP_TIM13_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM13, TIM_IT_Update) == SET)
    {
        SCA_Handler_t *pSCA = NULL;
        pSCA = getInstance(3);
        if (pSCA != NULL)
        {
            if (__fabs(mp[3].target - pSCA->Position_Real) > (mp[3].step/2))//
            {
                motor_act(3, mp[3].step, mp[3].target - pSCA->Position_Real > 0 ? 0 : 1);
                tim13_counter++;
            }
            else
            {
                TIM_Cmd(TIM13, DISABLE);
							  setPosition(3, mp[3].target);
            }
        }
    }
    TIM_ClearITPendingBit(TIM13, TIM_IT_Update);
}

void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM14, TIM_IT_Update) == SET)
    {
        SCA_Handler_t *pSCA = NULL;
        pSCA = getInstance(4);
        if (pSCA != NULL)
        {
            if (__fabs(mp[4].target - pSCA->Position_Real) > (mp[4].step/2))//
            {
                motor_act(4, mp[4].step, mp[4].target - pSCA->Position_Real > 0 ? 0 : 1);
                tim14_counter++;
            }
            else
            {
                TIM_Cmd(TIM14, DISABLE);
							  setPosition(4, mp[4].target);
            }
        }
    }
    TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
}
