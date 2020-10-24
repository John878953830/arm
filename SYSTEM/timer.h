#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
#include "stm32f4xx_tim.h"
#include "system_stm32f4xx.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx.h"
#include "main.h"

void TIM2_init(u16 auto_data, u16 fractional);
void TIM_init(u16 auto_data, u16 fractional, u8 timer_id);

#endif
