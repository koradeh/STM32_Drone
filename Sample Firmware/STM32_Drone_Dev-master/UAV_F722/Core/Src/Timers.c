#include "Timers.h"

uint16_t TIM6_GetTick(void)
{
	return __HAL_TIM_GET_COUNTER(&MicrosecondTimer); //1 tick is 1 microsecond
}


int GetMicroseconds(uint16_t current_tick, uint16_t prev_tick)
{

    if (prev_tick <= current_tick)
    {
    	return (current_tick - prev_tick); // Time difference in microseconds
    }
    else
    {
    	return ((TIM6_CounterResetValue - prev_tick + 1) + current_tick);// Handle counter overflow
    }

}

void Delay_ms(uint32_t ms)
{
    uint32_t target_us = ms * 1000;
    uint16_t start_tick = TIM6_GetTick();
    uint32_t elapsed_us = 0;

    while (elapsed_us < target_us)
    {
        uint16_t current_tick = TIM6_GetTick();
        elapsed_us += GetMicroseconds(current_tick, start_tick);
        start_tick = current_tick;
    }
}

//		ct= TIM6_GetTick();
//		dt =((float)GetMicroseconds(ct, pt))/1000000.0; //calculate dt in us to s
//		pt = ct;
