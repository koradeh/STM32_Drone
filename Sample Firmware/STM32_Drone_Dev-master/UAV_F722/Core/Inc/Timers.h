#include "main.h"
#include "tim.h"

#define MicrosecondTimer htim6
#define TIM6_CounterResetValue 0xFFFF

uint16_t TIM6_GetTick(void);
int GetMicroseconds(uint16_t current_tick, uint16_t prev_tick);
void Delay_ms(uint32_t ms);
