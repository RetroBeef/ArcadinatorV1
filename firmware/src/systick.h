#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
void systick_setup(void);
void delay_1ms(uint32_t count);
uint32_t secs(void);
uint32_t millis(void);

//arduino compat
#define delay delay_1ms

#ifdef __cplusplus
}
#endif
