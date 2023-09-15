#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#define digitalRead(x)  gpio_get(x.port, x.pin)

void buttons_setup(void);

typedef struct{
    uint32_t rcc;
    uint32_t port;
    uint32_t pin;
} button_t;

extern button_t B23;
extern button_t B24;

#ifdef __cplusplus
}
#endif
