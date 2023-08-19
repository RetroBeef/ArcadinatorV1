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

extern button_t B01;
extern button_t B02;
extern button_t B03;
extern button_t B04;
extern button_t B05;
extern button_t B06;
extern button_t B07;
extern button_t B08;
extern button_t B09;
extern button_t B10;
extern button_t B11;
extern button_t B12;
extern button_t B13;
extern button_t B14;
extern button_t B15;
extern button_t B16;
extern button_t B17;
extern button_t B18;
extern button_t B19;
extern button_t B20;
extern button_t B21;
extern button_t B22;
extern button_t B23;
extern button_t B24;

#ifdef __cplusplus
}
#endif
