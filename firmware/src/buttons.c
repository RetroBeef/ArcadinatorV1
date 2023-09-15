#include "buttons.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

button_t B23 = {
    RCC_GPIOB,
    GPIOB,
    GPIO8
};
button_t B24 = {
    RCC_GPIOB,
    GPIOB,
    GPIO9
};

static void button_setup_input(button_t button){
    //rcc_periph_clock_enable(button.rcc);
    gpio_set_mode(button.port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, button.pin);
    gpio_set(button.port, button.pin);
}

void buttons_setup(void){
    button_setup_input(B23);
    button_setup_input(B24);
}

