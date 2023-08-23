#include "buttons.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
//RCC_GPIOA
//GPIOA
//GPIO0
button_t B01 = {
    RCC_GPIOB,
    GPIOB,
    GPIO12
};
button_t B02 = {
    RCC_GPIOB,
    GPIOB,
    GPIO13
};
button_t B03 = {
    RCC_GPIOB,
    GPIOB,
    GPIO14
};
button_t B04 = {
    RCC_GPIOB,
    GPIOB,
    GPIO15
};
button_t B05 = {
    RCC_GPIOA,
    GPIOA,
    GPIO8
};
button_t B06 = {
    RCC_GPIOA,
    GPIOA,
    GPIO9
};
button_t B07 = {
    RCC_GPIOA,
    GPIOA,
    GPIO10
};
button_t B08 = {
    RCC_GPIOA,
    GPIOA,
    GPIO15
};
button_t B09 = {
    RCC_GPIOB,
    GPIOB,
    GPIO3
};
button_t B10 = {
    RCC_GPIOB,
    GPIOB,
    GPIO4
};
button_t B11 = {
    RCC_GPIOB,
    GPIOB,
    GPIO5
};
button_t B12 = {
    RCC_GPIOB,
    GPIOB,
    GPIO6
};
button_t B13 = {
    RCC_GPIOB,
    GPIOB,
    GPIO11
};
button_t B14 = {
    RCC_GPIOB,
    GPIOB,
    GPIO10
};
button_t B15 = {
    RCC_GPIOB,
    GPIOB,
    GPIO1
};
button_t B16 = {
    RCC_GPIOA,
    GPIOA,
    GPIO3
};
button_t B17 = {
    RCC_GPIOA,
    GPIOA,
    GPIO2
};
button_t B18 = {
    RCC_GPIOA,
    GPIOA,
    GPIO1
};
button_t B19 = {
    RCC_GPIOC,
    GPIOC,
    GPIO15
};
button_t B20 = {
    RCC_GPIOC,
    GPIOC,
    GPIO14
};
button_t B21 = {
    RCC_GPIOC,
    GPIOC,
    GPIO13
};
button_t B22 = {
    RCC_GPIOB,
    GPIOB,
    GPIO7
};
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
    button_setup_input(B01);
    button_setup_input(B02);
    button_setup_input(B03);
    button_setup_input(B04);
    button_setup_input(B05);
    button_setup_input(B06);
    button_setup_input(B07);
    button_setup_input(B08);
    button_setup_input(B09);
    button_setup_input(B10);
    button_setup_input(B11);
    button_setup_input(B12);
    button_setup_input(B13);
    button_setup_input(B14);
    button_setup_input(B15);
    button_setup_input(B16);
    button_setup_input(B17);
    button_setup_input(B18);
    button_setup_input(B19);
    button_setup_input(B20);
    button_setup_input(B21);
    button_setup_input(B22);
    button_setup_input(B23);
    button_setup_input(B24);
}

