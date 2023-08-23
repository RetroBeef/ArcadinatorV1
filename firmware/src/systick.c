#include "systick.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

volatile uint32_t _delay;

void systick_setup(void){
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(8999);
	systick_interrupt_enable();
	systick_counter_enable();
}

void delay_1ms(uint32_t count) {
	_delay = count;

	while (0U != _delay) {
	}
}

static volatile uint32_t _millis = 0;
static volatile uint32_t _secs = 0;

uint32_t millis(void) {
	return _millis;
}

uint32_t secs(void){
	return _secs;
}

void sys_tick_handler(void) {
	if (0U != _delay) {
		_delay--;
	}
    _millis++;
	if(_millis%1000==0){
	    _secs++;
	}
}
