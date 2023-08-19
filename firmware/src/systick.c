#include "systick.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

volatile uint32_t _delay;

void systick_setup(void){
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(999999);//SysTick interrupt every N clock pulses: set reload to N-1, should be 1us
	systick_interrupt_enable();
	systick_counter_enable();
}

void delay_1ms(uint32_t count) {
	_delay = count * 1000;

	while (0U != _delay) {
	}
}

void delay_1us(uint32_t count) {
	_delay = count;

	while (0U != _delay) {
	}
}

static uint32_t _micros = 0;
static uint32_t _millis = 0;
static uint32_t _secs = 0;

uint32_t millis(void) {
	return _millis;
}

uint32_t micros(void) {
	return _micros;
}

uint32_t secs(void){
	return _secs;
}

void sys_tick_handler(void) {
	if (0U != _delay) {
		_delay--;
	}
	_micros++;
	if(_micros % 1000==0){
		_millis++;
		if(_millis%1000==0){
			_secs++;
		}
	}
}
