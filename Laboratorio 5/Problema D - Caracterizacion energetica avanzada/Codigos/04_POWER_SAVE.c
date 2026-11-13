/*
 * LABORATORIO 5 - PROBLEMA D
 * CPU activa frente a modo Power-Save
 * ATmega328P / Arduino Uno / avr-gcc
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdint.h>

/* 0U = CPU activa; 1U = modo Power-Save. */
#define POWER_SAVE_ACTIVO 1U

static void apagar_perifericos(void)
{
    ADCSRA = 0U;

    PRR = _BV(PRADC) |
          _BV(PRUSART0) |
          _BV(PRSPI) |
          _BV(PRTIM0) |
          _BV(PRTIM1) |
          _BV(PRTIM2) |
          _BV(PRTWI);

    ACSR |= _BV(ACD);

    DIDR0 = _BV(ADC0D) |
            _BV(ADC1D) |
            _BV(ADC2D) |
            _BV(ADC3D) |
            _BV(ADC4D) |
            _BV(ADC5D);

    DIDR1 = _BV(AIN0D) | _BV(AIN1D);
}

static void entrar_power_save(void)
{
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);

    while (1) {
        cli();
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();
    }
}

int main(void)
{
    cli();
    MCUSR &= (uint8_t)~_BV(WDRF);
    wdt_disable();

    DDRB &= (uint8_t)~_BV(PB5);
    PORTB &= (uint8_t)~_BV(PB5);

    apagar_perifericos();

    if (POWER_SAVE_ACTIVO == 1U) {
        entrar_power_save();
    }

    sei();

    while (1) {
        __asm__ __volatile__("nop");
    }

    return 0;
}

