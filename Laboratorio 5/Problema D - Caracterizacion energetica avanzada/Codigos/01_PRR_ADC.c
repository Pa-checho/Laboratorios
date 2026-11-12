/*
 * LABORATORIO 5 - PROBLEMA D
 * Prueba individual del consumo del ADC mediante PRR
 * ATmega328P / Arduino Uno / avr-gcc
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <stdint.h>

/* 0U = ADC habilitado; 1U = ADC deshabilitado mediante PRR. */
#define ADC_APAGADO 1U

static void adc_habilitar(void)
{
    PRR &= (uint8_t)~_BV(PRADC);
    ADMUX = _BV(REFS0);
    ADCSRB = 0U;
    ADCSRA = _BV(ADEN)  |
              _BV(ADATE) |
              _BV(ADPS2) |
              _BV(ADPS1) |
              _BV(ADPS0);
    ADCSRA |= _BV(ADSC);
}

static void adc_deshabilitar(void)
{
    ADCSRA = 0U;
    PRR |= _BV(PRADC);
}

int main(void)
{
    DDRB &= (uint8_t)~_BV(PB5);
    PORTB &= (uint8_t)~_BV(PB5);

    /* Debe permanecer igual en ambas condiciones. */
    DIDR0 |= _BV(ADC0D);

    if (ADC_APAGADO == 1U) {
        adc_deshabilitar();
    } else {
        adc_habilitar();
    }

    while (1) {
        __asm__ __volatile__("nop");
    }

    return 0;
}

