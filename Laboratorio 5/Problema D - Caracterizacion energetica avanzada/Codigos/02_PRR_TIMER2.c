/*
 * LABORATORIO 5 - PROBLEMA D
 * Prueba individual del consumo de Timer2 mediante PRR
 * ATmega328P / Arduino Uno / avr-gcc
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <stdint.h>

/* 0U = Timer2 habilitado; 1U = Timer2 deshabilitado mediante PRR. */
#define TIMER2_APAGADO 1U

static void apagar_perifericos_comunes(void)
{
    ADCSRA = 0U;
    PRR |= _BV(PRADC) |
           _BV(PRUSART0) |
           _BV(PRSPI) |
           _BV(PRTIM0) |
           _BV(PRTIM1) |
           _BV(PRTWI);
    DIDR0 |= _BV(ADC0D);
}

static void timer2_habilitar(void)
{
    PRR &= (uint8_t)~_BV(PRTIM2);
    ASSR = 0U;
    TCCR2A = 0U;
    TCNT2 = 0U;
    TIMSK2 = 0U;

    /* Reloj de 16 MHz sin prescaler. */
    TCCR2B = _BV(CS20);
}

static void timer2_deshabilitar(void)
{
    TCCR2B = 0U;
    TIMSK2 = 0U;
    PRR |= _BV(PRTIM2);
}

int main(void)
{
    DDRB &= (uint8_t)~_BV(PB5);
    PORTB &= (uint8_t)~_BV(PB5);

    apagar_perifericos_comunes();

    if (TIMER2_APAGADO == 1U) {
        timer2_deshabilitar();
    } else {
        timer2_habilitar();
    }

    while (1) {
        __asm__ __volatile__("nop");
    }

    return 0;
}

