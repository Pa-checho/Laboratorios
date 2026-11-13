/*
 * LABORATORIO 5 - PROBLEMA D
 * CPU activa frente a ADC Noise Reduction
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

/* 0U = CPU activa durante conversiones; 1U = ADC Noise Reduction. */
#define ADC_NOISE_REDUCTION 1U

ISR(ADC_vect)
{
    /* La finalización de la conversión despierta a la CPU. */
}

static void configurar_perifericos(void)
{
    /* Mantiene activo únicamente el ADC. */
    PRR = _BV(PRUSART0) |
          _BV(PRSPI) |
          _BV(PRTIM0) |
          _BV(PRTIM1) |
          _BV(PRTIM2) |
          _BV(PRTWI);

    ADMUX = _BV(REFS0);             /* AVCC y ADC0. */
    ADCSRB = 0U;
    DIDR0 = _BV(ADC0D);
    ADCSRA = _BV(ADEN) |
              _BV(ADIE) |
              _BV(ADPS2) |
              _BV(ADPS1) |
              _BV(ADPS0);
}

static void medir_cpu_activa(void)
{
    cli();

    while (1) {
        ADCSRA |= _BV(ADIF);
        ADCSRA |= _BV(ADSC);

        while ((ADCSRA & _BV(ADIF)) == 0U) {
            __asm__ __volatile__("nop");
        }
    }
}

static void medir_adc_noise_reduction(void)
{
    set_sleep_mode(SLEEP_MODE_ADC);

    while (1) {
        cli();
        ADCSRA |= _BV(ADIF);
        ADCSRA |= _BV(ADSC);
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

    configurar_perifericos();

    if (ADC_NOISE_REDUCTION == 1U) {
        medir_adc_noise_reduction();
    } else {
        medir_cpu_activa();
    }

    return 0;
}

