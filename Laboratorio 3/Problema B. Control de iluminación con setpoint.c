/*
 * Problema B: control de iluminacion con persiana motorizada.
 * Bloque 1: adquisicion de sensores y diagnostico por UART.
 * ATmega328P, 16 MHz. Compilar como C con optimizacion -Os.
 * LDR en A0, potenciometro de posicion en A1.
 * L298: ENA=D9, IN1=D4, IN2=D5. Motor deshabilitado en esta etapa.
 * Monitor Serie: 9600 baudios, 8N1.
 * Lecturas cada 50 ms; diagnostico cada 100 ms.
 */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdint.h>

#define LDR_ADC_CHANNEL 0u
#define POT_ADC_CHANNEL 1u
#define MOTOR_IN1 PD4
#define MOTOR_IN2 PD5
#define MOTOR_PWM PB1

static volatile uint32_t reloj_ms;
static uint16_t lectura_ldr;
static uint16_t posicion_persiana;


ISR(TIMER0_COMPA_vect)
{
    reloj_ms++;
}

static uint32_t tiempo_actual_ms(void)
{
    uint32_t ahora;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        ahora = reloj_ms;
    }

    return ahora;
}

static void temporizador_iniciar(void)
{
    TCCR0A = _BV(WGM01);
    TCCR0B = 0;

    TCNT0 = 0;

    OCR0A = 249;

    TIFR0 = _BV(OCF0A);

    TIMSK0 = _BV(OCIE0A);

    TCCR0B =
        _BV(CS01)
        |
        _BV(CS00);
}

static void adc_iniciar(void)
{
    ADMUX = _BV(REFS0);

    ADCSRA =
        _BV(ADEN)
        |
        _BV(ADPS2)
        |
        _BV(ADPS1)
        |
        _BV(ADPS0);

    DIDR0 =
        _BV(ADC0D)
        |
        _BV(ADC1D);
}

static uint16_t adc_leer(uint8_t canal)
{
    ADMUX =
        (uint8_t)(
            _BV(REFS0)
            |
            (canal & 0x0Fu)
        );

    ADCSRA |= _BV(ADSC);

    while (ADCSRA & _BV(ADSC))
    {
    }

    return ADC;
}

static uint16_t adc_promedio(uint8_t canal)
{
    uint32_t suma = 0;

    (void)adc_leer(canal);

    for (uint8_t i = 0; i < 8; i++)
    {
        suma += adc_leer(canal);
    }

    return (uint16_t)(suma / 8u);
}

static void uart_iniciar(void)
{
    uint16_t divisor =
        (uint16_t)(
            F_CPU / (16UL * 9600UL) - 1UL
        );

    UCSR0A = 0;

    UBRR0H =
        (uint8_t)(divisor >> 8);

    UBRR0L =
        (uint8_t)divisor;

    UCSR0C =
        _BV(UCSZ01)
        |
        _BV(UCSZ00);

    UCSR0B = _BV(TXEN0); /* Solo transmision en el bloque 1. */
}

static void uart_enviar_caracter(char caracter)
{
    while (!(UCSR0A & _BV(UDRE0)))
    {
    }

    UDR0 = (uint8_t)caracter;
}

static void uart_enviar_texto(const char *texto)
{
    while (*texto)
    {
        uart_enviar_caracter(*texto++);
    }
}

static void uart_enviar_numero(uint16_t numero)
{
    char digitos[5];

    uint8_t cantidad = 0;

    do
    {
        digitos[cantidad++] =
            (char)(
                '0'
                +
                (numero % 10u)
            );

        numero /= 10u;

    }
    while (
        numero > 0
        &&
        cantidad < sizeof(digitos)
    );

    while (cantidad > 0)
    {
        uart_enviar_caracter(
            digitos[--cantidad]
        );
    }
}

/* Permite comprobar ambos sensores antes de incorporar el control. */
static void enviar_diagnostico(void)
{
    uart_enviar_texto("LDR=");
    uart_enviar_numero(lectura_ldr);
    uart_enviar_texto(",POS=");
    uart_enviar_numero(posicion_persiana);
    uart_enviar_texto("\r\n");
}

int main(void)
{
    uint32_t siguiente_lectura = 0;
    uint32_t siguiente_diagnostico = 0;

    DDRC &= (uint8_t)~(_BV(PC0) | _BV(PC1));
    PORTC &= (uint8_t)~(_BV(PC0) | _BV(PC1));

    /* Mantiene ENA y las dos entradas del puente en nivel bajo. */
    TCCR1A = 0;
    TCCR1B = 0;
    PORTB &= (uint8_t)~_BV(MOTOR_PWM);
    DDRB |= _BV(MOTOR_PWM);
    PORTD &= (uint8_t)~(_BV(MOTOR_IN1) | _BV(MOTOR_IN2));
    DDRD |= _BV(MOTOR_IN1) | _BV(MOTOR_IN2);

    adc_iniciar();
    uart_iniciar();
    temporizador_iniciar();
    sei();

    uart_enviar_texto("Lectura de LDR y posicion lista. Motor deshabilitado.\r\n");

    for (;;)
    {
        uint32_t ahora = tiempo_actual_ms();

        if ((int32_t)(ahora - siguiente_lectura) >= 0)
        {
            lectura_ldr = adc_promedio(LDR_ADC_CHANNEL);
            posicion_persiana = adc_promedio(POT_ADC_CHANNEL);
            siguiente_lectura = ahora + 50u;
        }

        if ((int32_t)(ahora - siguiente_diagnostico) >= 0)
        {
            enviar_diagnostico();
            siguiente_diagnostico = ahora + 100u;
        }
    }
}
