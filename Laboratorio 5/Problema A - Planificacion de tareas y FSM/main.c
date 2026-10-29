/*
 * Laboratorio 5 - Problema A
 * Primer avance: planificacion basica de tareas.
 * ATmega328P a 16 MHz.
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#define LED_PIN       PB5    /* D13 */
#define SOUNDER_PIN   PB3    /* D11 */
#define BOTON_PIN     PD2    /* D2  */

#define PERIODO_LED_MS       200U
#define DURACION_SONIDO_MS   500U
#define ANTI_REBOTE_MS       50UL

/* Estas variables cambian dentro de las interrupciones. */
static volatile uint32_t tiempo_ms = 0UL;
static volatile uint16_t contador_led_ms = 0U;
static volatile uint16_t tiempo_sonido_ms = 0U;

static volatile uint8_t bandera_led = 0U;
static volatile uint8_t bandera_adc = 0U;
static volatile uint8_t bandera_boton = 0U;

static uint16_t valor_adc = 0U;
static uint32_t ultimo_boton_ms = 0UL;

/* Timer0 genera una interrupcion cada 1 ms. */
ISR(TIMER0_COMPA_vect)
{
    tiempo_ms++;

    contador_led_ms++;
    if (contador_led_ms >= PERIODO_LED_MS) {
        contador_led_ms = 0U;
        bandera_led = 1U;
    }

    /* Cambiar la salida cada 1 ms produce un tono de 500 Hz. */
    if (tiempo_sonido_ms > 0U) {
        PORTB ^= (1U << SOUNDER_PIN);
        tiempo_sonido_ms--;
    } else {
        PORTB &= (uint8_t)~(1U << SOUNDER_PIN);
    }
}

/* Timer1 solicita una lectura del ADC cada 100 ms. */
ISR(TIMER1_COMPA_vect)
{
    bandera_adc = 1U;
}

/* INT0 avisa que se pulso el boton. */
ISR(INT0_vect)
{
    bandera_boton = 1U;
}

static uint32_t leer_tiempo(void)
{
    uint8_t estado_interrupciones = SREG;
    uint32_t copia;

    cli();
    copia = tiempo_ms;
    SREG = estado_interrupciones;

    return copia;
}

static void gpio_init(void)
{
    /* LED y sounder como salidas. */
    DDRB |= (1U << DDB5) | (1U << DDB3);
    PORTB &= (uint8_t)~((1U << LED_PIN) | (1U << SOUNDER_PIN));

    /* Boton como entrada con pull-up interno. */
    DDRD &= (uint8_t)~(1U << DDD2);
    PORTD |= (1U << BOTON_PIN);

    /* INT0 por flanco descendente: al pulsar, D2 pasa a GND. */
    EICRA = (1U << ISC01);
    EIFR = (1U << INTF0);
    EIMSK = (1U << INT0);
}

static void timer0_init(void)
{
    /* 16 MHz / 64 = 250 kHz. 250 cuentas forman 1 ms. */
    TCCR0A = (1U << WGM01);
    TCCR0B = (1U << CS01) | (1U << CS00);
    OCR0A = 249U;
    TCNT0 = 0U;
    TIMSK0 = (1U << OCIE0A);
}

static void timer1_init(void)
{
    /* 16 MHz / 64 = 250 kHz. 25000 cuentas forman 100 ms. */
    TCCR1A = 0U;
    TCCR1B = (1U << WGM12) | (1U << CS11) | (1U << CS10);
    OCR1A = 24999U;
    TCNT1 = 0U;
    TIMSK1 = (1U << OCIE1A);
}

static void adc_init(void)
{
    /* A0 como entrada, con AVcc como referencia. */
    DDRC &= (uint8_t)~(1U << DDC0);
    PORTC &= (uint8_t)~(1U << PORTC0);

    ADMUX = (1U << REFS0);
    ADCSRA = (1U << ADEN) |
             (1U << ADPS2) | (1U << ADPS1) | (1U << ADPS0);
}

static uint16_t adc_leer_a0(void)
{
    ADMUX = (1U << REFS0);
    ADCSRA |= (1U << ADSC);

    while ((ADCSRA & (1U << ADSC)) != 0U) {
        /* Se espera solamente el tiempo de conversion del ADC. */
    }

    return ADC;
}

static void activar_sonido(uint16_t duracion_ms)
{
    uint8_t estado_interrupciones = SREG;

    cli();
    tiempo_sonido_ms = duracion_ms;
    PORTB &= (uint8_t)~(1U << SOUNDER_PIN);
    SREG = estado_interrupciones;
}

static void procesar_boton(uint32_t ahora)
{
    if (!bandera_boton) {
        return;
    }

    bandera_boton = 0U;

    /* Se descartan los rebotes mecanicos menores a 50 ms. */
    if ((uint32_t)(ahora - ultimo_boton_ms) < ANTI_REBOTE_MS) {
        return;
    }

    ultimo_boton_ms = ahora;
    activar_sonido(DURACION_SONIDO_MS);
}

int main(void)
{
    uint32_t ahora;

    gpio_init();
    adc_init();
    timer0_init();
    timer1_init();

    sei();

    while (1) {
        ahora = leer_tiempo();
        procesar_boton(ahora);

        if (bandera_led) {
            bandera_led = 0U;
            PORTB ^= (1U << LED_PIN);
        }

        if (bandera_adc) {
            bandera_adc = 0U;
            valor_adc = adc_leer_a0();
        }

        /* Se conserva la lectura para agregar la LCD en el siguiente avance. */
        (void)valor_adc;
    }
}
