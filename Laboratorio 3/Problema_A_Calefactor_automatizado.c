/* Etapa 1: Probar los LED.
 * C para ATmega328P / Arduino Uno R3 a 16 MHz.
 * Cada archivo es un programa completo: compilar SOLO una etapa.
 * LED: D4/PD4, D5/PD5, D6/PD6 y calefactor D8/PB0.
 * Cada LED: pin -> resistencia 330 ohm -> anodo; catodo -> GND.
 */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#define CALEFACTOR_PIN PB0
#define LED_1 PD4
#define LED_2 PD5
#define LED_3 PD6

void leds_iniciar(void)
{
    PORTD &= ~((1 << LED_1) | (1 << LED_2) | (1 << LED_3));
    DDRD |= (1 << LED_1) | (1 << LED_2) | (1 << LED_3);
}

void ventilador(uint8_t nivel)
{
    /* Nivel 0: ninguno; 1: un LED; 2: dos; 3: los tres. */
    PORTD &= ~((1 << LED_1) | (1 << LED_2) | (1 << LED_3));
    if (nivel >= 1) PORTD |= (1 << LED_1);
    if (nivel >= 2) PORTD |= (1 << LED_2);
    if (nivel >= 3) PORTD |= (1 << LED_3);
}

/* Poner a cero antes de configurar evita encendidos al iniciar. */
void salidas_iniciar(void)
{
    PORTB &= ~(1 << CALEFACTOR_PIN);
    DDRB |= (1 << CALEFACTOR_PIN);
    leds_iniciar();
}

/* Secuencia de prueba: calefactor, reposo, bajo, medio y alto.
 * Todavia NO representa una temperatura medida.
 */
void mostrar_paso(uint8_t paso)
{
    PORTB &= ~(1 << CALEFACTOR_PIN);
    ventilador(0);
    if (paso == 0) PORTB |= (1 << CALEFACTOR_PIN);
    else if (paso >= 2) ventilador(paso - 1);
}

int main(void)
{
    uint8_t paso = 0;
    salidas_iniciar();
    while (1) {
        mostrar_paso(paso);
        _delay_ms(1000); /* Espera bloqueante, solo para esta prueba. */
        paso++;
        if (paso == 5) paso = 0; /* Repite la secuencia. */
    }
}
