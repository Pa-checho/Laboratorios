/* Etapa 2: Agregar el puerto serie.
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

void serie_iniciar(void)
{
    uint16_t divisor = F_CPU / (16UL * 9600UL) - 1;
    UBRR0H = divisor >> 8;
    UBRR0L = divisor;
    UCSR0A = 0;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0); /* Envio y recepcion. */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); /* Formato 8N1. */
}

void serie_texto(const char *texto)
{
    while (*texto != '\0') {
        while (!(UCSR0A & (1 << UDRE0))) {
            /* Espera hasta poder enviar otro caracter. */
        }
        UDR0 = *texto;
        texto++;
    }
}

void numero_a_texto(int16_t numero, char *texto)
{
    char digitos[5];
    uint8_t cantidad = 0;
    uint8_t posicion = 0;
    uint16_t valor;

    if (numero < 0) {
        texto[posicion++] = '-';
        /* Se usa 32 bits para poder convertir tambien -32768. */
        valor = (uint16_t)(-(int32_t)numero);
    } else {
        valor = (uint16_t)numero;
    }

    /* Extrae los digitos desde las unidades. Incluye el caso cero. */
    do {
        digitos[cantidad++] = '0' + valor % 10;
        valor /= 10;
    } while (valor > 0);

    /* Copia los digitos en el orden correcto. */
    while (cantidad > 0) texto[posicion++] = digitos[--cantidad];
    texto[posicion] = '\0'; /* Marca el final del texto. */
}

void serie_numero(int16_t numero)
{
    char texto[7];
    numero_a_texto(numero, texto);
    serie_texto(texto);
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

void informar_paso(uint8_t paso)
{
    if (paso == 0) serie_texto("Prueba: calefactor encendido\r\n");
    else if (paso == 1) serie_texto("Prueba: todas las salidas apagadas\r\n");
    else if (paso == 2) serie_texto("Prueba: ventilador bajo, 1 LED\r\n");
    else if (paso == 3) serie_texto("Prueba: ventilador medio, 2 LED\r\n");
    else serie_texto("Prueba: ventilador alto, 3 LED\r\n");
}

int main(void)
{
    uint8_t paso = 0;
    salidas_iniciar();
    serie_iniciar();
    serie_texto("Prueba de salidas, sin sensor.\r\n");
    while (1) {
        mostrar_paso(paso);
        informar_paso(paso);
        _delay_ms(1000); /* Espera bloqueante, solo para esta prueba. */
        paso++;
        if (paso == 5) paso = 0; /* Repite la secuencia. */
    }
}
