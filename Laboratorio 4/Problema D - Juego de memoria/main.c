/*
 * Laboratorio 4 - Problema D
 * Base de hardware para el juego de memoria.
 * Microcontrolador: ATmega328P a 16 MHz.
 *
 * Conexiones:
 *   PD6 / D6  -> DIN de la matriz WS2812B 8x8
 *   PD2 / D2  -> SW del joystick
 *   PB1 / D9  -> buzzer pasivo
 *   PC0 / A0  -> VRx del joystick
 *   PC1 / A1  -> VRy del joystick
 *   PD1 / D1  -> RXD de la terminal serie
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#define FILAS 8U
#define COLUMNAS 8U
#define N_PIXELES 64U
#define CANTIDAD_COLORES 4U

#define PIN_WS PD6
#define PIN_BOTON PD2
#define PIN_BUZZER PB1

#define UMBRAL_BAJO 300U
#define UMBRAL_ALTO 700U
#define BRILLO 40U

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint16_t frecuencia;
    const char *nombre;
} color_t;

static const color_t colores[CANTIDAD_COLORES] = {
    {255U, 0U, 0U, 262U, "ROJO"},
    {0U, 255U, 0U, 330U, "VERDE"},
    {0U, 0U, 255U, 392U, "AZUL"},
    {255U, 255U, 0U, 523U, "AMARILLO"}
};

static uint8_t cuadro[N_PIXELES][3];
static uint8_t fila_cursor = 3U;
static uint8_t columna_cursor = 3U;
static uint8_t color_cursor = 2U;
static uint8_t joystick_listo = 0U;
static uint8_t boton_anterior = 1U;

static void esperar_ms(uint16_t tiempo) {
    while (tiempo-- != 0U) {
        _delay_ms(1);
    }
}

static void uart_init(void) {
    UBRR0H = 0U;
    UBRR0L = 103U;
    UCSR0B = _BV(TXEN0);
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
}

static void uart_caracter(char caracter) {
    while ((UCSR0A & _BV(UDRE0)) == 0U) {}
    UDR0 = (uint8_t)caracter;
}

static void uart_texto(const char *texto) {
    while (*texto != '\0') {
        uart_caracter(*texto++);
    }
}

static void uart_numero(uint8_t numero) {
    uart_caracter((char)('0' + numero));
}

static void adc_init(void) {
    ADMUX = _BV(REFS0);
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
}

static uint16_t adc_leer(uint8_t canal) {
    ADMUX = (uint8_t)(_BV(REFS0) | (canal & 0x0FU));
    ADCSRA |= _BV(ADSC);
    while ((ADCSRA & _BV(ADSC)) != 0U) {}
    return ADC;
}

static void ws_byte(uint8_t valor, uint8_t alto, uint8_t bajo) {
    uint8_t bits = 8U;

    __asm__ volatile (
        "1: out %[puerto], %[hi] \n\t"
        "lsl %[dato]             \n\t"
        "brcs 2f                 \n\t"
        "nop                     \n\t"
        "nop                     \n\t"
        "out %[puerto], %[lo]    \n\t"
        "rjmp .+0                \n\t"
        "rjmp .+0                \n\t"
        "rjmp 3f                 \n\t"
        "2: rjmp .+0             \n\t"
        "rjmp .+0                \n\t"
        "rjmp .+0                \n\t"
        "nop                     \n\t"
        "out %[puerto], %[lo]    \n\t"
        "3: nop                  \n\t"
        "nop                     \n\t"
        "nop                     \n\t"
        "nop                     \n\t"
        "nop                     \n\t"
        "dec %[n]                \n\t"
        "brne 1b                 \n\t"
        : [dato] "+&r" (valor), [n] "+&r" (bits)
        : [puerto] "I" (_SFR_IO_ADDR(PORTD)),
          [hi] "r" (alto), [lo] "r" (bajo)
        : "cc"
    );
}

static uint8_t escalar(uint8_t valor) {
    return (uint8_t)(((uint16_t)valor * BRILLO) / 255U);
}

static uint8_t indice_pixel(uint8_t fila, uint8_t columna) {
    uint8_t fila_original = fila;
    fila = columna;
    columna = (uint8_t)(7U - fila_original);
    return (uint8_t)(fila * COLUMNAS + columna);
}

static void cuadro_limpiar(void) {
    uint8_t i;
    for (i = 0U; i < N_PIXELES; i++) {
        cuadro[i][0] = 0U;
        cuadro[i][1] = 0U;
        cuadro[i][2] = 0U;
    }
}

static void cuadro_pixel(uint8_t fila, uint8_t columna,
                         uint8_t r, uint8_t g, uint8_t b) {
    uint8_t indice;

    if ((fila >= FILAS) || (columna >= COLUMNAS)) {
        return;
    }

    indice = indice_pixel(fila, columna);
    cuadro[indice][0] = escalar(r);
    cuadro[indice][1] = escalar(g);
    cuadro[indice][2] = escalar(b);
}

static void matriz_mostrar(void) {
    uint8_t i;
    uint8_t sreg = SREG;
    uint8_t alto = (uint8_t)(PORTD | _BV(PIN_WS));
    uint8_t bajo = (uint8_t)(PORTD & (uint8_t)~_BV(PIN_WS));

    cli();
    for (i = 0U; i < N_PIXELES; i++) {
        ws_byte(cuadro[i][1], alto, bajo);
        ws_byte(cuadro[i][0], alto, bajo);
        ws_byte(cuadro[i][2], alto, bajo);
    }
    SREG = sreg;
    _delay_us(80);
}

static void mostrar_cursor(void) {
    const color_t *color = &colores[color_cursor];

    cuadro_limpiar();
    cuadro_pixel(fila_cursor, columna_cursor,
                 color->r, color->g, color->b);
    matriz_mostrar();
}

static void tono(uint16_t frecuencia, uint16_t duracion_ms) {
    OCR1A = (uint16_t)((F_CPU / (2UL * 8UL * frecuencia)) - 1UL);
    TCCR1A = _BV(COM1A0);
    TCCR1B = _BV(WGM12) | _BV(CS11);
    esperar_ms(duracion_ms);
    TCCR1A = 0U;
    TCCR1B = 0U;
    PORTB &= (uint8_t)~_BV(PIN_BUZZER);
}

static uint8_t boton_presionado(void) {
    return ((PIND & _BV(PIN_BOTON)) == 0U) ? 1U : 0U;
}

static uint8_t evento_boton(void) {
    uint8_t actual = boton_presionado();

    if (actual != boton_anterior) {
        esperar_ms(25U);
        actual = boton_presionado();
        if (actual != boton_anterior) {
            boton_anterior = actual;
            return actual;
        }
    }
    return 0U;
}

static void informar_cursor(void) {
    uart_texto("FILA=");
    uart_numero(fila_cursor);
    uart_texto(" COLUMNA=");
    uart_numero(columna_cursor);
    uart_texto(" COLOR=");
    uart_texto(colores[color_cursor].nombre);
    uart_texto("\r\n");
}

static void revisar_joystick(void) {
    uint16_t x = adc_leer(0U);
    uint16_t y = adc_leer(1U);
    uint8_t centrado_x = ((x >= UMBRAL_BAJO) && (x <= UMBRAL_ALTO));
    uint8_t centrado_y = ((y >= UMBRAL_BAJO) && (y <= UMBRAL_ALTO));
    uint8_t cambio = 0U;

    if ((centrado_x != 0U) && (centrado_y != 0U)) {
        joystick_listo = 1U;
        return;
    }
    if (joystick_listo == 0U) {
        return;
    }

    if ((x < UMBRAL_BAJO) && (columna_cursor < 7U)) {
        columna_cursor++;
        cambio = 1U;
    } else if ((x > UMBRAL_ALTO) && (columna_cursor > 0U)) {
        columna_cursor--;
        cambio = 1U;
    } else if ((y < UMBRAL_BAJO) && (fila_cursor < 7U)) {
        fila_cursor++;
        cambio = 1U;
    } else if ((y > UMBRAL_ALTO) && (fila_cursor > 0U)) {
        fila_cursor--;
        cambio = 1U;
    }

    joystick_listo = 0U;
    if (cambio != 0U) {
        mostrar_cursor();
        informar_cursor();
    }
}

static void hardware_init(void) {
    DDRD |= _BV(PIN_WS);
    PORTD &= (uint8_t)~_BV(PIN_WS);

    DDRD &= (uint8_t)~_BV(PIN_BOTON);
    PORTD |= _BV(PIN_BOTON);

    DDRB |= _BV(PIN_BUZZER);
    PORTB &= (uint8_t)~_BV(PIN_BUZZER);

    adc_init();
    uart_init();
}

int main(void) {
    hardware_init();
    esperar_ms(100U);

    uart_texto("LABORATORIO 4 - PROBLEMA D\r\n");
    uart_texto("PRUEBA DE MATRIZ, JOYSTICK Y BUZZER\r\n");
    mostrar_cursor();
    tono(colores[color_cursor].frecuencia, 120U);
    informar_cursor();

    for (;;) {
        revisar_joystick();

        if (evento_boton() != 0U) {
            color_cursor = (uint8_t)((color_cursor + 1U) % CANTIDAD_COLORES);
            mostrar_cursor();
            tono(colores[color_cursor].frecuencia, 120U);
            informar_cursor();
        }
        esperar_ms(5U);
    }
}
