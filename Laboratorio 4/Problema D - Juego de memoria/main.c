/*
 * Laboratorio 4 - Problema D
 * Juego de memoria para ATmega328P a 16 MHz.
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
#define NIVEL_MAXIMO 20U

#define PIN_WS PD6
#define PIN_BOTON PD2
#define PIN_BUZZER PB1

#define UMBRAL_BAJO 300U
#define UMBRAL_ALTO 700U
#define BRILLO 40U

#define ARRIBA 0U
#define ABAJO 1U
#define IZQUIERDA 2U
#define DERECHA 3U

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint16_t frecuencia;
    const char *nombre;
} color_t;

typedef struct {
    uint8_t fila;
    uint8_t columna;
    uint8_t color;
    uint8_t direccion;
} paso_t;

static const color_t colores[CANTIDAD_COLORES] = {
    {255U, 0U, 0U, 262U, "ROJO"},
    {0U, 255U, 0U, 330U, "VERDE"},
    {0U, 0U, 255U, 392U, "AZUL"},
    {255U, 255U, 0U, 523U, "AMARILLO"}
};

static uint8_t cuadro[N_PIXELES][3];
static paso_t secuencia[NIVEL_MAXIMO];

static uint8_t nivel = 1U;
static uint8_t fila_cursor = 3U;
static uint8_t columna_cursor = 3U;
static uint8_t color_cursor = 2U;
static uint8_t joystick_listo = 0U;
static uint8_t boton_anterior = 1U;
static uint32_t estado_aleatorio = 1UL;

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
    if (numero >= 100U) {
        uart_caracter((char)('0' + (numero / 100U)));
    }
    if (numero >= 10U) {
        uart_caracter((char)('0' + ((numero / 10U) % 10U)));
    }
    uart_caracter((char)('0' + (numero % 10U)));
}

static void uart_nueva_linea(void) {
    uart_texto("\r\n");
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

static void matriz_apagar(void) {
    cuadro_limpiar();
    matriz_mostrar();
}

static void mostrar_cursor(void) {
    const color_t *color = &colores[color_cursor];

    cuadro_limpiar();
    cuadro_pixel(fila_cursor, columna_cursor,
                 color->r, color->g, color->b);
    matriz_mostrar();
}

static void mostrar_objetivo(const paso_t *paso) {
    const color_t *color = &colores[paso->color];

    cuadro_limpiar();
    cuadro_pixel(paso->fila, paso->columna,
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
    uart_nueva_linea();
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

static uint32_t aleatorio_siguiente(void) {
    estado_aleatorio ^= estado_aleatorio << 13;
    estado_aleatorio ^= estado_aleatorio >> 17;
    estado_aleatorio ^= estado_aleatorio << 5;
    return estado_aleatorio;
}

static uint8_t aleatorio_hasta(uint8_t limite) {
    uint16_t parte_alta = (uint16_t)(aleatorio_siguiente() >> 16);
    return (uint8_t)(((uint32_t)parte_alta * limite) >> 16);
}

static void aleatorio_mezclar(uint32_t valor) {
    estado_aleatorio ^= valor + 0x9E3779B9UL +
                        (estado_aleatorio << 6) +
                        (estado_aleatorio >> 2);
    (void)aleatorio_siguiente();
}

static void aleatorio_sembrar(void) {
    uint8_t i;

    for (i = 0U; i < 24U; i++) {
        uint32_t muestra = (uint32_t)adc_leer(0U);
        muestra ^= (uint32_t)adc_leer(1U) << 10;
        muestra ^= (uint32_t)adc_leer(2U) << 20;
        muestra ^= (uint32_t)(adc_leer(3U) & 0x03U) << 30;
        muestra ^= (uint32_t)TCNT0 * (uint32_t)(i + 1U);
        aleatorio_mezclar(muestra);
    }

    if (estado_aleatorio == 0UL) {
        estado_aleatorio = 1UL;
    }
}

static const char *nombre_direccion(uint8_t direccion) {
    if (direccion == ARRIBA) return "ARRIBA";
    if (direccion == ABAJO) return "ABAJO";
    if (direccion == IZQUIERDA) return "IZQUIERDA";
    return "DERECHA";
}

static void agregar_paso(uint8_t indice) {
    uint8_t fila_base = 3U;
    uint8_t columna_base = 3U;
    uint8_t candidatas[4];
    uint8_t cantidad = 0U;
    uint8_t direccion;

    if (indice > 0U) {
        fila_base = secuencia[indice - 1U].fila;
        columna_base = secuencia[indice - 1U].columna;
    }

    if (fila_base > 0U) candidatas[cantidad++] = ARRIBA;
    if (fila_base < 7U) candidatas[cantidad++] = ABAJO;
    if (columna_base > 0U) candidatas[cantidad++] = IZQUIERDA;
    if (columna_base < 7U) candidatas[cantidad++] = DERECHA;

    direccion = candidatas[aleatorio_hasta(cantidad)];
    secuencia[indice].fila = fila_base;
    secuencia[indice].columna = columna_base;
    secuencia[indice].direccion = direccion;
    secuencia[indice].color = aleatorio_hasta(CANTIDAD_COLORES);

    if (direccion == ARRIBA) secuencia[indice].fila--;
    if (direccion == ABAJO) secuencia[indice].fila++;
    if (direccion == IZQUIERDA) secuencia[indice].columna--;
    if (direccion == DERECHA) secuencia[indice].columna++;
}

static void reproducir_secuencia(void) {
    uint8_t i;

    uart_texto("--- MEMORICE LA SECUENCIA ---\r\n");
    for (i = 0U; i < nivel; i++) {
        const paso_t *paso = &secuencia[i];
        const color_t *color = &colores[paso->color];

        uart_texto("PASO ");
        uart_numero((uint8_t)(i + 1U));
        uart_texto(": ");
        uart_texto(nombre_direccion(paso->direccion));
        uart_texto(" | COLOR: ");
        uart_texto(color->nombre);
        uart_texto(" | POSICION: ");
        uart_numero(paso->fila);
        uart_caracter(',');
        uart_numero(paso->columna);
        uart_nueva_linea();

        mostrar_objetivo(paso);
        tono(color->frecuencia, 250U);
        esperar_ms(250U);
        matriz_apagar();
        esperar_ms(200U);
    }
}

static uint8_t turno_jugador(void) {
    uint8_t paso_jugador = 0U;

    fila_cursor = 3U;
    columna_cursor = 3U;
    joystick_listo = 0U;
    mostrar_cursor();

    uart_texto("--- SU TURNO ---\r\n");
    uart_texto("MUEVA EL CURSOR Y CONFIRME CADA POSICION\r\n");

    for (;;) {
        revisar_joystick();

        if (evento_boton() != 0U) {
            const paso_t *esperado = &secuencia[paso_jugador];

            if ((fila_cursor != esperado->fila) ||
                (columna_cursor != esperado->columna)) {
                uart_texto("POSICION INCORRECTA\r\n");
                tono(165U, 400U);
                matriz_apagar();
                return 0U;
            }

            tono(colores[esperado->color].frecuencia, 140U);
            uart_texto("PASO ");
            uart_numero((uint8_t)(paso_jugador + 1U));
            uart_texto(" CORRECTO\r\n");
            paso_jugador++;

            if (paso_jugador >= nivel) {
                return 1U;
            }
            mostrar_cursor();
        }
        esperar_ms(5U);
    }
}

static void sonido_nivel_superado(void) {
    tono(659U, 120U);
    esperar_ms(40U);
    tono(784U, 120U);
    esperar_ms(40U);
    tono(1047U, 180U);
}

static void esperar_inicio(void) {
    fila_cursor = 3U;
    columna_cursor = 3U;
    joystick_listo = 0U;
    mostrar_cursor();

    uart_texto("--- MODO DE ESPERA ---\r\n");
    uart_texto("MUEVA EL JOYSTICK Y PRESIONE SW PARA JUGAR\r\n");

    for (;;) {
        revisar_joystick();
        if (evento_boton() != 0U) {
            return;
        }
        esperar_ms(5U);
    }
}

static void jugar_partida(void) {
    nivel = 1U;
    aleatorio_sembrar();
    agregar_paso(0U);

    for (;;) {
        uart_texto("NIVEL: ");
        uart_numero(nivel);
        uart_nueva_linea();
        esperar_ms(500U);
        reproducir_secuencia();

        if (turno_jugador() == 0U) {
            uart_texto("FIN DE LA PARTIDA\r\n");
            esperar_ms(600U);
            return;
        }

        uart_texto("SECUENCIA CORRECTA\r\n");
        sonido_nivel_superado();

        if (nivel >= NIVEL_MAXIMO) {
            uart_texto("NIVEL MAXIMO SUPERADO\r\n");
            matriz_apagar();
            return;
        }

        nivel++;
        agregar_paso((uint8_t)(nivel - 1U));
        esperar_ms(500U);
    }
}

static void hardware_init(void) {
    DDRD |= _BV(PIN_WS);
    PORTD &= (uint8_t)~_BV(PIN_WS);

    DDRD &= (uint8_t)~_BV(PIN_BOTON);
    PORTD |= _BV(PIN_BOTON);

    DDRB |= _BV(PIN_BUZZER);
    PORTB &= (uint8_t)~_BV(PIN_BUZZER);

    TCCR0A = 0U;
    TCCR0B = _BV(CS02) | _BV(CS00);

    adc_init();
    uart_init();
}

int main(void) {
    hardware_init();
    esperar_ms(100U);

    uart_texto("LABORATORIO 4 - PROBLEMA D\r\n");
    uart_texto("JUEGO DE MEMORIA\r\n");

    for (;;) {
        esperar_inicio();
        jugar_partida();
    }
}
