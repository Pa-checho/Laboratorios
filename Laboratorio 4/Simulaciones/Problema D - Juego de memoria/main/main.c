/*
 * Laboratorio 4 - Problema D
 * Juego de memoria para ATmega328P escrito en C puro (avr-gcc).
 * No utiliza Arduino, C++ ni bibliotecas externas.
 *
 * Conexiones:
 *   PD6 / D6  -> DIN WS2812B 8x8
 *   PD2 / D2  -> SW del joystick (activo en bajo)
 *   PB1 / D9  -> buzzer pasivo
 *   PC0 / A0  -> VRx
 *   PC1 / A1  -> VRy
 *   PD1 / D1  -> RXD de la terminal virtual
 *
 * DISTRIBUCION_MATRIZ:
 *   0: lineal por filas (Proteus y matriz fisica usada en el laboratorio).
 *   1: filas en zigzag.
 *   2: columnas en zigzag, inicio arriba a la izquierda.
 *
 * GIRO_PANTALLA:
 *   0: orientacion electrica original.
 *   1: compensa el montaje fisico girado 90 grados en sentido antihorario.
 *
 * BRILLO:
 *   255 para Proteus.
 *   40 recomendado como inicio en el circuito fisico.
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef DISTRIBUCION_MATRIZ
#define DISTRIBUCION_MATRIZ 0
#endif

#ifndef GIRO_PANTALLA
#define GIRO_PANTALLA 1
#endif

#ifndef BRILLO
#define BRILLO 40
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdint.h>

#define FILAS 8U
#define COLUMNAS 8U
#define N_PIXELES 64U
#define CANTIDAD_COLORES 4U
#define VIDAS_INICIALES 3U
#define NIVEL_MAXIMO 20U

#define PIN_WS PD6
#define PIN_BOTON PD2
#define PIN_BUZZER PB1

#define UMBRAL_BAJO 300U
#define UMBRAL_ALTO 700U

#define ARRIBA 0U
#define ABAJO 1U
#define IZQUIERDA 2U
#define DERECHA 3U

#define MARCA_EEPROM 0xD4U
#define EEPROM_RECORD_BASE 4U

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint16_t frecuencia;
    char inicial;
} color_t;

typedef struct {
    uint8_t fila;
    uint8_t columna;
    uint8_t color;
    uint8_t direccion;
} paso_t;

typedef struct {
    const char *nombre_color;
    const char *nombre;
    uint16_t tono_flecha_ms;
    uint16_t flecha_extra_ms;
    uint16_t objetivo_ms;
    uint16_t pausa_ms;
} dificultad_t;

static const color_t colores[CANTIDAD_COLORES] = {
    {255U, 0U, 0U, 262U, 'R'},
    {0U, 255U, 0U, 330U, 'V'},
    {0U, 0U, 255U, 392U, 'A'},
    {255U, 255U, 0U, 523U, 'Y'}
};

/* La dificultad elegida corresponde al color del cursor en modo libre. */
static const dificultad_t dificultades[CANTIDAD_COLORES] = {
    {"ROJO",     "EXPERTO",  70U,  10U,  40U,  30U},
    {"VERDE",    "FACIL",   250U,  50U, 200U, 150U},
    {"AZUL",     "NORMAL",  140U,  40U, 100U,  80U},
    {"AMARILLO", "DIFICIL",  70U,  20U,  50U,  40U}
};

static const uint8_t digitos[10][5] = {
    {0x07U, 0x05U, 0x05U, 0x05U, 0x07U},
    {0x02U, 0x06U, 0x02U, 0x02U, 0x07U},
    {0x07U, 0x01U, 0x07U, 0x04U, 0x07U},
    {0x07U, 0x01U, 0x07U, 0x01U, 0x07U},
    {0x05U, 0x05U, 0x07U, 0x01U, 0x01U},
    {0x07U, 0x04U, 0x07U, 0x01U, 0x07U},
    {0x07U, 0x04U, 0x07U, 0x05U, 0x07U},
    {0x07U, 0x01U, 0x02U, 0x02U, 0x02U},
    {0x07U, 0x05U, 0x07U, 0x05U, 0x07U},
    {0x07U, 0x05U, 0x07U, 0x01U, 0x07U}
};

static const uint8_t flechas[4][8] = {
    {
        0x18U, 0x3CU, 0x7EU, 0xFFU,
        0x18U, 0x18U, 0x18U, 0x18U
    },
    {
        0x18U, 0x18U, 0x18U, 0x18U,
        0xFFU, 0x7EU, 0x3CU, 0x18U
    },
    {
        0x10U, 0x30U, 0x7FU, 0xFFU,
        0x7FU, 0x30U, 0x10U, 0x00U
    },
    {
        0x08U, 0x0CU, 0xFEU, 0xFFU,
        0xFEU, 0x0CU, 0x08U, 0x00U
    }
};

/* Framebuffer en orden RGB. La transmision a WS2812 se hace en orden GRB. */
static uint8_t cuadro[N_PIXELES][3];
static paso_t secuencia[NIVEL_MAXIMO];

static uint8_t nivel = 1U;
static uint8_t vidas = VIDAS_INICIALES;
static uint8_t fila_cursor = 3U;
static uint8_t columna_cursor = 3U;
static uint8_t color_cursor = 2U;
static uint8_t records[CANTIDAD_COLORES] = {1U, 1U, 1U, 1U};
static uint8_t ultimo_nivel_guardado = 1U;
static uint8_t joystick_listo = 0U;
static uint8_t boton_anterior = 1U;
static uint8_t contador_arranques = 0U;
static uint8_t primer_paso_anterior = 0xFFU;
static uint32_t estado_aleatorio = 1UL;

static void esperar_ms(uint16_t tiempo) {
    while (tiempo-- != 0U) {
        _delay_ms(1);
    }
}

static void uart_init(void) {
    UBRR0H = 0U;
    UBRR0L = 103U; /* 9600 baud a 16 MHz. */
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

/* Rutina WS2812B calibrada para ATmega328P a 16 MHz. */
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
    return (uint8_t)(((uint16_t)valor * (uint16_t)BRILLO) / 255U);
}

static uint8_t indice_pixel(uint8_t fila, uint8_t columna) {
#if GIRO_PANTALLA == 1
    /*
     * Vista frontal elegida para el montaje: la imagen sin compensar aparece
     * girada 90 grados a la izquierda. Se gira el framebuffer a la derecha
     * antes de convertir las coordenadas al indice electrico de la matriz.
     */
    uint8_t fila_original = fila;
    fila = columna;
    columna = (uint8_t)(7U - fila_original);
#endif

#if DISTRIBUCION_MATRIZ == 0
    return (uint8_t)(fila * 8U + columna);
#elif DISTRIBUCION_MATRIZ == 1
    if ((fila & 1U) != 0U) {
        columna = (uint8_t)(7U - columna);
    }
    return (uint8_t)(fila * 8U + columna);
#else
    /* Variante para matrices cuyo cableado avanza por columnas en zigzag. */
    if ((columna & 1U) != 0U) {
        fila = (uint8_t)(7U - fila);
    }
    return (uint8_t)(columna * 8U + fila);
#endif
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

static void cuadro_llenar(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t fila;
    uint8_t columna;
    for (fila = 0U; fila < FILAS; fila++) {
        for (columna = 0U; columna < COLUMNAS; columna++) {
            cuadro_pixel(fila, columna, r, g, b);
        }
    }
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

static void tono(uint16_t frecuencia, uint16_t duracion_ms) {
    OCR1A = (uint16_t)((F_CPU / (2UL * 8UL * frecuencia)) - 1UL);
    TCCR1A = _BV(COM1A0);
    TCCR1B = _BV(WGM12) | _BV(CS11);
    esperar_ms(duracion_ms);
    TCCR1A = 0U;
    TCCR1B = 0U;
    PORTB &= (uint8_t)~_BV(PIN_BUZZER);
}

static void informar_estado(void) {
    uart_texto("NIVEL: ");
    uart_numero(nivel);
    uart_texto(" | CURSOR: ");
    uart_texto(dificultades[color_cursor].nombre_color);
    uart_texto(" | DIFICULTAD: ");
    uart_texto(dificultades[color_cursor].nombre);
    uart_texto(" | VIDAS: ");
    uart_numero(vidas);
    uart_texto(" | RECORD: ");
    uart_numero(records[color_cursor]);
    uart_nueva_linea();
}

static void informar_records(void) {
    uart_texto("RECORDS | FACIL: ");
    uart_numero(records[1U]);
    uart_texto(" | NORMAL: ");
    uart_numero(records[2U]);
    uart_texto(" | DIFICIL: ");
    uart_numero(records[3U]);
    uart_texto(" | EXPERTO: ");
    uart_numero(records[0U]);
    uart_nueva_linea();
}

static void informar_cursor(void) {
    uart_texto("CURSOR FILA=");
    uart_numero(fila_cursor);
    uart_texto(" COLUMNA=");
    uart_numero(columna_cursor);
    uart_nueva_linea();
}

static void mostrar_cursor(void) {
    const color_t *c = &colores[color_cursor];
    cuadro_limpiar();
    cuadro_pixel(fila_cursor, columna_cursor, c->r, c->g, c->b);

    if (fila_cursor > 0U) {
        cuadro_pixel((uint8_t)(fila_cursor - 1U), columna_cursor,
                     (uint8_t)(c->r / 5U), (uint8_t)(c->g / 5U),
                     (uint8_t)(c->b / 5U));
    }
    if (fila_cursor < 7U) {
        cuadro_pixel((uint8_t)(fila_cursor + 1U), columna_cursor,
                     (uint8_t)(c->r / 5U), (uint8_t)(c->g / 5U),
                     (uint8_t)(c->b / 5U));
    }
    if (columna_cursor > 0U) {
        cuadro_pixel(fila_cursor, (uint8_t)(columna_cursor - 1U),
                     (uint8_t)(c->r / 5U), (uint8_t)(c->g / 5U),
                     (uint8_t)(c->b / 5U));
    }
    if (columna_cursor < 7U) {
        cuadro_pixel(fila_cursor, (uint8_t)(columna_cursor + 1U),
                     (uint8_t)(c->r / 5U), (uint8_t)(c->g / 5U),
                     (uint8_t)(c->b / 5U));
    }
    matriz_mostrar();
}

static void mostrar_flecha(uint8_t direccion, uint8_t color) {
    uint8_t fila;
    uint8_t columna;
    const color_t *c = &colores[color];

    cuadro_limpiar();
    for (fila = 0U; fila < 8U; fila++) {
        for (columna = 0U; columna < 8U; columna++) {
            if ((flechas[direccion][fila] & (uint8_t)(0x80U >> columna)) != 0U) {
                cuadro_pixel(fila, columna, c->r, c->g, c->b);
            }
        }
    }
    matriz_mostrar();
}

static void mostrar_objetivo(const paso_t *paso) {
    const color_t *c = &colores[paso->color];
    cuadro_limpiar();
    cuadro_pixel(paso->fila, paso->columna, c->r, c->g, c->b);
    matriz_mostrar();
}

static void dibujar_digito(uint8_t digito, uint8_t columna_inicial) {
    uint8_t fila;
    uint8_t columna;
    for (fila = 0U; fila < 5U; fila++) {
        for (columna = 0U; columna < 3U; columna++) {
            if ((digitos[digito][fila] & (uint8_t)(0x04U >> columna)) != 0U) {
                cuadro_pixel((uint8_t)(fila + 1U),
                             (uint8_t)(columna_inicial + columna),
                             0U, 180U, 255U);
            }
        }
    }
}

static void mostrar_nivel(void) {
    cuadro_limpiar();
    if (nivel < 10U) {
        dibujar_digito(nivel, 2U);
    } else {
        dibujar_digito((uint8_t)((nivel / 10U) % 10U), 0U);
        dibujar_digito((uint8_t)(nivel % 10U), 4U);
    }
    matriz_mostrar();
    tono(659U, 120U);
    esperar_ms(880U);
    matriz_apagar();
    esperar_ms(200U);
}

static void mostrar_vidas(void) {
    static const uint8_t columnas[3] = {0U, 3U, 6U};
    uint8_t vida;
    uint8_t fila;

    cuadro_limpiar();
    for (vida = 0U; vida < vidas; vida++) {
        for (fila = 3U; fila <= 4U; fila++) {
            cuadro_pixel(fila, columnas[vida], 255U, 0U, 0U);
            cuadro_pixel(fila, (uint8_t)(columnas[vida] + 1U), 255U, 0U, 0U);
        }
    }
    matriz_mostrar();
}

static void animacion_inicio(void) {
    uint8_t i;
    for (i = 0U; i < CANTIDAD_COLORES; i++) {
        cuadro_llenar(colores[i].r, colores[i].g, colores[i].b);
        matriz_mostrar();
        tono(colores[i].frecuencia, 100U);
        esperar_ms(80U);
    }
    matriz_apagar();
}

static void animacion_exito(void) {
    uint8_t vuelta;
    uint8_t i;
    for (vuelta = 0U; vuelta < 2U; vuelta++) {
        cuadro_limpiar();
        for (i = 0U; i < 8U; i++) {
            cuadro_pixel(0U, i, 0U, 255U, 0U);
            cuadro_pixel(7U, i, 0U, 255U, 0U);
            cuadro_pixel(i, 0U, 0U, 255U, 0U);
            cuadro_pixel(i, 7U, 0U, 255U, 0U);
        }
        matriz_mostrar();
        tono(784U, 140U);
        esperar_ms(40U);

        cuadro_llenar(0U, 255U, 0U);
        matriz_mostrar();
        tono(1047U, 180U);
        esperar_ms(40U);
        matriz_apagar();
        esperar_ms(120U);
    }
}

static void sonido_vida_perdida(void) {
    tono(220U, 120U);
    esperar_ms(30U);
    tono(165U, 180U);
}

static void animacion_derrota(void) {
    uint8_t i;
    for (i = 0U; i < 4U; i++) {
        cuadro_llenar(255U, 0U, 0U);
        matriz_mostrar();
        tono(110U, 250U);
        esperar_ms(30U);
        matriz_apagar();
        esperar_ms(180U);
    }
}

static void eeprom_cargar(void) {
    uint8_t marca = eeprom_read_byte((const uint8_t *)0U);
    uint8_t record_anterior;
    uint8_t i;

    if (marca != MARCA_EEPROM) {
        eeprom_update_byte((uint8_t *)0U, MARCA_EEPROM);
        eeprom_update_byte((uint8_t *)1U, 1U);
        eeprom_update_byte((uint8_t *)2U, 1U);
        eeprom_update_byte((uint8_t *)3U, 0U);
        for (i = 0U; i < CANTIDAD_COLORES; i++) {
            eeprom_update_byte((uint8_t *)(EEPROM_RECORD_BASE + i), 1U);
        }
    }

    ultimo_nivel_guardado = eeprom_read_byte((const uint8_t *)1U);
    record_anterior = eeprom_read_byte((const uint8_t *)2U);
    contador_arranques = (uint8_t)(eeprom_read_byte((const uint8_t *)3U) + 1U);
    eeprom_update_byte((uint8_t *)3U, contador_arranques);

    if ((ultimo_nivel_guardado < 1U) ||
        (ultimo_nivel_guardado > NIVEL_MAXIMO)) {
        ultimo_nivel_guardado = 1U;
    }

    for (i = 0U; i < CANTIDAD_COLORES; i++) {
        records[i] = eeprom_read_byte(
            (const uint8_t *)(EEPROM_RECORD_BASE + i));
        if ((records[i] < 1U) || (records[i] > NIVEL_MAXIMO)) {
            /* El record unico de versiones anteriores se conserva en NORMAL. */
            if ((i == 2U) && (record_anterior >= 1U) &&
                (record_anterior <= NIVEL_MAXIMO)) {
                records[i] = record_anterior;
            } else {
                records[i] = 1U;
            }
            eeprom_update_byte((uint8_t *)(EEPROM_RECORD_BASE + i),
                               records[i]);
        }
    }
}

static void eeprom_guardar_nivel(void) {
    ultimo_nivel_guardado = nivel;
    if (nivel > records[color_cursor]) {
        records[color_cursor] = nivel;
        eeprom_update_byte(
            (uint8_t *)(EEPROM_RECORD_BASE + color_cursor),
            records[color_cursor]);
    }
    eeprom_update_byte((uint8_t *)1U, ultimo_nivel_guardado);
}

static uint32_t aleatorio_siguiente(void) {
    /*
     * xorshift32 evita el ciclo corto de los bits bajos que presentaba el
     * generador congruencial anterior al seleccionar entre cuatro opciones.
     */
    estado_aleatorio ^= estado_aleatorio << 13;
    estado_aleatorio ^= estado_aleatorio >> 17;
    estado_aleatorio ^= estado_aleatorio << 5;
    return estado_aleatorio;
}

static uint8_t aleatorio_hasta(uint8_t limite) {
    uint16_t parte_alta = (uint16_t)(aleatorio_siguiente() >> 16);
    return (uint8_t)(((uint32_t)parte_alta * (uint32_t)limite) >> 16);
}

static void aleatorio_mezclar(uint32_t valor) {
    estado_aleatorio ^= valor + 0x9E3779B9UL +
                        (estado_aleatorio << 6) +
                        (estado_aleatorio >> 2);
    estado_aleatorio ^= estado_aleatorio << 13;
    estado_aleatorio ^= estado_aleatorio >> 17;
    estado_aleatorio ^= estado_aleatorio << 5;
}

static void aleatorio_sembrar(void) {
    uint8_t i;

    /*
     * A0/A1 aportan el ruido de los potenciometros del joystick.
     * A2/A3 deben quedar sin conectar para captar ruido electrico ambiental.
     * El temporizador y el contador persistente evitan semillas iguales aun
     * cuando el Arduino se reinicia siguiendo siempre el mismo procedimiento.
     */
    aleatorio_mezclar(((uint32_t)contador_arranques << 24) |
                      ((uint32_t)TCNT0 << 16));

    for (i = 0U; i < 32U; i++) {
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
    uint8_t elegida;

    if (indice > 0U) {
        fila_base = secuencia[indice - 1U].fila;
        columna_base = secuencia[indice - 1U].columna;
    }

    if (fila_base > 0U) candidatas[cantidad++] = ARRIBA;
    if (fila_base < 7U) candidatas[cantidad++] = ABAJO;
    if (columna_base > 0U) candidatas[cantidad++] = IZQUIERDA;
    if (columna_base < 7U) candidatas[cantidad++] = DERECHA;

    elegida = candidatas[aleatorio_hasta(cantidad)];
    secuencia[indice].direccion = elegida;
    secuencia[indice].color = aleatorio_hasta(CANTIDAD_COLORES);
    secuencia[indice].fila = fila_base;
    secuencia[indice].columna = columna_base;

    if (elegida == ARRIBA) secuencia[indice].fila--;
    if (elegida == ABAJO) secuencia[indice].fila++;
    if (elegida == IZQUIERDA) secuencia[indice].columna--;
    if (elegida == DERECHA) secuencia[indice].columna++;
}

static void reproducir_secuencia(void) {
    uint8_t i;
    const dificultad_t *configuracion = &dificultades[color_cursor];
    uart_texto("--- MEMORICE LA SECUENCIA ---\r\n");

    for (i = 0U; i < nivel; i++) {
        const paso_t *paso = &secuencia[i];
        const color_t *c = &colores[paso->color];

        uart_texto("PASO ");
        uart_numero((uint8_t)(i + 1U));
        uart_texto(": ");
        uart_texto(nombre_direccion(paso->direccion));
        uart_texto(" - COLOR ");
        uart_caracter(c->inicial);
        uart_texto(" - OBJETIVO ");
        uart_numero(paso->fila);
        uart_caracter(',');
        uart_numero(paso->columna);
        uart_nueva_linea();

        mostrar_flecha(paso->direccion, paso->color);
        tono(c->frecuencia, configuracion->tono_flecha_ms);
        esperar_ms(configuracion->flecha_extra_ms);
        mostrar_objetivo(paso);
        esperar_ms(configuracion->objetivo_ms);
        matriz_apagar();
        esperar_ms(configuracion->pausa_ms);
    }
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
            if (actual != 0U) {
                return 1U;
            }
        }
    }
    return 0U;
}

static void esperar_liberacion_boton(void) {
    while (boton_presionado() != 0U) {
        esperar_ms(5U);
    }
    esperar_ms(25U);
    boton_anterior = 0U;
    (void)evento_boton();
}

static void revisar_joystick(void) {
    uint16_t x = adc_leer(0U);
    uint16_t y = adc_leer(1U);
    uint8_t x_centrado = ((x >= UMBRAL_BAJO) && (x <= UMBRAL_ALTO)) ? 1U : 0U;
    uint8_t y_centrado = ((y >= UMBRAL_BAJO) && (y <= UMBRAL_ALTO)) ? 1U : 0U;
    uint8_t cambio = 0U;

    if ((x_centrado != 0U) && (y_centrado != 0U)) {
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

static void restaurar_posicion(uint8_t paso_jugador) {
    if (paso_jugador == 0U) {
        fila_cursor = 3U;
        columna_cursor = 3U;
    } else {
        fila_cursor = secuencia[paso_jugador - 1U].fila;
        columna_cursor = secuencia[paso_jugador - 1U].columna;
    }
}

/* Devuelve 1 si se completa el nivel y 0 si se pierden todas las vidas. */
static uint8_t turno_jugador(void) {
    uint8_t paso_jugador = 0U;

    fila_cursor = 3U;
    columna_cursor = 3U;
    joystick_listo = 0U;
    mostrar_vidas();
    esperar_ms(700U);
    mostrar_cursor();

    uart_texto("--- SU TURNO ---\r\n");
    uart_texto("MUEVA Y CONFIRME CADA PASO\r\n");
    informar_estado();

    for (;;) {
        revisar_joystick();

        if (evento_boton() != 0U) {
            const paso_t *esperado = &secuencia[paso_jugador];
            if ((fila_cursor == esperado->fila) &&
                (columna_cursor == esperado->columna)) {
                tono(colores[esperado->color].frecuencia, 160U);
                uart_texto("PASO ");
                uart_numero((uint8_t)(paso_jugador + 1U));
                uart_texto(" CORRECTO\r\n");
                paso_jugador++;

                if (paso_jugador >= nivel) {
                    return 1U;
                }
                mostrar_cursor();
            } else {
                vidas--;
                uart_texto("ERROR. VIDAS RESTANTES: ");
                uart_numero(vidas);
                uart_nueva_linea();
                sonido_vida_perdida();
                mostrar_vidas();
                esperar_ms(750U);

                if (vidas == 0U) {
                    return 0U;
                }

                restaurar_posicion(paso_jugador);
                mostrar_cursor();
                informar_estado();
                uart_texto("REPITA EL MISMO PASO\r\n");
            }
        }
        esperar_ms(5U);
    }
}

/* Espera en modo libre. Devuelve cuando se mantiene SW durante un segundo. */
static void modo_libre(void) {
    uint16_t tiempo_presionado;
    uint8_t nuevo_color;

    fila_cursor = 3U;
    columna_cursor = 3U;
    joystick_listo = 0U;
    mostrar_cursor();
    uart_texto("--- MODO LIBRE ---\r\n");
    uart_texto("CLIC: COLOR ALEATORIO | MANTENER 1 s: JUGAR\r\n");
    uart_texto("VERDE=FACIL | AZUL=NORMAL | AMARILLO=DIFICIL | ROJO=EXPERTO\r\n");
    uart_texto("CURSOR: ");
    uart_texto(dificultades[color_cursor].nombre_color);
    uart_texto(" | DIFICULTAD: ");
    uart_texto(dificultades[color_cursor].nombre);
    uart_texto(" | RECORD: ");
    uart_numero(records[color_cursor]);
    uart_nueva_linea();

    for (;;) {
        revisar_joystick();

        if (evento_boton() != 0U) {
            tiempo_presionado = 0U;
            while ((boton_presionado() != 0U) &&
                   (tiempo_presionado < 1000U)) {
                esperar_ms(10U);
                tiempo_presionado = (uint16_t)(tiempo_presionado + 10U);
            }

            if (tiempo_presionado >= 1000U) {
                aleatorio_sembrar();
                esperar_liberacion_boton();
                uart_texto("NUEVA PARTIDA\r\n");
                return;
            }

            nuevo_color = color_cursor;
            while (nuevo_color == color_cursor) {
                nuevo_color = aleatorio_hasta(CANTIDAD_COLORES);
            }
            color_cursor = nuevo_color;
            mostrar_cursor();
            tono(colores[color_cursor].frecuencia, 120U);
            uart_texto("CURSOR: ");
            uart_texto(dificultades[color_cursor].nombre_color);
            uart_texto(" | DIFICULTAD: ");
            uart_texto(dificultades[color_cursor].nombre);
            uart_texto(" | RECORD: ");
            uart_numero(records[color_cursor]);
            uart_nueva_linea();
        }
        esperar_ms(5U);
    }
}

static void jugar_partida(void) {
    uint8_t primer_paso;

    nivel = 1U;
    vidas = VIDAS_INICIALES;

    /* Una partida nueva nunca repite exactamente el primer paso anterior. */
    do {
        agregar_paso(0U);
        primer_paso = (uint8_t)((secuencia[0].direccion << 2) |
                                secuencia[0].color);
    } while (primer_paso == primer_paso_anterior);
    primer_paso_anterior = primer_paso;

    for (;;) {
        eeprom_guardar_nivel();
        informar_estado();
        mostrar_nivel();
        reproducir_secuencia();

        if (turno_jugador() == 0U) {
            uart_texto("SIN VIDAS - FIN DE LA PARTIDA\r\n");
            eeprom_guardar_nivel();
            animacion_derrota();
            return;
        }

        uart_texto("SECUENCIA CORRECTA\r\n");
        animacion_exito();

        if (nivel >= NIVEL_MAXIMO) {
            uart_texto("NIVEL MAXIMO SUPERADO\r\n");
            eeprom_guardar_nivel();
            return;
        }

        nivel++;
        agregar_paso((uint8_t)(nivel - 1U));
    }
}

static void hardware_init(void) {
    DDRD |= _BV(PIN_WS);
    PORTD &= (uint8_t)~_BV(PIN_WS);

    DDRD &= (uint8_t)~_BV(PIN_BOTON);
    PORTD |= _BV(PIN_BOTON);

    DDRB |= _BV(PIN_BUZZER);
    PORTB &= (uint8_t)~_BV(PIN_BUZZER);

    /* Timer 0 libre para aportar variacion a la semilla aleatoria. */
    TCCR0A = 0U;
    TCCR0B = _BV(CS02) | _BV(CS00);

    adc_init();
    uart_init();
}

int main(void) {
    hardware_init();
    esperar_ms(100U);
    eeprom_cargar();
    aleatorio_sembrar();

    uart_texto("LABORATORIO 4 - PROBLEMA D - C PURO\r\n");
    uart_texto("ULTIMO NIVEL: ");
    uart_numero(ultimo_nivel_guardado);
    uart_nueva_linea();
    informar_records();

    animacion_inicio();

    for (;;) {
        modo_libre();
        jugar_partida();
    }
}
