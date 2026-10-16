/*
 * EJERCICIO E - BURBUJA DE INCLINACION
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdint.h>
#include <string.h>

#define UART_UBRR              103U
#define LCD_ADDRESS            0x3EU
#define LCD_CONTROL_COMMAND    0x80U
#define LCD_CONTROL_DATA       0x40U

#define PIN_WS2812             PD6
#define PIN_BOTON_HOLD         PD2
#define PIN_BOTON_CALIBRAR     PD3
#define PIN_BUZZER             PB1

#define CANTIDAD_LEDS          64U
#define TAMANO_MATRIZ          8U
#define BRILLO                 255U

#define ZONA_NIVELADA          30
#define DIFERENCIA_DOMINANTE   30

#define PERIODO_MUESTREO       20UL
#define PERIODO_MATRIZ         40UL
#define PERIODO_LCD            100UL
#define PERIODO_TERMINAL       300UL

#define DURACION_ALERTA        500UL
#define DURACION_CALIBRACION   800UL
#define PERIODO_PARPADEO       120UL
#define TIEMPO_REBOTE          25UL

#define MARCA_CALIBRACION      0xE5U

typedef struct {
    uint8_t rojo;
    uint8_t verde;
    uint8_t azul;
} color_t;

typedef struct {
    uint8_t ultima_lectura;
    uint8_t estado_estable;
    uint32_t momento_cambio;
} boton_t;

typedef struct {
    uint8_t marca;
    int16_t offset_roll;
    int16_t offset_pitch;
} calibracion_t;

typedef enum {
    ESTADO_MEDICION,
    ESTADO_HOLD,
    ESTADO_CALIBRACION,
    ESTADO_ALERTA_LIMITE
} estado_t;

static volatile uint32_t tiempo_ms = 0;

static color_t matriz[CANTIDAD_LEDS];

static char lcd_linea_anterior_1[16];
static char lcd_linea_anterior_2[16];

static int16_t roll_tenths = 0;
static int16_t pitch_tenths = 0;

static int16_t offset_roll = 0;
static int16_t offset_pitch = 0;

static uint8_t filtro_inicializado = 0;

static uint8_t fila_actual = 4U;
static uint8_t columna_actual = 4U;
static uint8_t nivelado = 1U;
static uint8_t limite_activo = 0U;
static uint8_t limite_notificado = 0U;

static char eje_dominante = 'N';

static estado_t estado_actual = ESTADO_MEDICION;
static uint32_t inicio_estado_ms = 0;

static boton_t boton_hold = {1U, 1U, 0UL};
static boton_t boton_calibrar = {1U, 1U, 0UL};

static uint8_t buzzer_activo = 0U;
static uint32_t fin_buzzer_ms = 0;

static calibracion_t EEMEM calibracion_eeprom;

/* =========================================================
   TEMPORIZADOR DE 1 ms
   ========================================================= */

ISR(TIMER0_COMPA_vect)
{
    tiempo_ms++;
}

static void timer0_init(void)
{
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 249U;
    TIMSK0 = (1 << OCIE0A);
}

static uint32_t millis_read(void)
{
    uint32_t valor;
    uint8_t sreg_anterior = SREG;

    cli();
    valor = tiempo_ms;
    SREG = sreg_anterior;

    return valor;
}

/* =========================================================
   UART
   ========================================================= */

static void uart_init(void)
{
    UBRR0H = (uint8_t)(UART_UBRR >> 8);
    UBRR0L = (uint8_t)UART_UBRR;

    UCSR0A = 0U;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_send_char(char caracter)
{
    while ((UCSR0A & (1 << UDRE0)) == 0U) {
    }

    UDR0 = (uint8_t)caracter;
}

static void uart_send_text(const char *texto)
{
    while (*texto != '\0') {
        uart_send_char(*texto++);
    }
}

/* =========================================================
   I2C
   ========================================================= */

static void twi_init(void)
{
    TWSR = 0U;
    TWBR = (uint8_t)(((F_CPU / 100000UL) - 16UL) / 2UL);
    TWCR = (1 << TWEN);
}

static uint8_t twi_start(uint8_t direccion)
{
    uint8_t estado;

    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

    while ((TWCR & (1 << TWINT)) == 0U) {
    }

    estado = TWSR & 0xF8U;

    if (estado != TW_START && estado != TW_REP_START) {
        return 0U;
    }

    TWDR = direccion;
    TWCR = (1 << TWINT) | (1 << TWEN);

    while ((TWCR & (1 << TWINT)) == 0U) {
    }

    return ((TWSR & 0xF8U) == TW_MT_SLA_ACK) ? 1U : 0U;
}

static uint8_t twi_write(uint8_t valor)
{
    TWDR = valor;
    TWCR = (1 << TWINT) | (1 << TWEN);

    while ((TWCR & (1 << TWINT)) == 0U) {
    }

    return ((TWSR & 0xF8U) == TW_MT_DATA_ACK) ? 1U : 0U;
}

static void twi_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);

    while ((TWCR & (1 << TWSTO)) != 0U) {
    }
}

static uint8_t i2c_probe(uint8_t direccion)
{
    uint8_t respuesta;

    respuesta = twi_start((uint8_t)(direccion << 1));
    twi_stop();

    return respuesta;
}

static uint8_t i2c_write_pair(
    uint8_t direccion,
    uint8_t control,
    uint8_t valor)
{
    if (!twi_start((uint8_t)(direccion << 1))) {
        twi_stop();
        return 0U;
    }

    if (!twi_write(control)) {
        twi_stop();
        return 0U;
    }

    if (!twi_write(valor)) {
        twi_stop();
        return 0U;
    }

    twi_stop();
    return 1U;
}

/* =========================================================
   LCD 16x2 I2C
   ========================================================= */

static void lcd_command(uint8_t comando)
{
    i2c_write_pair(
        LCD_ADDRESS,
        LCD_CONTROL_COMMAND,
        comando
    );

    if (comando == 0x01U || comando == 0x02U) {
        _delay_ms(2);
    } else {
        _delay_us(50);
    }
}

static void lcd_data(uint8_t dato)
{
    i2c_write_pair(
        LCD_ADDRESS,
        LCD_CONTROL_DATA,
        dato
    );

    _delay_us(50);
}

static uint8_t lcd_init(void)
{
    memset(
        lcd_linea_anterior_1,
        0,
        sizeof(lcd_linea_anterior_1)
    );

    memset(
        lcd_linea_anterior_2,
        0,
        sizeof(lcd_linea_anterior_2)
    );

    _delay_ms(50);

    if (!i2c_probe(LCD_ADDRESS)) {
        return 0U;
    }

    lcd_command(0x3CU);
    lcd_command(0x0CU);
    lcd_command(0x01U);
    lcd_command(0x06U);

    return 1U;
}

static void lcd_format_line(char linea[16], const char *texto)
{
    uint8_t indice = 0U;

    while (indice < 16U && texto[indice] != '\0') {
        linea[indice] = texto[indice];
        indice++;
    }

    while (indice < 16U) {
        linea[indice++] = ' ';
    }
}

static void lcd_write_line(
    uint8_t fila,
    const char linea[16],
    char anterior[16])
{
    uint8_t indice;

    if (memcmp(linea, anterior, 16U) == 0) {
        return;
    }

    lcd_command(fila == 0U ? 0x80U : 0xC0U);

    for (indice = 0U; indice < 16U; indice++) {
        lcd_data((uint8_t)linea[indice]);
    }

    memcpy(anterior, linea, 16U);
}

/* =========================================================
   ADC
   ========================================================= */

static void adc_init(void)
{
    DDRC &= (uint8_t)~((1 << DDC0) | (1 << DDC1));
    PORTC &= (uint8_t)~((1 << PORTC0) | (1 << PORTC1));

    ADMUX = (1 << REFS0);

    ADCSRA =
        (1 << ADEN) |
        (1 << ADPS2) |
        (1 << ADPS1) |
        (1 << ADPS0);
}

static uint16_t adc_read(uint8_t canal)
{
    ADMUX = (uint8_t)(
        (1 << REFS0) |
        (canal & 0x0FU)
    );

    _delay_us(10);

    ADCSRA |= (1 << ADSC);

    while ((ADCSRA & (1 << ADSC)) != 0U) {
    }

    (void)ADC;

    ADCSRA |= (1 << ADSC);

    while ((ADCSRA & (1 << ADSC)) != 0U) {
    }

    return ADC;
}

static int16_t adc_to_angle_tenths(uint16_t valor_adc)
{
    int32_t resultado;

    resultado = ((int32_t)valor_adc * 600L) / 1023L;
    resultado -= 300L;

    return (int16_t)resultado;
}

static int16_t limitar_angulo(int16_t angulo)
{
    if (angulo > 300) {
        return 300;
    }

    if (angulo < -300) {
        return -300;
    }

    return angulo;
}

static int16_t valor_absoluto(int16_t valor)
{
    return valor < 0 ? (int16_t)(-valor) : valor;
}

static void leer_angulos(void)
{
    int16_t nuevo_roll;
    int16_t nuevo_pitch;

    nuevo_roll = (int16_t)(
        adc_to_angle_tenths(adc_read(0U)) - offset_roll
    );

    nuevo_pitch = (int16_t)(
        adc_to_angle_tenths(adc_read(1U)) - offset_pitch
    );

    nuevo_roll = limitar_angulo(nuevo_roll);
    nuevo_pitch = limitar_angulo(nuevo_pitch);

    if (!filtro_inicializado) {
        roll_tenths = nuevo_roll;
        pitch_tenths = nuevo_pitch;
        filtro_inicializado = 1U;
    } else {
        roll_tenths = (int16_t)(
            ((int32_t)roll_tenths * 7L + nuevo_roll) / 8L
        );

        pitch_tenths = (int16_t)(
            ((int32_t)pitch_tenths * 7L + nuevo_pitch) / 8L
        );
    }
}

/* =========================================================
   EEPROM Y CALIBRACIÓN
   ========================================================= */

static void calibracion_cargar(void)
{
    calibracion_t datos;

    eeprom_read_block(
        &datos,
        &calibracion_eeprom,
        sizeof(calibracion_t)
    );

    if (datos.marca == MARCA_CALIBRACION) {
        offset_roll = datos.offset_roll;
        offset_pitch = datos.offset_pitch;
    } else {
        offset_roll = 0;
        offset_pitch = 0;
    }
}

static void calibracion_guardar(void)
{
    calibracion_t datos;

    datos.marca = MARCA_CALIBRACION;
    datos.offset_roll = offset_roll;
    datos.offset_pitch = offset_pitch;

    eeprom_update_block(
        &datos,
        &calibracion_eeprom,
        sizeof(calibracion_t)
    );
}

static void calibrar_posicion_actual(uint32_t ahora)
{
    offset_roll = adc_to_angle_tenths(adc_read(0U));
    offset_pitch = adc_to_angle_tenths(adc_read(1U));

    calibracion_guardar();

    roll_tenths = 0;
    pitch_tenths = 0;
    filtro_inicializado = 0U;

    fila_actual = 4U;
    columna_actual = 4U;
    nivelado = 1U;
    limite_activo = 0U;
    limite_notificado = 0U;
    eje_dominante = 'N';

    estado_actual = ESTADO_CALIBRACION;
    inicio_estado_ms = ahora;
}

/* =========================================================
   BOTONES
   ========================================================= */

static void botones_init(void)
{
    DDRD &= (uint8_t)~(
        (1 << PIN_BOTON_HOLD) |
        (1 << PIN_BOTON_CALIBRAR)
    );

    PORTD |=
        (1 << PIN_BOTON_HOLD) |
        (1 << PIN_BOTON_CALIBRAR);
}

static uint8_t boton_evento(
    boton_t *boton,
    uint8_t mascara,
    uint32_t ahora)
{
    uint8_t lectura;

    lectura = (PIND & mascara) ? 1U : 0U;

    if (lectura != boton->ultima_lectura) {
        boton->ultima_lectura = lectura;
        boton->momento_cambio = ahora;
    }

    if (lectura != boton->estado_estable &&
        (uint32_t)(ahora - boton->momento_cambio) >=
        TIEMPO_REBOTE) {

        boton->estado_estable = lectura;

        if (lectura == 0U) {
            return 1U;
        }
    }

    return 0U;
}

/* =========================================================
   BUZZER
   ========================================================= */

static void buzzer_init(void)
{
    DDRB |= (1 << PIN_BUZZER);
    PORTB &= (uint8_t)~(1 << PIN_BUZZER);

    TCCR1A = 0U;
    TCCR1B = 0U;
}

static void buzzer_start(
    uint16_t frecuencia,
    uint16_t duracion,
    uint32_t ahora)
{
    OCR1A = (uint16_t)(
        (F_CPU / (2UL * 8UL * frecuencia)) - 1UL
    );

    TCCR1A = (1 << COM1A0);
    TCCR1B = (1 << WGM12) | (1 << CS11);

    buzzer_activo = 1U;
    fin_buzzer_ms = ahora + duracion;
}

static void buzzer_stop(void)
{
    TCCR1A = 0U;
    TCCR1B = 0U;

    PORTB &= (uint8_t)~(1 << PIN_BUZZER);
    buzzer_activo = 0U;
}

static void buzzer_update(uint32_t ahora)
{
    if (buzzer_activo &&
        (int32_t)(ahora - fin_buzzer_ms) >= 0) {

        buzzer_stop();
    }
}

/* =========================================================
   MATRIZ WS2812
   ========================================================= */

static void ws2812_init(void)
{
    DDRD |= (1 << PIN_WS2812);
    PORTD &= (uint8_t)~(1 << PIN_WS2812);
}

static void ws2812_send_byte(
    uint8_t valor,
    uint8_t puerto_alto,
    uint8_t puerto_bajo)
{
    uint8_t bits = 8U;

    __asm__ volatile (
        "1: out %[puerto], %[alto] \n\t"
        "lsl %[dato]               \n\t"
        "brcs 2f                   \n\t"
        "nop                       \n\t"
        "nop                       \n\t"
        "out %[puerto], %[bajo]    \n\t"
        "rjmp .+0                  \n\t"
        "rjmp .+0                  \n\t"
        "rjmp 3f                   \n\t"
        "2: rjmp .+0               \n\t"
        "rjmp .+0                  \n\t"
        "rjmp .+0                  \n\t"
        "nop                       \n\t"
        "out %[puerto], %[bajo]    \n\t"
        "3: nop                    \n\t"
        "nop                       \n\t"
        "nop                       \n\t"
        "nop                       \n\t"
        "nop                       \n\t"
        "dec %[cantidad]           \n\t"
        "brne 1b                   \n\t"
        : [dato] "+&r" (valor),
          [cantidad] "+&r" (bits)
        : [puerto] "I" (_SFR_IO_ADDR(PORTD)),
          [alto] "r" (puerto_alto),
          [bajo] "r" (puerto_bajo)
        : "cc"
    );
}

static uint8_t aplicar_brillo(uint8_t valor)
{
    return (uint8_t)(
        ((uint16_t)valor * (uint16_t)BRILLO) / 255U
    );
}

static uint8_t obtener_indice(
    uint8_t fila,
    uint8_t columna)
{
    return (uint8_t)(
        fila * TAMANO_MATRIZ + columna
    );
}

static void matriz_limpiar(void)
{
    memset(matriz, 0, sizeof(matriz));
}

static void matriz_pixel(
    uint8_t fila,
    uint8_t columna,
    uint8_t rojo,
    uint8_t verde,
    uint8_t azul)
{
    uint8_t indice;

    if (fila >= TAMANO_MATRIZ ||
        columna >= TAMANO_MATRIZ) {

        return;
    }

    indice = obtener_indice(fila, columna);

    matriz[indice].rojo = aplicar_brillo(rojo);
    matriz[indice].verde = aplicar_brillo(verde);
    matriz[indice].azul = aplicar_brillo(azul);
}

static void matriz_mostrar(void)
{
    uint8_t indice;
    uint8_t sreg_anterior = SREG;

    uint8_t puerto_alto = (uint8_t)(
        PORTD | (1 << PIN_WS2812)
    );

    uint8_t puerto_bajo = (uint8_t)(
        PORTD & (uint8_t)~(1 << PIN_WS2812)
    );

    cli();

    for (indice = 0U; indice < CANTIDAD_LEDS; indice++) {
        ws2812_send_byte(
            matriz[indice].verde,
            puerto_alto,
            puerto_bajo
        );

        ws2812_send_byte(
            matriz[indice].rojo,
            puerto_alto,
            puerto_bajo
        );

        ws2812_send_byte(
            matriz[indice].azul,
            puerto_alto,
            puerto_bajo
        );
    }

    SREG = sreg_anterior;

    _delay_us(80);
}

static uint8_t angulo_a_posicion(int16_t angulo)
{
    int32_t posicion;

    angulo = limitar_angulo(angulo);

    posicion = (int32_t)(angulo + 300) * 7L;
    posicion = (posicion + 300L) / 600L;

    if (posicion < 0L) {
        posicion = 0L;
    }

    if (posicion > 7L) {
        posicion = 7L;
    }

    return (uint8_t)posicion;
}

static void calcular_posicion(void)
{
    int16_t roll_absoluto;
    int16_t pitch_absoluto;

    roll_absoluto = valor_absoluto(roll_tenths);
    pitch_absoluto = valor_absoluto(pitch_tenths);

    nivelado =
        roll_absoluto <= ZONA_NIVELADA &&
        pitch_absoluto <= ZONA_NIVELADA;

    if (nivelado) {
        fila_actual = 4U;
        columna_actual = 4U;
        eje_dominante = 'N';
        limite_activo = 0U;
        return;
    }

    columna_actual = angulo_a_posicion(roll_tenths);
    fila_actual = angulo_a_posicion(pitch_tenths);

    if (roll_absoluto >
        (int16_t)(pitch_absoluto +
        DIFERENCIA_DOMINANTE)) {

        eje_dominante = 'X';
    } else if (
        pitch_absoluto >
        (int16_t)(roll_absoluto +
        DIFERENCIA_DOMINANTE)) {

        eje_dominante = 'Y';
    } else {
        eje_dominante = 'D';
    }

    limite_activo =
        fila_actual == 0U ||
        fila_actual == 7U ||
        columna_actual == 0U ||
        columna_actual == 7U;
}

static void matriz_actualizar(uint32_t ahora)
{
    uint8_t visible = 1U;

    matriz_limpiar();

    if (nivelado) {
        matriz_pixel(3U, 3U, 0U, 0U, 255U);
        matriz_pixel(3U, 4U, 0U, 0U, 255U);
        matriz_pixel(4U, 3U, 0U, 0U, 255U);
        matriz_pixel(4U, 4U, 0U, 0U, 255U);
    } else {
        if (estado_actual == ESTADO_ALERTA_LIMITE) {
            visible = (uint8_t)(
                (ahora / PERIODO_PARPADEO) & 0x01U
            );
        }

        if (visible) {
            if (eje_dominante == 'X') {
                matriz_pixel(
                    fila_actual,
                    columna_actual,
                    255U,
                    0U,
                    0U
                );
            } else if (eje_dominante == 'Y') {
                matriz_pixel(
                    fila_actual,
                    columna_actual,
                    0U,
                    255U,
                    0U
                );
            } else {
                matriz_pixel(
                    fila_actual,
                    columna_actual,
                    255U,
                    255U,
                    0U
                );
            }
        }
    }

    matriz_mostrar();
}

/* =========================================================
   FORMATO DE ÁNGULOS
   ========================================================= */

static void format_angle(
    int16_t angulo,
    char texto[6])
{
    uint16_t magnitud;

    if (angulo > 999) {
        angulo = 999;
    } else if (angulo < -999) {
        angulo = -999;
    }

    if (angulo < 0) {
        texto[0] = '-';
        magnitud = (uint16_t)(-angulo);
    } else {
        texto[0] = '+';
        magnitud = (uint16_t)angulo;
    }

    texto[1] = (char)(
        '0' + ((magnitud / 100U) % 10U)
    );

    texto[2] = (char)(
        '0' + ((magnitud / 10U) % 10U)
    );

    texto[3] = '.';
    texto[4] = (char)(
        '0' + (magnitud % 10U)
    );

    texto[5] = '\0';
}

/* =========================================================
   LCD
   ========================================================= */

static void lcd_update(void)
{
    char linea_1[16];
    char linea_2[16];

    char texto_x[6];
    char texto_y[6];

    format_angle(roll_tenths, texto_x);
    format_angle(pitch_tenths, texto_y);

    lcd_format_line(linea_1, "");
    lcd_format_line(linea_2, "");

    linea_1[0] = 'X';
    linea_1[1] = ':';
    linea_1[2] = ' ';
    memcpy(&linea_1[3], texto_x, 5U);

    linea_2[0] = 'Y';
    linea_2[1] = ':';
    linea_2[2] = ' ';
    memcpy(&linea_2[3], texto_y, 5U);

    lcd_write_line(
        0U,
        linea_1,
        lcd_linea_anterior_1
    );

    lcd_write_line(
        1U,
        linea_2,
        lcd_linea_anterior_2
    );
}

/* =========================================================
   TERMINAL SERIAL
   ========================================================= */

static void terminal_update(void)
{
    char texto_x[6];
    char texto_y[6];

    format_angle(roll_tenths, texto_x);
    format_angle(pitch_tenths, texto_y);

    uart_send_char('\r');

    uart_send_text("X: ");
    uart_send_text(texto_x);

    uart_send_text(" | Y: ");
    uart_send_text(texto_y);

    uart_send_text(" | ");

    switch (estado_actual) {
        case ESTADO_HOLD:
            uart_send_text("HOLD");
            break;

        case ESTADO_CALIBRACION:
            uart_send_text("CALIBRACION");
            break;

        case ESTADO_ALERTA_LIMITE:
            uart_send_text("ALERTA LIMITE");
            break;

        case ESTADO_MEDICION:
        default:
            if (nivelado) {
                uart_send_text("NIVELADO");
            } else {
                uart_send_text("MEDICION");
            }
            break;
    }

    uart_send_text("                    ");
}

/* =========================================================
   PROGRAMA PRINCIPAL
   ========================================================= */

int main(void)
{
    uint32_t ahora;

    uint32_t ultimo_muestreo = 0UL;
    uint32_t ultima_matriz = 0UL;
    uint32_t ultimo_lcd = 0UL;
    uint32_t ultima_terminal = 0UL;

    uint8_t evento_hold;
    uint8_t evento_calibrar;

    uart_init();
    adc_init();
    twi_init();
    botones_init();
    buzzer_init();
    ws2812_init();
    timer0_init();

    calibracion_cargar();
    (void)lcd_init();

    sei();

    uart_send_text(
        "\r\n=== BURBUJA DE INCLINACION ===\r\n"
    );

    leer_angulos();
    calcular_posicion();

    matriz_actualizar(0UL);
    lcd_update();
    terminal_update();

    while (1) {
        ahora = millis_read();

        buzzer_update(ahora);

        evento_hold = boton_evento(
            &boton_hold,
            (1 << PIN_BOTON_HOLD),
            ahora
        );

        evento_calibrar = boton_evento(
            &boton_calibrar,
            (1 << PIN_BOTON_CALIBRAR),
            ahora
        );

        if (evento_calibrar) {
            calibrar_posicion_actual(ahora);
            buzzer_start(2000U, 100U, ahora);
        } else if (evento_hold) {
            if (estado_actual == ESTADO_HOLD) {
                estado_actual = ESTADO_MEDICION;
                inicio_estado_ms = ahora;
                filtro_inicializado = 0U;
            } else if (
                estado_actual == ESTADO_MEDICION ||
                estado_actual == ESTADO_ALERTA_LIMITE) {

                estado_actual = ESTADO_HOLD;
                inicio_estado_ms = ahora;
                buzzer_stop();
            }
        }

        if (estado_actual == ESTADO_CALIBRACION &&
            (uint32_t)(ahora - inicio_estado_ms) >=
            DURACION_CALIBRACION) {

            estado_actual = ESTADO_MEDICION;
            inicio_estado_ms = ahora;
            filtro_inicializado = 0U;
        }

        if (estado_actual == ESTADO_ALERTA_LIMITE &&
            (uint32_t)(ahora - inicio_estado_ms) >=
            DURACION_ALERTA) {

            estado_actual = ESTADO_MEDICION;
            inicio_estado_ms = ahora;
        }

        if ((uint32_t)(ahora - ultimo_muestreo) >=
            PERIODO_MUESTREO) {

            ultimo_muestreo = ahora;

            if (estado_actual != ESTADO_HOLD &&
                estado_actual != ESTADO_CALIBRACION) {

                leer_angulos();
                calcular_posicion();

                if (!limite_activo) {
                    limite_notificado = 0U;
                }

                if (estado_actual == ESTADO_MEDICION &&
                    limite_activo &&
                    !limite_notificado) {

                    limite_notificado = 1U;
                    estado_actual = ESTADO_ALERTA_LIMITE;
                    inicio_estado_ms = ahora;

                    buzzer_start(1800U, 150U, ahora);
                }
            }
        }

        if ((uint32_t)(ahora - ultima_matriz) >=
            PERIODO_MATRIZ) {

            ultima_matriz = ahora;
            matriz_actualizar(ahora);
        }

        if ((uint32_t)(ahora - ultimo_lcd) >=
            PERIODO_LCD) {

            ultimo_lcd = ahora;
            lcd_update();
        }

        if ((uint32_t)(ahora - ultima_terminal) >=
            PERIODO_TERMINAL) {

            ultima_terminal = ahora;
            terminal_update();
        }
    }
}
