/*
 * Laboratorio 5 - Problema A
 * Segundo avance: LCD I2C y DHT11.
 * ATmega328P a 16 MHz.
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdint.h>

#define LED_PIN       PB5    /* D13 */
#define SOUNDER_PIN   PB3    /* D11 */
#define DHT_PIN       PD4    /* D4  */
#define BOTON_PIN     PD2    /* D2  */

#define LCD_RS        0x01U
#define LCD_E         0x04U
#define LCD_LUZ       0x08U

#define PERIODO_LED_MS       200U
#define PERIODO_LCD_MS       500U
#define PERIODO_DHT_MS       2000U
#define DURACION_SONIDO_MS   500U
#define ANTI_REBOTE_MS       50UL

static volatile uint32_t tiempo_ms = 0UL;
static volatile uint16_t contador_led_ms = 0U;
static volatile uint16_t contador_lcd_ms = 0U;
static volatile uint16_t contador_dht_ms = 0U;
static volatile uint16_t tiempo_sonido_ms = 0U;

static volatile uint8_t bandera_led = 0U;
static volatile uint8_t bandera_adc = 0U;
static volatile uint8_t bandera_lcd = 0U;
static volatile uint8_t bandera_dht = 0U;
static volatile uint8_t bandera_boton = 0U;

static uint16_t valor_adc = 0U;
static uint8_t temperatura = 0U;
static uint8_t humedad = 0U;
static uint8_t dht_valido = 0U;
static uint8_t error_dht = 0U;
static uint32_t ultimo_boton_ms = 0UL;

static uint8_t direccion_lcd = 0x27U;
static uint8_t lcd_disponible = 0U;
static uint8_t luz_lcd = LCD_LUZ;

/* Timer0 genera la base de tiempo de 1 ms. */
ISR(TIMER0_COMPA_vect)
{
    tiempo_ms++;

    contador_led_ms++;
    if (contador_led_ms >= PERIODO_LED_MS) {
        contador_led_ms = 0U;
        bandera_led = 1U;
    }

    contador_lcd_ms++;
    if (contador_lcd_ms >= PERIODO_LCD_MS) {
        contador_lcd_ms = 0U;
        bandera_lcd = 1U;
    }

    contador_dht_ms++;
    if (contador_dht_ms >= PERIODO_DHT_MS) {
        contador_dht_ms = 0U;
        bandera_dht = 1U;
    }

    if (tiempo_sonido_ms > 0U) {
        PORTB ^= (1U << SOUNDER_PIN);
        tiempo_sonido_ms--;
    } else {
        PORTB &= (uint8_t)~(1U << SOUNDER_PIN);
    }
}

/* Timer1 solicita una conversion ADC cada 100 ms. */
ISR(TIMER1_COMPA_vect)
{
    bandera_adc = 1U;
}

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
    DDRB |= (1U << DDB5) | (1U << DDB3);
    PORTB &= (uint8_t)~((1U << LED_PIN) | (1U << SOUNDER_PIN));

    /* DHT11 como entrada con pull-up. */
    DDRD &= (uint8_t)~(1U << DDD4);
    PORTD |= (1U << DHT_PIN);

    /* Boton contra GND con pull-up interno. */
    DDRD &= (uint8_t)~(1U << DDD2);
    PORTD |= (1U << BOTON_PIN);
    EICRA = (1U << ISC01);
    EIFR = (1U << INTF0);
    EIMSK = (1U << INT0);
}

static void timer0_init(void)
{
    /* 16 MHz / 64 y OCR0A=249 producen 1 ms. */
    TCCR0A = (1U << WGM01);
    TCCR0B = (1U << CS01) | (1U << CS00);
    OCR0A = 249U;
    TCNT0 = 0U;
    TIMSK0 = (1U << OCIE0A);
}

static void timer1_init(void)
{
    /* 16 MHz / 64 y OCR1A=24999 producen 100 ms. */
    TCCR1A = 0U;
    TCCR1B = (1U << WGM12) | (1U << CS11) | (1U << CS10);
    OCR1A = 24999U;
    TCNT1 = 0U;
    TIMSK1 = (1U << OCIE1A);
}

static void adc_init(void)
{
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
        /* La conversion dura cerca de 104 us. */
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
    if ((uint32_t)(ahora - ultimo_boton_ms) < ANTI_REBOTE_MS) {
        return;
    }

    ultimo_boton_ms = ahora;
    activar_sonido(DURACION_SONIDO_MS);
}

/* =========================================================
   LCD I2C CON PCF8574
   ========================================================= */

static uint8_t twi_esperar(void)
{
    uint16_t espera;

    for (espera = 0U; espera < 3000U; espera++) {
        if ((TWCR & (1U << TWINT)) != 0U) {
            return 1U;
        }
        _delay_us(1);
    }
    return 0U;
}

static void twi_init(void)
{
    DDRC &= (uint8_t)~((1U << DDC4) | (1U << DDC5));
    PORTC |= (1U << PORTC4) | (1U << PORTC5);
    TWSR = 0U;
    TWBR = (uint8_t)(((F_CPU / 100000UL) - 16UL) / 2UL);
    TWCR = (1U << TWEN);
}

static uint8_t twi_iniciar(uint8_t direccion)
{
    uint8_t estado_twi;

    TWCR = (1U << TWINT) | (1U << TWSTA) | (1U << TWEN);
    if (!twi_esperar()) {
        return 0U;
    }

    estado_twi = TWSR & 0xF8U;
    if (estado_twi != TW_START && estado_twi != TW_REP_START) {
        return 0U;
    }

    TWDR = direccion;
    TWCR = (1U << TWINT) | (1U << TWEN);
    if (!twi_esperar()) {
        return 0U;
    }

    return ((TWSR & 0xF8U) == TW_MT_SLA_ACK) ? 1U : 0U;
}

static uint8_t twi_enviar(uint8_t dato)
{
    TWDR = dato;
    TWCR = (1U << TWINT) | (1U << TWEN);

    if (!twi_esperar()) {
        return 0U;
    }
    return ((TWSR & 0xF8U) == TW_MT_DATA_ACK) ? 1U : 0U;
}

static void twi_stop(void)
{
    TWCR = (1U << TWINT) | (1U << TWEN) | (1U << TWSTO);
}

static uint8_t lcd_expansor(uint8_t valor)
{
    uint8_t correcto = twi_iniciar((uint8_t)(direccion_lcd << 1));

    if (correcto) {
        correcto = twi_enviar((uint8_t)(valor | luz_lcd));
    }
    twi_stop();
    return correcto;
}

static uint8_t lcd_pulso(uint8_t valor)
{
    if (!lcd_expansor((uint8_t)(valor | LCD_E))) {
        return 0U;
    }
    _delay_us(1);

    if (!lcd_expansor((uint8_t)(valor & (uint8_t)~LCD_E))) {
        return 0U;
    }
    _delay_us(50);
    return 1U;
}

static uint8_t lcd_medio_byte(uint8_t valor)
{
    if (!lcd_expansor(valor)) {
        return 0U;
    }
    return lcd_pulso(valor);
}

static uint8_t lcd_byte(uint8_t valor, uint8_t es_dato)
{
    uint8_t control = es_dato ? LCD_RS : 0U;

    if (!lcd_medio_byte((uint8_t)((valor & 0xF0U) | control))) {
        return 0U;
    }
    return lcd_medio_byte((uint8_t)(((valor << 4) & 0xF0U) | control));
}

static void lcd_comando(uint8_t comando)
{
    if (!lcd_disponible) {
        return;
    }
    if (!lcd_byte(comando, 0U)) {
        lcd_disponible = 0U;
        return;
    }
    if (comando == 0x01U || comando == 0x02U) {
        _delay_ms(2);
    }
}

static void lcd_dato(uint8_t dato)
{
    if (lcd_disponible && !lcd_byte(dato, 1U)) {
        lcd_disponible = 0U;
    }
}

static uint8_t lcd_init(void)
{
    static const uint8_t direcciones[] = {0x27U, 0x3FU};
    uint8_t i;

    _delay_ms(50);
    for (i = 0U; i < (uint8_t)sizeof(direcciones); i++) {
        if (twi_iniciar((uint8_t)(direcciones[i] << 1))) {
            direccion_lcd = direcciones[i];
            twi_stop();
            lcd_disponible = 1U;
            break;
        }
        twi_stop();
    }

    if (!lcd_disponible) {
        return 0U;
    }

    (void)lcd_medio_byte(0x30U);
    _delay_ms(5);
    (void)lcd_medio_byte(0x30U);
    _delay_us(150);
    (void)lcd_medio_byte(0x30U);
    (void)lcd_medio_byte(0x20U);

    lcd_comando(0x28U);
    lcd_comando(0x0CU);
    lcd_comando(0x01U);
    lcd_comando(0x06U);
    return lcd_disponible;
}

static void linea_limpiar(char linea[17])
{
    uint8_t i;

    for (i = 0U; i < 16U; i++) {
        linea[i] = ' ';
    }
    linea[16] = '\0';
}

static void linea_texto(char linea[17], uint8_t posicion, const char *texto)
{
    while (*texto != '\0' && posicion < 16U) {
        linea[posicion++] = *texto++;
    }
}

static void linea_numero(char linea[17], uint8_t posicion,
                         uint16_t numero, uint8_t cifras)
{
    while (cifras > 0U) {
        cifras--;
        linea[posicion + cifras] = (char)('0' + (numero % 10U));
        numero /= 10U;
    }
}

static void lcd_linea(uint8_t fila, const char linea[17])
{
    uint8_t i;

    lcd_comando(fila == 0U ? 0x80U : 0xC0U);
    for (i = 0U; i < 16U; i++) {
        lcd_dato((uint8_t)linea[i]);
    }
}

static void lcd_actualizar(void)
{
    char linea1[17];
    char linea2[17];

    linea_limpiar(linea1);
    linea_limpiar(linea2);
    linea_texto(linea1, 0U, "ADC:");
    linea_numero(linea1, 5U, valor_adc, 4U);

    if (dht_valido) {
        linea_texto(linea2, 0U, "T:");
        linea_numero(linea2, 2U, temperatura, 2U);
        linea_texto(linea2, 4U, "C H:");
        linea_numero(linea2, 8U, humedad, 3U);
        linea2[11] = '%';
    } else if (error_dht != 0U) {
        linea_texto(linea2, 0U, "DHT ERROR:");
        linea_numero(linea2, 11U, error_dht, 1U);
    } else {
        linea_texto(linea2, 0U, "DHT: ESPERANDO");
    }

    lcd_linea(0U, linea1);
    lcd_linea(1U, linea2);
}

/* =========================================================
   DHT11 - PRIMERA INTEGRACION
   ========================================================= */

static void liberar_dht(void)
{
    DDRD &= (uint8_t)~(1U << DDD4);
    PORTD |= (1U << DHT_PIN);
}

static uint8_t medir_pulso_dht(uint8_t nivel_alto, uint16_t *cuentas)
{
    uint8_t nivel_actual;

    TCNT1 = 0U;
    while (1) {
        nivel_actual = ((PIND & (1U << DHT_PIN)) != 0U) ? 1U : 0U;
        if (nivel_actual != nivel_alto) {
            *cuentas = TCNT1;
            return 1U;
        }
        if (TCNT1 >= 2000U) {
            return 0U;
        }
    }
}

static uint8_t dht_leer(uint8_t *temp, uint8_t *hum)
{
    uint8_t datos[5] = {0U, 0U, 0U, 0U, 0U};
    uint16_t cuentas;
    uint8_t i;

    error_dht = 0U;

    /* Se guarda Timer1 porque normalmente planifica el ADC. */
    uint8_t timer1_a = TCCR1A;
    uint8_t timer1_b = TCCR1B;
    uint8_t timer1_int = TIMSK1;
    uint16_t timer1_comparacion = OCR1A;
    uint8_t estado_interrupciones = SREG;

    cli();
    TCCR1B = 0U;
    TCCR1A = 0U;
    TIMSK1 = 0U;
    TCNT1 = 0U;
    TCCR1B = (1U << CS11);       /* 0,5 us por cuenta. */

    DDRD |= (1U << DDD4);
    PORTD &= (uint8_t)~(1U << DHT_PIN);
    _delay_ms(25);
    liberar_dht();
    _delay_us(30);

    if (!medir_pulso_dht(1U, &cuentas)) {
        error_dht = 1U;
        goto finalizar;
    }
    if (!medir_pulso_dht(0U, &cuentas)) {
        error_dht = 2U;
        goto finalizar;
    }
    if (!medir_pulso_dht(1U, &cuentas)) {
        error_dht = 3U;
        goto finalizar;
    }

    for (i = 0U; i < 40U; i++) {
        uint16_t duracion_alta;

        if (!medir_pulso_dht(0U, &cuentas)) {
            error_dht = 4U;
            goto finalizar;
        }
        if (!medir_pulso_dht(1U, &duracion_alta)) {
            error_dht = 5U;
            goto finalizar;
        }

        datos[i / 8U] <<= 1U;
        if (duracion_alta > 100U) {
            datos[i / 8U] |= 1U;
        }
    }

    if ((uint8_t)(datos[0] + datos[1] + datos[2] + datos[3]) != datos[4]) {
        error_dht = 6U;
        goto finalizar;
    }

    *hum = datos[0];
    *temp = datos[2];

finalizar:
    TCCR1B = 0U;
    TCCR1A = timer1_a;
    OCR1A = timer1_comparacion;
    TCNT1 = 0U;
    TIFR1 = (1U << OCF1A);
    TIMSK1 = timer1_int;
    TCCR1B = timer1_b;
    SREG = estado_interrupciones;

    return (error_dht == 0U) ? 1U : 0U;
}

int main(void)
{
    uint32_t ahora;

    gpio_init();
    adc_init();
    twi_init();
    (void)lcd_init();
    timer0_init();
    timer1_init();

    bandera_adc = 1U;
    bandera_lcd = 1U;
    bandera_dht = 1U;
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

        if (bandera_dht) {
            bandera_dht = 0U;
            dht_valido = dht_leer(&temperatura, &humedad);
            bandera_lcd = 1U;
        }

        if (bandera_lcd) {
            bandera_lcd = 0U;
            lcd_actualizar();
        }
    }
}
