/*
 * LABORATORIO 5 - PROBLEMA A
 * Simulacion completa del Problema A en Proteus.
 *
 * ATmega328P / Arduino Uno a 16 MHz
 *
 * D13 / PB5: LED y canal A del osciloscopio
 * D11 / PB3: sounder pasivo (500 Hz mientras esta activo)
 * D3  / PD3: DATA del DHT11
 * D2  / PD2: pulsador INT0 contra GND
 * A0  / PC0: cursor del potenciometro
 * A4  / PC4: SDA del LCD I2C JHD-2X16-I2C
 * A5  / PC5: SCL del LCD I2C JHD-2X16-I2C
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdint.h>

#define LED_PIN             PB5
#define SOUNDER_PIN         PB3
#define DHT_PIN             PD3
#define BOTON_PIN           PD2

/* El modelo JHD-2X16-I2C de Proteus usa esta interfaz directa. */
#define LCD_ADDRESS         0x3EU
#define LCD_CMD             0x80U
#define LCD_DATA            0x40U

#define PERIODO_LED_MS              200U
#define PERIODO_LCD_MS              500U
#define PERIODO_DHT_MS              2000U
#define DURACION_SOUNDER_MS          500U
#define ANTI_REBOTE_MS              50UL
#define PULSACION_EMERGENCIA_MS      2000UL
#define TIEMPO_STANDBY_MS            10000UL
#define TIEMPO_APAGADO_MS            30000UL
#define UMBRAL_ACTIVIDAD_ADC         20U

typedef enum {
    ESTADO_ACTIVO = 0,
    ESTADO_STANDBY,
    ESTADO_APAGADO,
    ESTADO_SEGURO
} estado_sistema_t;

static volatile uint32_t tiempo_ms = 0UL;
static volatile uint16_t contador_led = 0U;
static volatile uint16_t contador_lcd = 0U;
static volatile uint16_t contador_dht = 0U;
static volatile uint16_t sounder_restante_ms = 0U;

static volatile uint8_t tarea_led = 0U;
static volatile uint8_t tarea_adc = 0U;
static volatile uint8_t tarea_lcd = 0U;
static volatile uint8_t tarea_dht = 0U;
static volatile uint8_t evento_boton = 0U;
static volatile uint8_t alarma_continua = 0U;

static uint16_t valor_adc = 0U;
static uint8_t temperatura = 0U;
static uint8_t humedad = 0U;
static uint8_t dht_valido = 0U;
static uint8_t lcd_disponible = 0U;
static estado_sistema_t estado = ESTADO_ACTIVO;

static uint32_t ultima_actividad = 0UL;
static uint32_t inicio_boton = 0UL;
static uint32_t ultimo_boton = 0UL;
static uint8_t boton_presionado = 0U;
static uint8_t pulsacion_larga_lista = 0U;
static uint16_t referencia_adc = 0U;
static uint8_t referencia_adc_valida = 0U;

/* =========================================================
   INTERRUPCIONES
   ========================================================= */

/* Timer0 genera la base temporal de 1 ms. */
ISR(TIMER0_COMPA_vect)
{
    tiempo_ms++;

    contador_led++;
    if (contador_led >= PERIODO_LED_MS) {
        contador_led = 0U;
        tarea_led = 1U;
    }

    contador_lcd++;
    if (contador_lcd >= PERIODO_LCD_MS) {
        contador_lcd = 0U;
        tarea_lcd = 1U;
    }

    contador_dht++;
    if (contador_dht >= PERIODO_DHT_MS) {
        contador_dht = 0U;
        tarea_dht = 1U;
    }

    /* Un cambio por milisegundo produce 500 Hz: periodo total de 2 ms. */
    if (alarma_continua || sounder_restante_ms > 0U) {
        PORTB ^= (1U << SOUNDER_PIN);

        if (!alarma_continua && sounder_restante_ms > 0U) {
            sounder_restante_ms--;
        }
    } else {
        PORTB &= (uint8_t)~(1U << SOUNDER_PIN);
    }
}

/* Timer1 solicita una adquisicion ADC cada 100 ms. */
ISR(TIMER1_COMPA_vect)
{
    tarea_adc = 1U;
}

/* INT0 solamente informa el evento; el trabajo se hace en main. */
ISR(INT0_vect)
{
    evento_boton = 1U;
}

/* =========================================================
   INICIALIZACION DE GPIO Y TEMPORIZADORES
   ========================================================= */

static void gpio_init(void)
{
    /* LED y sounder como salidas, inicialmente apagados. */
    DDRB |= (1U << DDB5) | (1U << DDB3);
    PORTB &= (uint8_t)~((1U << LED_PIN) | (1U << SOUNDER_PIN));

    /* DHT11 liberado como entrada con pull-up. */
    DDRD &= (uint8_t)~(1U << DDD3);
    PORTD |= (1U << DHT_PIN);

    /* Pulsador en D2 contra GND, usando el pull-up interno. */
    DDRD &= (uint8_t)~(1U << DDD2);
    PORTD |= (1U << BOTON_PIN);

    /* INT0 por flanco descendente: al pulsar pasa de 1 a 0. */
    EICRA = (uint8_t)((EICRA &
             (uint8_t)~((1U << ISC01) | (1U << ISC00))) |
             (1U << ISC01));
    EIFR = (1U << INTF0);
    EIMSK |= (1U << INT0);
}

static void timer0_init(void)
{
    /*
     * 16 MHz / 64 = 250 kHz -> cada cuenta dura 4 us.
     * 250 cuentas x 4 us = 1 ms, por eso OCR0A = 249.
     */
    TCCR0A = (1U << WGM01);
    TCCR0B = (1U << CS01) | (1U << CS00);
    OCR0A = 249U;
    TCNT0 = 0U;
    TIMSK0 = (1U << OCIE0A);
}

static void timer1_init(void)
{
    /*
     * 16 MHz / 64 = 250 kHz.
     * 25000 cuentas = 100 ms, por eso OCR1A = 24999.
     */
    TCCR1A = 0U;
    TCCR1B = (1U << WGM12) | (1U << CS11) | (1U << CS10);
    OCR1A = 24999U;
    TCNT1 = 0U;
    TIMSK1 = (1U << OCIE1A);
}

static uint32_t millis_seguro(void)
{
    uint8_t sreg = SREG;
    uint32_t copia;

    cli();
    copia = tiempo_ms;
    SREG = sreg;

    return copia;
}

static void sounder_activar(uint16_t duracion_ms)
{
    uint8_t sreg = SREG;

    cli();
    alarma_continua = 0U;
    sounder_restante_ms = duracion_ms;
    PORTB &= (uint8_t)~(1U << SOUNDER_PIN);
    SREG = sreg;
}

static void alarma_activar(void)
{
    uint8_t sreg = SREG;

    cli();
    sounder_restante_ms = 0U;
    alarma_continua = 1U;
    SREG = sreg;
}

static void sounder_apagar(void)
{
    uint8_t sreg = SREG;

    cli();
    sounder_restante_ms = 0U;
    alarma_continua = 0U;
    PORTB &= (uint8_t)~(1U << SOUNDER_PIN);
    SREG = sreg;
}

/* =========================================================
   ADC
   ========================================================= */

static void adc_habilitar(void)
{
    ADMUX = (1U << REFS0);
    ADCSRA = (1U << ADEN) |
             (1U << ADPS2) | (1U << ADPS1) | (1U << ADPS0);
}

static void adc_deshabilitar(void)
{
    ADCSRA &= (uint8_t)~(1U << ADEN);
}

static void adc_init(void)
{
    DDRC &= (uint8_t)~(1U << DDC0);
    PORTC &= (uint8_t)~(1U << PORTC0);

    /* Referencia AVcc y canal ADC0. Reloj ADC = 16 MHz / 128 = 125 kHz. */
    adc_habilitar();
}

static uint16_t adc_leer_a0(void)
{
    if ((ADCSRA & (1U << ADEN)) == 0U) {
        adc_habilitar();
    }

    ADMUX = (1U << REFS0);       /* ADC0. */
    ADCSRA |= (1U << ADSC);      /* Inicia conversion. */

    while ((ADCSRA & (1U << ADSC)) != 0U) {
    }

    return ADC;
}

/* =========================================================
   TWI / I2C
   ========================================================= */

static void twi_init(void)
{
    /* Pull-ups internos para la simulacion; el modulo real suele traerlos. */
    DDRC &= (uint8_t)~((1U << DDC4) | (1U << DDC5));
    PORTC |= (1U << PORTC4) | (1U << PORTC5);

    TWSR = 0U;  /* Prescaler TWI = 1. */
    TWBR = (uint8_t)(((F_CPU / 100000UL) - 16UL) / 2UL);
    TWCR = (1U << TWEN);
}

static uint8_t twi_start(uint8_t direccion_escritura)
{
    uint8_t estado;

    TWCR = (1U << TWINT) | (1U << TWSTA) | (1U << TWEN);
    while ((TWCR & (1U << TWINT)) == 0U) {
    }

    estado = TWSR & 0xF8U;
    if (estado != TW_START && estado != TW_REP_START) {
        return 0U;
    }

    TWDR = direccion_escritura;
    TWCR = (1U << TWINT) | (1U << TWEN);
    while ((TWCR & (1U << TWINT)) == 0U) {
    }

    return ((TWSR & 0xF8U) == TW_MT_SLA_ACK) ? 1U : 0U;
}

static uint8_t twi_write(uint8_t dato)
{
    TWDR = dato;
    TWCR = (1U << TWINT) | (1U << TWEN);
    while ((TWCR & (1U << TWINT)) == 0U) {
    }

    return ((TWSR & 0xF8U) == TW_MT_DATA_ACK) ? 1U : 0U;
}

static void twi_stop(void)
{
    TWCR = (1U << TWINT) | (1U << TWEN) | (1U << TWSTO);
    while ((TWCR & (1U << TWSTO)) != 0U) {
    }
}

static uint8_t i2c_write_pair(uint8_t control, uint8_t dato)
{
    if (!twi_start((uint8_t)(LCD_ADDRESS << 1))) {
        twi_stop();
        return 0U;
    }

    if (!twi_write(control) || !twi_write(dato)) {
        twi_stop();
        return 0U;
    }

    twi_stop();
    return 1U;
}

/* =========================================================
   LCD I2C DE PROTEUS
   ========================================================= */

static void lcd_comando(uint8_t comando)
{
    if (!lcd_disponible) {
        return;
    }

    if (!i2c_write_pair(LCD_CMD, comando)) {
        lcd_disponible = 0U;
        return;
    }

    if (comando == 0x01U || comando == 0x02U) {
        _delay_ms(2);
    } else {
        _delay_us(50);
    }
}

static void lcd_caracter(uint8_t dato)
{
    if (!lcd_disponible) {
        return;
    }

    if (!i2c_write_pair(LCD_DATA, dato)) {
        lcd_disponible = 0U;
        return;
    }

    _delay_us(50);
}

static uint8_t lcd_init(void)
{
    _delay_ms(50);

    if (!twi_start((uint8_t)(LCD_ADDRESS << 1))) {
        twi_stop();
        return 0U;
    }
    twi_stop();

    lcd_disponible = 1U;
    lcd_comando(0x3CU);  /* Dos lineas, caracteres 5x8. */
    lcd_comando(0x0CU);  /* Display encendido, cursor apagado. */
    lcd_comando(0x01U);  /* Limpiar. */
    lcd_comando(0x06U);  /* Incrementar cursor. */

    return lcd_disponible;
}

static void lcd_linea(uint8_t fila, const char texto[17])
{
    uint8_t i;

    lcd_comando(fila == 0U ? 0x80U : 0xC0U);
    for (i = 0U; i < 16U; i++) {
        lcd_caracter((uint8_t)texto[i]);
    }
}

static void linea_vacia(char linea[17])
{
    uint8_t i;

    for (i = 0U; i < 16U; i++) {
        linea[i] = ' ';
    }
    linea[16] = '\0';
}

static void poner_numero(char linea[17], uint8_t posicion,
                         uint16_t valor, uint8_t cifras)
{
    while (cifras > 0U) {
        cifras--;
        linea[posicion + cifras] = (char)('0' + (valor % 10U));
        valor /= 10U;
    }
}

static void poner_texto(char linea[17], uint8_t posicion, const char *texto)
{
    while (*texto != '\0' && posicion < 16U) {
        linea[posicion++] = *texto++;
    }
}

static void lcd_mostrar_activo(void)
{
    char linea1[17];
    char linea2[17];

    linea_vacia(linea1);
    linea_vacia(linea2);

    poner_texto(linea1, 0U, "ACTIVO ADC:");
    poner_numero(linea1, 11U, valor_adc, 4U);

    if (dht_valido) {
        linea2[0] = 'T';
        linea2[1] = ':';
        poner_numero(linea2, 2U, temperatura, 2U);
        linea2[4] = 'C';
        linea2[6] = 'H';
        linea2[7] = ':';
        poner_numero(linea2, 8U, humedad, 3U);
        linea2[11] = '%';
    } else {
        linea2[0] = 'D';
        linea2[1] = 'H';
        linea2[2] = 'T';
        linea2[3] = ':';
        linea2[5] = 'E';
        linea2[6] = 'S';
        linea2[7] = 'P';
        linea2[8] = 'E';
        linea2[9] = 'R';
        linea2[10] = 'A';
    }

    lcd_linea(0U, linea1);
    lcd_linea(1U, linea2);
}

static void lcd_mostrar_standby(void)
{
    char linea1[17];
    char linea2[17];

    linea_vacia(linea1);
    linea_vacia(linea2);
    poner_texto(linea1, 0U, "STANDBY");
    poner_texto(linea2, 0U, "Esperando...");
    lcd_linea(0U, linea1);
    lcd_linea(1U, linea2);
}

static void lcd_mostrar_apagado(void)
{
    char linea1[17];
    char linea2[17];

    linea_vacia(linea1);
    linea_vacia(linea2);
    poner_texto(linea1, 0U, "APAGADO");
    poner_texto(linea2, 0U, "Pulse el boton");
    lcd_linea(0U, linea1);
    lcd_linea(1U, linea2);
}

static void lcd_mostrar_seguro(void)
{
    char linea1[17];
    char linea2[17];

    linea_vacia(linea1);
    linea_vacia(linea2);
    poner_texto(linea1, 0U, "EMERGENCIA");
    poner_texto(linea2, 0U, "Pulse para salir");
    lcd_linea(0U, linea1);
    lcd_linea(1U, linea2);
}

/* =========================================================
   DHT11
   ========================================================= */

/* Espera mientras el pin conserve el nivel indicado. */
static uint8_t dht_esperar_mientras(uint8_t nivel_alto,
                                    uint16_t timeout_us)
{
    while ((((PIND & (1U << DHT_PIN)) != 0U) ? 1U : 0U) == nivel_alto) {
        if (timeout_us == 0U) {
            return 0U;
        }
        timeout_us--;
        _delay_us(1);
    }

    return 1U;
}

static uint8_t dht11_leer(uint8_t *temp, uint8_t *hum)
{
    uint8_t datos[5] = {0U, 0U, 0U, 0U, 0U};
    uint8_t i;
    uint8_t sreg;
    uint8_t correcto = 1U;

    /* Pulso de inicio: el microcontrolador mantiene DATA en 0 por 18 ms. */
    DDRD |= (1U << DDD3);
    PORTD &= (uint8_t)~(1U << DHT_PIN);
    _delay_ms(18);

    /* Libera el bus. La resistencia pull-up lo lleva a 1. */
    PORTD |= (1U << DHT_PIN);
    DDRD &= (uint8_t)~(1U << DDD3);
    _delay_us(20);

    /* La parte critica dura pocos milisegundos y requiere medir pulsos. */
    sreg = SREG;
    cli();

    if (!dht_esperar_mientras(1U, 100U) ||
        !dht_esperar_mientras(0U, 100U) ||
        !dht_esperar_mientras(1U, 100U)) {
        correcto = 0U;
    }

    for (i = 0U; i < 40U && correcto; i++) {
        if (!dht_esperar_mientras(0U, 80U)) {
            correcto = 0U;
            break;
        }

        _delay_us(40);
        datos[i / 8U] <<= 1U;

        if ((PIND & (1U << DHT_PIN)) != 0U) {
            datos[i / 8U] |= 1U;

            if (!dht_esperar_mientras(1U, 100U)) {
                correcto = 0U;
                break;
            }
        }
    }

    SREG = sreg;

    if (!correcto) {
        return 0U;
    }

    if ((uint8_t)(datos[0] + datos[1] + datos[2] + datos[3]) != datos[4]) {
        return 0U;
    }

    *hum = datos[0];
    *temp = datos[2];
    return 1U;
}

/* =========================================================
   MAQUINA DE ESTADOS
   ========================================================= */

static void revisar_actividad_adc(uint16_t nueva_lectura, uint32_t ahora)
{
    uint16_t diferencia;

    if (!referencia_adc_valida) {
        referencia_adc = nueva_lectura;
        referencia_adc_valida = 1U;
        return;
    }

    diferencia = (nueva_lectura > referencia_adc)
        ? (uint16_t)(nueva_lectura - referencia_adc)
        : (uint16_t)(referencia_adc - nueva_lectura);

    if (diferencia >= UMBRAL_ACTIVIDAD_ADC) {
        referencia_adc = nueva_lectura;
        ultima_actividad = ahora;
    }
}

static void entrar_activo(uint32_t ahora, uint8_t registrar_actividad)
{
    uint8_t sreg;

    alarma_continua = 0U;
    adc_habilitar();
    lcd_comando(0x0CU);
    estado = ESTADO_ACTIVO;
    PORTB &= (uint8_t)~(1U << LED_PIN);

    if (registrar_actividad) {
        ultima_actividad = ahora;
    }

    tarea_adc = 1U;
    tarea_lcd = 1U;

    /* Al despertar se esperan 2 s antes de volver a leer el DHT11. */
    sreg = SREG;
    cli();
    contador_dht = 0U;
    tarea_dht = 0U;
    SREG = sreg;
}

static void entrar_standby(void)
{
    estado = ESTADO_STANDBY;
    adc_deshabilitar();
    tarea_dht = 0U;
    PORTB &= (uint8_t)~(1U << LED_PIN);
    lcd_comando(0x0CU);
    lcd_mostrar_standby();
}

static void entrar_apagado(void)
{
    estado = ESTADO_APAGADO;
    adc_deshabilitar();
    tarea_dht = 0U;
    sounder_apagar();
    PORTB &= (uint8_t)~(1U << LED_PIN);
    lcd_comando(0x0CU);
    lcd_mostrar_apagado();
    _delay_ms(5);
    lcd_comando(0x08U);
}

static void entrar_seguro(void)
{
    estado = ESTADO_SEGURO;
    adc_deshabilitar();
    tarea_dht = 0U;
    lcd_comando(0x0CU);
    lcd_mostrar_seguro();
    alarma_activar();
}

static void revisar_inactividad(uint32_t ahora)
{
    uint32_t inactividad = (uint32_t)(ahora - ultima_actividad);

    if (estado == ESTADO_ACTIVO &&
        inactividad >= TIEMPO_STANDBY_MS) {
        entrar_standby();
    } else if (estado == ESTADO_STANDBY &&
               inactividad >= TIEMPO_APAGADO_MS) {
        entrar_apagado();
    }
}

static void procesar_boton(uint32_t ahora)
{
    if (!evento_boton) {
        return;
    }

    evento_boton = 0U;

    if ((uint32_t)(ahora - ultimo_boton) < ANTI_REBOTE_MS) {
        return;
    }

    ultimo_boton = ahora;

    if (estado == ESTADO_SEGURO) {
        sounder_apagar();
        boton_presionado = 1U;
        pulsacion_larga_lista = 1U;
        inicio_boton = ahora;
        entrar_activo(ahora, 1U);
        sounder_activar(DURACION_SOUNDER_MS);
    } else {
        boton_presionado = 1U;
        pulsacion_larga_lista = 0U;
        inicio_boton = ahora;

        if (estado != ESTADO_ACTIVO) {
            entrar_activo(ahora, 1U);
        } else {
            ultima_actividad = ahora;
        }

        sounder_activar(DURACION_SOUNDER_MS);
    }
}

static void revisar_boton_mantenido(uint32_t ahora)
{
    uint8_t presionado =
        ((PIND & (1U << BOTON_PIN)) == 0U) ? 1U : 0U;

    if (boton_presionado && !presionado) {
        boton_presionado = 0U;
        pulsacion_larga_lista = 0U;
    }

    if (boton_presionado && presionado && !pulsacion_larga_lista &&
        estado != ESTADO_SEGURO &&
        (uint32_t)(ahora - inicio_boton) >= PULSACION_EMERGENCIA_MS) {
        pulsacion_larga_lista = 1U;
        entrar_seguro();
    }
}

static void ejecutar_tareas(uint32_t ahora)
{
    if (tarea_led) {
        tarea_led = 0U;
        PORTB ^= (1U << LED_PIN);
    }

    if (tarea_adc) {
        tarea_adc = 0U;
        valor_adc = adc_leer_a0();
        revisar_actividad_adc(valor_adc, ahora);
    }

    if (tarea_dht) {
        tarea_dht = 0U;
        dht_valido = dht11_leer(&temperatura, &humedad);
        tarea_lcd = 1U;
    }

    if (tarea_lcd) {
        tarea_lcd = 0U;
        lcd_mostrar_activo();
    }
}

static void descartar_tareas(void)
{
    tarea_adc = 0U;
    tarea_dht = 0U;
    tarea_lcd = 0U;

    if (tarea_led) {
        tarea_led = 0U;
        if (estado == ESTADO_SEGURO) {
            PORTB ^= (1U << LED_PIN);
        } else {
            PORTB &= (uint8_t)~(1U << LED_PIN);
        }
    }
}

/* =========================================================
   PROGRAMA PRINCIPAL
   ========================================================= */

int main(void)
{
    uint32_t ahora;

    gpio_init();
    adc_init();
    twi_init();
    (void)lcd_init();
    timer0_init();
    timer1_init();

    valor_adc = adc_leer_a0();
    referencia_adc = valor_adc;
    referencia_adc_valida = 1U;
    ultima_actividad = 0UL;
    lcd_mostrar_activo();

    sei();

    while (1) {
        ahora = millis_seguro();

        procesar_boton(ahora);
        revisar_boton_mantenido(ahora);

        if (estado != ESTADO_SEGURO) {
            revisar_inactividad(ahora);
        }

        if (estado == ESTADO_ACTIVO) {
            ejecutar_tareas(ahora);
        } else {
            descartar_tareas();
        }
    }
}

