/*
 * LABORATORIO 5 - PROBLEMA B / ACTIVIDAD 1
 * ATmega328P / Arduino Uno - 16 MHz - avr-gcc
 *
 * Arquitectura:
 * - Las ISR solo actualizan contadores, banderas o capturan pulsos breves.
 * - Las tareas se ejecutan en el lazo principal.
 * - La FSM utiliza if/else; no utiliza switch-case.
 * - La lectura del DHT11 se inicia de forma temporal y sus pulsos se
 *   capturan con PCINT20 + Timer2, sin detener Timer0 ni Timer1.
 *
 * Agregados B1:
 * - STANDBY: IDLE, Timer1 cada 100 ms y contexto conservado en SRAM.
 * - APAGADO: POWER-DOWN, perifericos detenidos y despertar por INT0 bajo.
 * - UART 9600 bps: comandos de prueba ACTIVO, ESTADO y EMERGENCIA + LF.
 * - El protocolo completo de Bluetooth y el WDT se desarrollan despues.
 * - Se conserva la FSM original, incluido ESTADO_SEGURO.
 *
 * Pines:
 *   D13 / PB5 -> LED
 *   D11 / PB3 -> sounder pasivo
 *   D4  / PD4 -> DATA DHT11 (requiere pull-up)
 *   D2  / PD2 -> pulsador INT0 contra GND
 *   A0  / PC0 -> cursor del potenciometro
 *   A4  / PC4 -> SDA LCD I2C
 *   A5  / PC5 -> SCL LCD I2C
 *   D0  / PD0 -> RX UART desde TX del HC-05 / terminal
 *   D1  / PD1 -> TX UART hacia RX del HC-05 (adaptar nivel si corresponde)
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdint.h>
#include <string.h>
#include <avr/sleep.h>

#if F_CPU != 16000000UL
#error "Esta version requiere revisar los temporizadores si F_CPU no es 16 MHz"
#endif

#define LED_PIN                         PB5
#define SOUNDER_PIN                     PB3
#define DHT_PIN                         PD4
#define BOTON_PIN                       PD2

#define LCD_RS                          0x01U
#define LCD_E                           0x04U
#define LCD_LUZ                         0x08U

#define PERIODO_LED_MS                  200U
#define PERIODO_LCD_MS                  500U
#define PERIODO_DHT_MS                  2000U
#define DURACION_SOUNDER_MS             500U
#define ANTI_REBOTE_MS                  50UL
#define PULSACION_EMERGENCIA_MS         2000UL
#define TIEMPO_STANDBY_MS               10000UL
#define TIEMPO_APAGADO_MS               30000UL

/*
 * Si vale 1, un cambio intencional del potenciometro cuenta como actividad.
 * Si el potenciometro se usa solo para simular un sensor, cambiar a 0.
 */
#define POTENCIOMETRO_ES_ACTIVIDAD      1U
#define UMBRAL_ACTIVIDAD_ADC            20U

/* Los dos relojes usan prescaler 64: 4 us por cuenta a 16 MHz. */
#define RELOJ_RAPIDO                    0U
#define RELOJ_STANDBY                   1U
#define RELOJ_DETENIDO                  2U
#define PASO_STANDBY_MS                 100UL

#define UART_RX_TAM                     64U
#define UART_TX_TAM                     128U
#define UART_LINEA_TAM                  24U
#define UART_TIMEOUT_LINEA_MS           500UL
#define UART_UBRR_9600                  103U

/* Solo bits definidos: el bit 4 reservado queda excluido. */
#define PRR_DORMIR (_BV(PRADC) | _BV(PRUSART0) | _BV(PRSPI) | \
                    _BV(PRTIM1) | _BV(PRTIM0) | _BV(PRTIM2) | _BV(PRTWI))

/* Timer2: 16 MHz / 8 = 2 MHz, 0,5 us por cuenta. */
#define DHT_RESPUESTA_MIN_CUENTAS       100U
#define DHT_RESPUESTA_MAX_CUENTAS       220U
#define DHT_BIT_BAJO_MIN_CUENTAS        50U
#define DHT_BIT_BAJO_MAX_CUENTAS        150U
#define DHT_UMBRAL_BIT_UNO_CUENTAS      100U
#define DHT_INICIO_BAJO_MS              20UL
#define DHT_TIMEOUT_MS                  10UL

typedef enum {
    ESTADO_ACTIVO = 0,
    ESTADO_STANDBY,
    ESTADO_APAGADO,
    ESTADO_SEGURO
} estado_sistema_t;

/* Estados internos del protocolo DHT11. Se almacenan en uint8_t. */
#define DHT_REPOSO                      0U
#define DHT_INICIO_BAJO                 1U
#define DHT_ESPERA_RESPUESTA_BAJA       2U
#define DHT_RESPUESTA_BAJA              3U
#define DHT_RESPUESTA_ALTA              4U
#define DHT_BIT_BAJO                    5U
#define DHT_BIT_ALTO                    6U
#define DHT_COMPLETO                    7U
#define DHT_ERROR                       8U

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
static volatile uint8_t alarma_continua = 0U;

static estado_sistema_t estado = ESTADO_ACTIVO;

static uint32_t ultima_actividad = 0UL;
static uint32_t inicio_boton = 0UL;
static uint32_t ultimo_boton = 0UL;

static uint8_t boton_presionado = 0U;
static uint8_t pulsacion_larga_lista = 0U;
static uint8_t lcd_disponible = 0U;
static uint8_t lectura_dht_correcta = 0U;
static uint8_t error_dht = 0U;
static uint8_t temperatura = 0U;
static uint8_t humedad = 0U;
static uint16_t valor_adc_actual = 0U;
static uint16_t referencia_adc_actividad = 0U;
static uint8_t referencia_adc_valida = 0U;

static uint8_t direccion_lcd = 0x27U;
static uint8_t luz_lcd = LCD_LUZ;

/* Variables compartidas con la ISR de cambio de pin del DHT11. */
static volatile uint8_t estado_dht = DHT_REPOSO;
static volatile uint8_t dht_datos[5] = {0U, 0U, 0U, 0U, 0U};
static volatile uint8_t dht_indice_bit = 0U;
static volatile uint8_t dht_nivel_anterior = 1U;
static volatile uint8_t dht_error_isr = 0U;
static uint32_t dht_marca_ms = 0UL;

/* Estado temporal y de comunicacion agregado para B1. */
static volatile uint8_t modo_reloj = RELOJ_RAPIDO;
static volatile uint8_t bandera_reloj = 0U;
static uint16_t fraccion_reloj_us = 0U;
static volatile uint8_t en_powerdown = 0U;

static volatile uint8_t uart_rx[UART_RX_TAM];
static volatile uint8_t uart_rx_cabeza = 0U;
static volatile uint8_t uart_rx_cola = 0U;
static volatile uint8_t uart_rx_error = 0U;
static volatile uint8_t uart_tx[UART_TX_TAM];
static volatile uint8_t uart_tx_cabeza = 0U;
static volatile uint8_t uart_tx_cola = 0U;
static volatile uint8_t uart_tx_iniciada = 0U;
static char uart_linea[UART_LINEA_TAM];
static uint8_t uart_linea_longitud = 0U;
static uint8_t uart_descartar_linea = 0U;
static uint32_t uart_inicio_linea = 0UL;

/* =========================================================
   INTERRUPCIONES
   ========================================================= */

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

    /* Cambio cada 1 ms: periodo de 2 ms, frecuencia de 500 Hz. */
    if (alarma_continua || tiempo_sonido_ms > 0U) {
        PORTB ^= (1U << SOUNDER_PIN);

        if (!alarma_continua && tiempo_sonido_ms > 0U) {
            tiempo_sonido_ms--;
        }
    } else {
        PORTB &= (uint8_t)~(1U << SOUNDER_PIN);
    }
}

ISR(TIMER1_COMPA_vect)
{
    if (modo_reloj == RELOJ_STANDBY) {
        tiempo_ms += PASO_STANDBY_MS; /* Mantiene la inactividad en IDLE. */
        bandera_reloj = 1U;
    } else if (modo_reloj == RELOJ_RAPIDO) {
        bandera_adc = 1U;             /* Funcion original de A en ACTIVO. */
    }
}

ISR(INT0_vect)
{
    bandera_boton = 1U;
    if (en_powerdown) {
        /* El nivel bajo podria repetir la IRQ mientras se mantiene pulsado. */
        EIMSK &= (uint8_t)~_BV(INT0);
    }
}

ISR(USART_RX_vect)
{
    uint8_t estado_uart = UCSR0A;     /* Leer errores ANTES de UDR0. */
    uint8_t dato = UDR0;
    uint8_t siguiente = (uint8_t)((uart_rx_cabeza + 1U) & (UART_RX_TAM - 1U));

    if ((estado_uart & (_BV(FE0) | _BV(DOR0) | _BV(UPE0))) != 0U ||
        siguiente == uart_rx_cola) {
        uart_rx_error = 1U;          /* No procesa ni valida comandos en ISR. */
    } else {
        uart_rx[uart_rx_cabeza] = dato;
        uart_rx_cabeza = siguiente;
    }
}

ISR(USART_UDRE_vect)
{
    if (uart_tx_cola != uart_tx_cabeza) {
        UCSR0A = _BV(TXC0);          /* Limpia TXC: aun falta transmitir. */
        UDR0 = uart_tx[uart_tx_cola];
        uart_tx_cola = (uint8_t)((uart_tx_cola + 1U) & (UART_TX_TAM - 1U));
        uart_tx_iniciada = 1U;
    } else {
        UCSR0B &= (uint8_t)~_BV(UDRIE0);
    }
}

/*
 * Captura cada flanco de DATA del DHT11. Timer2 se reinicia en cada flanco,
 * por lo que TCNT2 representa la duracion del nivel anterior en pasos de
 * 0,5 us. La ISR no espera activamente.
 */
ISR(PCINT2_vect)
{
    uint8_t nivel;
    uint8_t duracion;

    nivel = ((PIND & (1U << DHT_PIN)) != 0U) ? 1U : 0U;
    duracion = TCNT2;
    TCNT2 = 0U;

    if (nivel == dht_nivel_anterior) {
        return;
    }
    dht_nivel_anterior = nivel;

    if (estado_dht == DHT_ESPERA_RESPUESTA_BAJA) {
        if (nivel == 0U) {
            estado_dht = DHT_RESPUESTA_BAJA;
        }
    } else if (estado_dht == DHT_RESPUESTA_BAJA) {
        if (nivel == 1U &&
            duracion >= DHT_RESPUESTA_MIN_CUENTAS &&
            duracion <= DHT_RESPUESTA_MAX_CUENTAS) {
            estado_dht = DHT_RESPUESTA_ALTA;
        } else {
            dht_error_isr = 2U;
            estado_dht = DHT_ERROR;
            PCMSK2 &= (uint8_t)~(1U << PCINT20);
        }
    } else if (estado_dht == DHT_RESPUESTA_ALTA) {
        if (nivel == 0U &&
            duracion >= DHT_RESPUESTA_MIN_CUENTAS &&
            duracion <= DHT_RESPUESTA_MAX_CUENTAS) {
            estado_dht = DHT_BIT_BAJO;
        } else {
            dht_error_isr = 3U;
            estado_dht = DHT_ERROR;
            PCMSK2 &= (uint8_t)~(1U << PCINT20);
        }
    } else if (estado_dht == DHT_BIT_BAJO) {
        if (nivel == 1U &&
            duracion >= DHT_BIT_BAJO_MIN_CUENTAS &&
            duracion <= DHT_BIT_BAJO_MAX_CUENTAS) {
            estado_dht = DHT_BIT_ALTO;
        } else {
            dht_error_isr = 4U;
            estado_dht = DHT_ERROR;
            PCMSK2 &= (uint8_t)~(1U << PCINT20);
        }
    } else if (estado_dht == DHT_BIT_ALTO) {
        if (nivel == 0U) {
            uint8_t byte_actual = (uint8_t)(dht_indice_bit / 8U);

            dht_datos[byte_actual] <<= 1U;
            if (duracion > DHT_UMBRAL_BIT_UNO_CUENTAS) {
                dht_datos[byte_actual] |= 1U;
            }

            dht_indice_bit++;

            if (dht_indice_bit >= 40U) {
                estado_dht = DHT_COMPLETO;
                PCMSK2 &= (uint8_t)~(1U << PCINT20);
            } else {
                estado_dht = DHT_BIT_BAJO;
            }
        } else {
            dht_error_isr = 5U;
            estado_dht = DHT_ERROR;
            PCMSK2 &= (uint8_t)~(1U << PCINT20);
        }
    }
}

/* =========================================================
   TIEMPO Y SALIDAS
   ========================================================= */

static uint32_t leer_tiempo(void)
{
    uint8_t sreg = SREG;
    uint32_t copia;

    cli();
    copia = tiempo_ms;
    SREG = sreg;

    return copia;
}

static void iniciar_sonido(uint16_t duracion_ms)
{
    uint8_t sreg = SREG;

    cli();
    alarma_continua = 0U;
    tiempo_sonido_ms = duracion_ms;
    PORTB &= (uint8_t)~(1U << SOUNDER_PIN);
    SREG = sreg;
}

static void iniciar_alarma(void)
{
    uint8_t sreg = SREG;

    cli();
    tiempo_sonido_ms = 0U;
    alarma_continua = 1U;
    SREG = sreg;
}

static void apagar_sonido(void)
{
    uint8_t sreg = SREG;

    cli();
    tiempo_sonido_ms = 0U;
    alarma_continua = 0U;
    PORTB &= (uint8_t)~(1U << SOUNDER_PIN);
    SREG = sreg;
}


/* =========================================================
   UART 9600 8N1 - SOPORTE MINIMO DE PRUEBA PARA LA ACTIVIDAD 1
   El protocolo de tramas definitivo corresponde a la Actividad 4.
   ========================================================= */

static void uart_init(void)
{
    PRR &= (uint8_t)~_BV(PRUSART0);
    PORTD |= _BV(PD0);              /* RX definido cuando no hay modulo. */
    UCSR0A = 0U;                    /* Velocidad normal, U2X0=0. */
    UBRR0H = 0U;
    UBRR0L = UART_UBRR_9600;
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8 bits, sin paridad, 1 stop. */
    UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);
}

static uint8_t uart_enviar_texto(const char *texto)
{
    uint8_t longitud = 0U;
    uint8_t cabeza = uart_tx_cabeza;
    uint8_t libre;
    uint8_t i;
    uint8_t sreg;

    while (texto[longitud] != '\0' && longitud < UART_TX_TAM - 1U) {
        longitud++;
    }
    libre = (uint8_t)((uart_tx_cola - cabeza - 1U) & (UART_TX_TAM - 1U));
    if (longitud > libre) {
        return 0U;                 /* Sin espera activa si el buffer esta lleno. */
    }
    for (i = 0U; i < longitud; i++) {
        uart_tx[cabeza] = (uint8_t)texto[i];
        cabeza = (uint8_t)((cabeza + 1U) & (UART_TX_TAM - 1U));
    }
    sreg = SREG;
    cli();
    uart_tx_cabeza = cabeza;        /* Publica el bloque una vez copiado. */
    UCSR0B |= _BV(UDRIE0);
    SREG = sreg;
    return 1U;
}

static uint8_t uart_tomar_byte(uint8_t *dato)
{
    if (uart_rx_cola == uart_rx_cabeza) {
        return 0U;
    }
    *dato = uart_rx[uart_rx_cola];
    uart_rx_cola = (uint8_t)((uart_rx_cola + 1U) & (UART_RX_TAM - 1U));
    return 1U;
}

static uint8_t uart_tx_terminada(void)
{
    return (uart_tx_cola == uart_tx_cabeza &&
            (!uart_tx_iniciada || (UCSR0A & _BV(TXC0)) != 0U)) ? 1U : 0U;
}

static void uart_informar_estado(void)
{
    if (estado == ESTADO_ACTIVO) {
        (void)uart_enviar_texto("ESTADO:ACTIVO\r\n");
    } else if (estado == ESTADO_STANDBY) {
        (void)uart_enviar_texto("ESTADO:STANDBY\r\n");
    } else if (estado == ESTADO_APAGADO) {
        (void)uart_enviar_texto("ESTADO:APAGADO\r\n");
    } else {
        (void)uart_enviar_texto("ESTADO:SEGURO\r\n");
    }
}

/* =========================================================
   GPIO Y TEMPORIZADORES
   ========================================================= */

static void gpio_init(void)
{
    ACSR = _BV(ACD);                /* Comparador analogico no usado. */
    PRR |= _BV(PRSPI);              /* SPI no se usa en esta aplicacion. */
    DIDR0 |= _BV(ADC0D);            /* Evita entrada digital innecesaria en A0. */
    DDRB |= (1U << DDB5) | (1U << DDB3);
    PORTB &= (uint8_t)~((1U << LED_PIN) | (1U << SOUNDER_PIN));

    DDRD &= (uint8_t)~(1U << DDD4);
    PORTD |= (1U << DHT_PIN);

    DDRD &= (uint8_t)~(1U << DDD2);
    PORTD |= (1U << BOTON_PIN);

    EICRA = (uint8_t)((EICRA &
             (uint8_t)~((1U << ISC01) | (1U << ISC00))) |
             (1U << ISC01));
    EIFR = (1U << INTF0);
    EIMSK |= (1U << INT0);
}

static void timer0_init(void)
{
    PRR &= (uint8_t)~_BV(PRTIM0);
    TIFR0 = _BV(OCF0A) | _BV(OCF0B) | _BV(TOV0);
    /* 16 MHz / 64 = 250 kHz; 250 cuentas = 1 ms. */
    TCCR0A = (1U << WGM01);
    TCCR0B = (1U << CS01) | (1U << CS00);
    OCR0A = 249U;
    TCNT0 = 0U;
    TIMSK0 = (1U << OCIE0A);
}

static void timer1_init(void)
{
    PRR &= (uint8_t)~_BV(PRTIM1);
    TIFR1 = _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);
    /* 16 MHz / 64 = 250 kHz; 25000 cuentas = 100 ms. */
    TCCR1A = 0U;
    TCCR1B = (1U << WGM12) | (1U << CS11) | (1U << CS10);
    OCR1A = 24999U;
    TCNT1 = 0U;
    TIMSK1 = (1U << OCIE1A);
}

static void timer2_init(void)
{
    PRR &= (uint8_t)~_BV(PRTIM2);
    /* Modo normal, prescaler 8: 0,5 us por cuenta. */
    TCCR2A = 0U;
    TCCR2B = (1U << CS21);
    TCNT2 = 0U;
    TIMSK2 = 0U;
    TIFR2 = (1U << TOV2);
}

/* =========================================================
   ADC Y ACTIVIDAD DEL POTENCIOMETRO
   ========================================================= */

static void adc_habilitar(void)
{
    PRR &= (uint8_t)~_BV(PRADC);
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
    adc_habilitar();
}

static uint16_t adc_leer_a0(void)
{
    if ((ADCSRA & (1U << ADEN)) == 0U) {
        adc_habilitar();
    }

    ADMUX = (1U << REFS0);
    ADCSRA |= (1U << ADSC);

    while ((ADCSRA & (1U << ADSC)) != 0U) {
        /* La conversion dura aproximadamente 104 us. */
    }

    return ADC;
}

static void revisar_actividad_adc(uint16_t nueva_lectura, uint32_t ahora)
{
#if POTENCIOMETRO_ES_ACTIVIDAD
    uint16_t diferencia;

    if (!referencia_adc_valida) {
        referencia_adc_actividad = nueva_lectura;
        referencia_adc_valida = 1U;
        return;
    }

    diferencia = (nueva_lectura > referencia_adc_actividad)
        ? (uint16_t)(nueva_lectura - referencia_adc_actividad)
        : (uint16_t)(referencia_adc_actividad - nueva_lectura);

    if (diferencia >= UMBRAL_ACTIVIDAD_ADC) {
        referencia_adc_actividad = nueva_lectura;
        ultima_actividad = ahora;
    }
#else
    (void)nueva_lectura;
    (void)ahora;
#endif
}

/* =========================================================
   TWI / LCD 16x2 CON PCF8574
   ========================================================= */

static uint8_t twi_esperar(void)
{
    uint16_t timeout;

    for (timeout = 0U; timeout < 3000U; timeout++) {
        if ((TWCR & (1U << TWINT)) != 0U) {
            return 1U;
        }
        _delay_us(1);
    }

    return 0U;
}

static void twi_init(void)
{
    PRR &= (uint8_t)~_BV(PRTWI);
    DDRC &= (uint8_t)~((1U << DDC4) | (1U << DDC5));
    PORTC |= (1U << PORTC4) | (1U << PORTC5);

    TWSR = 0U;
    TWBR = (uint8_t)(((F_CPU / 100000UL) - 16UL) / 2UL);
    TWCR = (1U << TWEN);
}

static uint8_t twi_iniciar(uint8_t direccion_escritura)
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

    TWDR = direccion_escritura;
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
    uint16_t timeout;

    TWCR = (1U << TWINT) | (1U << TWEN) | (1U << TWSTO);

    for (timeout = 0U; timeout < 3000U; timeout++) {
        if ((TWCR & (1U << TWSTO)) == 0U) {
            break;
        }
        _delay_us(1);
    }
}

static uint8_t lcd_expansor(uint8_t valor)
{
    uint8_t correcto;

    correcto = twi_iniciar((uint8_t)(direccion_lcd << 1));
    if (correcto) {
        correcto = twi_enviar((uint8_t)(valor | luz_lcd));
    }
    twi_stop();

    return correcto;
}

static void lcd_encender_luz(void)
{
    if (!lcd_disponible) {
        return;
    }
    luz_lcd = LCD_LUZ;
    (void)lcd_expansor(0U);
}

static void lcd_apagar_luz(void)
{
    if (!lcd_disponible) {
        return;
    }
    luz_lcd = 0U;
    (void)lcd_expansor(0U);
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

    if (!lcd_medio_byte((uint8_t)(((valor << 4) & 0xF0U) | control))) {
        return 0U;
    }

    return 1U;
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
    if (!lcd_disponible) {
        return;
    }

    if (!lcd_byte(dato, 1U)) {
        lcd_disponible = 0U;
    }
}

static uint8_t lcd_init(void)
{
    static const uint8_t direcciones_posibles[] = {0x27U, 0x3FU};
    uint8_t indice;
    uint8_t encontrado = 0U;

    _delay_ms(50);

    for (indice = 0U;
         indice < (uint8_t)sizeof(direcciones_posibles);
         indice++) {
        if (twi_iniciar((uint8_t)(direcciones_posibles[indice] << 1))) {
            direccion_lcd = direcciones_posibles[indice];
            encontrado = 1U;
            twi_stop();
            break;
        }
        twi_stop();
    }

    if (!encontrado) {
        return 0U;
    }

    lcd_disponible = 1U;

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

static void mostrar_activo(void)
{
    char linea1[17];
    char linea2[17];

    linea_limpiar(linea1);
    linea_limpiar(linea2);

    linea_texto(linea1, 0U, "ACTIVO ADC:");
    linea_numero(linea1, 11U, valor_adc_actual, 4U);

    if (lectura_dht_correcta) {
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

static void mostrar_standby(void)
{
    char linea1[17];
    char linea2[17];

    linea_limpiar(linea1);
    linea_limpiar(linea2);
    linea_texto(linea1, 0U, "STANDBY");
    linea_texto(linea2, 0U, "Esperando...");
    lcd_linea(0U, linea1);
    lcd_linea(1U, linea2);
}

static void mostrar_apagado(void)
{
    char linea1[17];
    char linea2[17];

    linea_limpiar(linea1);
    linea_limpiar(linea2);
    linea_texto(linea1, 0U, "APAGADO");
    linea_texto(linea2, 0U, "Pulse el boton");
    lcd_linea(0U, linea1);
    lcd_linea(1U, linea2);
}

static void mostrar_seguro(void)
{
    char linea1[17];
    char linea2[17];

    linea_limpiar(linea1);
    linea_limpiar(linea2);
    linea_texto(linea1, 0U, "EMERGENCIA");
    linea_texto(linea2, 0U, "Pulse para salir");
    lcd_linea(0U, linea1);
    lcd_linea(1U, linea2);
}

/* =========================================================
   DHT11 NO BLOQUEANTE: FSM + PCINT20 + TIMER2
   ========================================================= */

static void liberar_dht(void)
{
    DDRD &= (uint8_t)~(1U << DDD4);
    PORTD |= (1U << DHT_PIN);
}

static void cancelar_dht(void)
{
    uint8_t sreg = SREG;

    cli();
    PCMSK2 &= (uint8_t)~(1U << PCINT20);
    estado_dht = DHT_REPOSO;
    dht_error_isr = 0U;
    SREG = sreg;

    liberar_dht();
    bandera_dht = 0U;
}

static void dht_comenzar(uint32_t ahora)
{
    uint8_t sreg = SREG;

    cli();
    PCMSK2 &= (uint8_t)~(1U << PCINT20);
    dht_datos[0] = 0U;
    dht_datos[1] = 0U;
    dht_datos[2] = 0U;
    dht_datos[3] = 0U;
    dht_datos[4] = 0U;
    dht_indice_bit = 0U;
    dht_error_isr = 0U;
    estado_dht = DHT_INICIO_BAJO;
    SREG = sreg;

    /* Pulso inicial sin espera activa: el main volvera despues de 20 ms. */
    DDRD |= (1U << DDD4);
    PORTD &= (uint8_t)~(1U << DHT_PIN);
    dht_marca_ms = ahora;
}

static void dht_liberar_y_capturar(uint32_t ahora)
{
    uint8_t sreg = SREG;

    cli();
    liberar_dht();

    dht_nivel_anterior =
        ((PIND & (1U << DHT_PIN)) != 0U) ? 1U : 0U;
    TCNT2 = 0U;
    PCIFR = (1U << PCIF2);
    estado_dht = DHT_ESPERA_RESPUESTA_BAJA;
    PCICR |= (1U << PCIE2);
    PCMSK2 |= (1U << PCINT20);
    SREG = sreg;

    dht_marca_ms = ahora;
}

static void dht_procesar_resultado(void)
{
    uint8_t datos[5];
    uint8_t sreg = SREG;

    cli();
    datos[0] = dht_datos[0];
    datos[1] = dht_datos[1];
    datos[2] = dht_datos[2];
    datos[3] = dht_datos[3];
    datos[4] = dht_datos[4];
    estado_dht = DHT_REPOSO;
    SREG = sreg;

    if ((uint8_t)(datos[0] + datos[1] + datos[2] + datos[3]) ==
        datos[4]) {
        humedad = datos[0];
        temperatura = datos[2];
        lectura_dht_correcta = 1U;
        error_dht = 0U;
    } else {
        lectura_dht_correcta = 0U;
        error_dht = 6U;
    }

    bandera_lcd = 1U;
}

static void tarea_dht(uint32_t ahora)
{
    uint8_t copia_estado;
    uint8_t copia_error;
    uint8_t sreg;

    if (estado != ESTADO_ACTIVO) {
        cancelar_dht();
        return;
    }

    sreg = SREG;
    cli();
    copia_estado = estado_dht;
    copia_error = dht_error_isr;
    SREG = sreg;

    if (bandera_dht && copia_estado == DHT_REPOSO) {
        bandera_dht = 0U;
        dht_comenzar(ahora);
    } else if (copia_estado == DHT_INICIO_BAJO &&
               (uint32_t)(ahora - dht_marca_ms) >= DHT_INICIO_BAJO_MS) {
        dht_liberar_y_capturar(ahora);
    } else if (copia_estado == DHT_COMPLETO) {
        dht_procesar_resultado();
    } else if (copia_estado == DHT_ERROR) {
        lectura_dht_correcta = 0U;
        error_dht = copia_error != 0U ? copia_error : 7U;
        cancelar_dht();
        bandera_lcd = 1U;
    } else if (copia_estado >= DHT_ESPERA_RESPUESTA_BAJA &&
               copia_estado <= DHT_BIT_ALTO &&
               (uint32_t)(ahora - dht_marca_ms) >= DHT_TIMEOUT_MS) {
        lectura_dht_correcta = 0U;
        error_dht = 7U;
        cancelar_dht();
        bandera_lcd = 1U;
    }
}


/* =========================================================
   RELOJ Y RESTAURACION DE PERIFERICOS - ACTIVIDAD 1
   ========================================================= */

/* Llamar con interrupciones deshabilitadas. Conserva la fraccion transcurrida. */
static void reloj_capturar_y_detener(void)
{
    uint32_t parcial_us = fraccion_reloj_us;

    if (modo_reloj == RELOJ_RAPIDO) {
        TCCR0B = 0U;
        if ((TIFR0 & _BV(OCF0A)) != 0U) {
            tiempo_ms++;           /* Compare ocurrido, aun sin atender su ISR. */
        }
        parcial_us += (uint32_t)TCNT0 * 4UL;
    } else if (modo_reloj == RELOJ_STANDBY) {
        TCCR1B = _BV(WGM12);        /* CTC se conserva; reloj detenido. */
        if ((TIFR1 & _BV(OCF1A)) != 0U) {
            tiempo_ms += PASO_STANDBY_MS;
        }
        parcial_us += (uint32_t)TCNT1 * 4UL;
    }
    tiempo_ms += parcial_us / 1000UL;
    fraccion_reloj_us = (uint16_t)(parcial_us % 1000UL);
    TCCR0B = 0U;
    TCCR1B = _BV(WGM12);
    TIMSK0 = 0U;
    TIMSK1 = 0U;
    TIFR0 = _BV(OCF0A) | _BV(OCF0B) | _BV(TOV0);
    TIFR1 = _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);
    modo_reloj = RELOJ_DETENIDO;
}

static void reloj_rapido(void)
{
    uint8_t sreg = SREG;
    cli();
    if (modo_reloj != RELOJ_RAPIDO) {
        reloj_capturar_y_detener();
        contador_led_ms = 0U;
        contador_lcd_ms = 0U;
        contador_dht_ms = 0U;
        bandera_reloj = 0U;
        modo_reloj = RELOJ_RAPIDO;
        timer0_init();
        timer1_init();
        timer2_init();
    }
    SREG = sreg;
}

static void reloj_standby(void)
{
    uint8_t sreg = SREG;
    cli();
    reloj_capturar_y_detener();
    bandera_reloj = 0U;
    modo_reloj = RELOJ_STANDBY;
    timer1_init();                 /* Solo 10 IRQ/s para contar la inactividad. */
    TCCR2B = 0U;
    PRR |= _BV(PRTIM0) | _BV(PRTIM2) | _BV(PRADC) | _BV(PRSPI);
    SREG = sreg;
}

static void restaurar_para_activo(void)
{
    reloj_rapido();
    twi_init();                    /* Antes de intentar escribir a la LCD. */
    adc_habilitar();
}

static void int0_flanco_descendente(void)
{
    EIMSK &= (uint8_t)~_BV(INT0);
    EICRA = (uint8_t)((EICRA & (uint8_t)~(_BV(ISC01) | _BV(ISC00))) |
                     _BV(ISC01));
    EIFR = _BV(INTF0);
    EIMSK |= _BV(INT0);
}

/* Esta comprobacion se llama con IRQ deshabilitadas. */
static uint8_t hay_evento_pendiente(void)
{
    return (bandera_boton || bandera_reloj || uart_rx_error ||
            uart_rx_cola != uart_rx_cabeza || (UCSR0A & _BV(RXC0)) != 0U ||
            (PIND & _BV(BOTON_PIN)) == 0U) ? 1U : 0U;
}

static void dormir_idle(void)
{
    uint8_t sreg = SREG;
    cli();
    if (estado == ESTADO_STANDBY && !hay_evento_pendiente()) {
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_enable();
        sei();
        sleep_cpu();               /* SEI y SLEEP deben ser consecutivos. */
        sleep_disable();
    }
    SREG = sreg;
}

static void dormir_powerdown(void)
{
    uint8_t sreg;
    uint8_t prr_anterior;
    uint8_t usart_anterior;

    if (!uart_tx_terminada()) {
        return;                    /* Termina de enviar el aviso sin bloquear main. */
    }
    sreg = SREG;
    cli();
    if (estado != ESTADO_APAGADO || hay_evento_pendiente() || !uart_tx_terminada()) {
        SREG = sreg;
        return;
    }

    /* INT0 por nivel bajo: fuente asincrona valida en POWER-DOWN. */
    EIMSK &= (uint8_t)~_BV(INT0);
    EICRA &= (uint8_t)~(_BV(ISC01) | _BV(ISC00));
    EIFR = _BV(INTF0);
    en_powerdown = 1U;
    EIMSK |= _BV(INT0);
    if ((PIND & _BV(BOTON_PIN)) == 0U) {
        bandera_boton = 1U;         /* Pulsacion llegada antes de dormir. */
        en_powerdown = 0U;
        int0_flanco_descendente();
        SREG = sreg;
        return;
    }

    prr_anterior = PRR;
    usart_anterior = UCSR0B;
    reloj_capturar_y_detener();
    adc_deshabilitar();
    UCSR0B = 0U;                    /* RX, TX y sus IRQ quedan inactivos. */
    uart_linea_longitud = 0U;        /* No unir una trama vieja con datos del despertar. */
    uart_descartar_linea = 0U;
    TWCR = 0U;                     /* La ultima escritura LCD ya termino. */
    PCMSK2 &= (uint8_t)~_BV(PCINT20);
    PCICR &= (uint8_t)~_BV(PCIE2);
    TCCR2B = 0U;
    TIMSK2 = 0U;
    PRR |= PRR_DORMIR;             /* Mascara explicita, nunca 0xFF. */

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();
    sleep_cpu();                   /* El pulsador debe mantenerse durante el arranque. */
    cli();
    sleep_disable();

    /* Se continua aqui tras el despertar; la SRAM conserva la FSM. */
    PRR = prr_anterior;
    UCSR0B = usart_anterior;
    reloj_standby();               /* Reloj disponible antes de validar el evento. */
    en_powerdown = 0U;
    int0_flanco_descendente();
    SREG = sreg;
    /* main valida el boton y entrar_activo restaura el resto. */
}

static void gestionar_sueno(void)
{
    if (estado == ESTADO_STANDBY) {
        dormir_idle();
    } else if (estado == ESTADO_APAGADO) {
        dormir_powerdown();
    }
    /* En SEGURO se conserva el reloj rapido y la alarma original. */
}

/* =========================================================
   FSM DEL SISTEMA - SIN SWITCH-CASE
   ========================================================= */

static void entrar_activo(uint32_t ahora, uint8_t registrar_actividad)
{
    alarma_continua = 0U;
    restaurar_para_activo();
    lcd_encender_luz();
    lcd_comando(0x0CU);

    estado = ESTADO_ACTIVO;
    PORTB &= (uint8_t)~(1U << LED_PIN);

    if (registrar_actividad) {
        ahora = leer_tiempo();      /* Incluye fraccion al salir del reloj lento. */
        ultima_actividad = ahora;
    }

    (void)uart_enviar_texto("EV:ACTIVO\r\n");
    bandera_adc = 1U;
    bandera_lcd = 1U;
    bandera_dht = 1U;
}

static void entrar_standby(void)
{
    estado = ESTADO_STANDBY;
    adc_deshabilitar();
    cancelar_dht();
    PORTB &= (uint8_t)~(1U << LED_PIN);
    lcd_encender_luz();
    lcd_comando(0x0CU);
    mostrar_standby();
    apagar_sonido();
    reloj_standby();
    TWCR = 0U;
    PRR |= _BV(PRTWI);              /* El PCF8574 conserva el mensaje y la luz. */
    (void)uart_enviar_texto("EV:STANDBY\r\n");
}

static void entrar_apagado(void)
{
    estado = ESTADO_APAGADO;
    adc_deshabilitar();
    cancelar_dht();
    apagar_sonido();
    PORTB &= (uint8_t)~(1U << LED_PIN);

    twi_init();                    /* TWI estaba detenido en STANDBY. */
    lcd_comando(0x0CU);
    mostrar_apagado();
    _delay_ms(5);
    lcd_comando(0x08U);
    lcd_apagar_luz();
    (void)uart_enviar_texto("EV:APAGADO\r\n");
}

static void entrar_seguro(void)
{
    restaurar_para_activo();       /* La alarma necesita Timer0 a 1 ms. */
    estado = ESTADO_SEGURO;
    adc_deshabilitar();
    cancelar_dht();
    lcd_encender_luz();
    lcd_comando(0x0CU);
    mostrar_seguro();
    iniciar_alarma();
    (void)uart_enviar_texto("EV:SEGURO\r\n");
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

/* =========================================================
   PULSADOR
   ========================================================= */

static void procesar_boton(uint32_t ahora)
{
    uint8_t sreg = SREG;
    uint8_t evento;
    uint8_t salia_de_seguro;

    cli();
    evento = bandera_boton;
    bandera_boton = 0U;
    SREG = sreg;
    if (!evento || boton_presionado || (PIND & _BV(BOTON_PIN)) != 0U) {
        return;
    }
    if ((uint32_t)(ahora - ultimo_boton) < ANTI_REBOTE_MS) {
        return;
    }

    salia_de_seguro = (estado == ESTADO_SEGURO) ? 1U : 0U;
    if (salia_de_seguro) {
        apagar_sonido();
    }
    if (estado != ESTADO_ACTIVO) {
        entrar_activo(ahora, 1U);
    }
    ahora = leer_tiempo();          /* Reloj rapido ya restaurado. */
    ultima_actividad = ahora;
    ultimo_boton = ahora;
    inicio_boton = ahora;
    boton_presionado = 1U;
    pulsacion_larga_lista = salia_de_seguro;
    iniciar_sonido(DURACION_SOUNDER_MS);
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
        (uint32_t)(ahora - inicio_boton) >=
            PULSACION_EMERGENCIA_MS) {
        pulsacion_larga_lista = 1U;
        entrar_seguro();
    }
}


/* =========================================================
   COMANDOS MINIMOS DE PRUEBA - PROCESAMIENTO FUERA DE LAS ISR
   Una linea valida es actividad; bytes sueltos/errores no lo son.
   ========================================================= */

static void uart_ejecutar_linea(uint32_t ahora)
{
    if (strcmp(uart_linea, "EMERGENCIA") == 0) {
        ultima_actividad = ahora;
        if (estado != ESTADO_SEGURO) {
            entrar_seguro();
        }
        (void)uart_enviar_texto("OK:EMERGENCIA\r\n");
    } else if (strcmp(uart_linea, "ACTIVO") == 0 ||
               strcmp(uart_linea, "ESTADO") == 0) {
        if (estado == ESTADO_SEGURO) {
            if (strcmp(uart_linea, "ESTADO") == 0) {
                uart_informar_estado();
            } else {
                (void)uart_enviar_texto("BLOQUEADO:EMERGENCIA\r\n");
            }
        } else {
            if (estado != ESTADO_ACTIVO) {
                entrar_activo(ahora, 1U);
            }
            ultima_actividad = leer_tiempo();
            uart_informar_estado();
        }
    } else {
        (void)uart_enviar_texto("ERROR:COMANDO\r\n");
    }
}

static void procesar_uart(uint32_t ahora)
{
    uint8_t dato;
    uint8_t cantidad;
    uint8_t sreg = SREG;
    uint8_t error;

    cli();
    error = uart_rx_error;
    uart_rx_error = 0U;
    SREG = sreg;
    if (error) {
        uart_linea_longitud = 0U;
        uart_descartar_linea = 1U;
        (void)uart_enviar_texto("ERROR:RX\r\n");
    }
    if (uart_linea_longitud != 0U &&
        (uint32_t)(ahora - uart_inicio_linea) >= UART_TIMEOUT_LINEA_MS) {
        uart_linea_longitud = 0U;
        uart_descartar_linea = 1U;   /* Descarta hasta LF: evita aceptar un sufijo. */
        (void)uart_enviar_texto("ERROR:TIMEOUT\r\n");
    }

    for (cantidad = 0U; cantidad < UART_RX_TAM && uart_tomar_byte(&dato); cantidad++) {
        if (dato == '\r') {
            continue;
        }
        if (dato == '\n') {
            if (!uart_descartar_linea && uart_linea_longitud != 0U) {
                uart_linea[uart_linea_longitud] = '\0';
                uart_ejecutar_linea(leer_tiempo());
            }
            uart_linea_longitud = 0U;
            uart_descartar_linea = 0U;
        } else if (!uart_descartar_linea) {
            if (uart_linea_longitud < UART_LINEA_TAM - 1U) {
                if (uart_linea_longitud == 0U) {
                    uart_inicio_linea = leer_tiempo();
                }
                uart_linea[uart_linea_longitud++] = (char)dato;
            } else {
                uart_linea_longitud = 0U;
                uart_descartar_linea = 1U;
                (void)uart_enviar_texto("ERROR:LONGITUD\r\n");
            }
        }
    }
}

/* =========================================================
   TAREAS COOPERATIVAS
   ========================================================= */

static void ejecutar_tareas(uint32_t ahora)
{
    if (bandera_led) {
        bandera_led = 0U;
        PORTB ^= (1U << LED_PIN);
    }

    if (bandera_adc) {
        bandera_adc = 0U;
        valor_adc_actual = adc_leer_a0();
        revisar_actividad_adc(valor_adc_actual, ahora);
    }

    tarea_dht(ahora);

    if (bandera_lcd) {
        bandera_lcd = 0U;
        mostrar_activo();
    }
}

static void descartar_tareas(void)
{
    bandera_adc = 0U;
    bandera_dht = 0U;
    bandera_lcd = 0U;

    if (bandera_led) {
        bandera_led = 0U;

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
    timer2_init();
    twi_init();
    (void)lcd_init();

    if (lcd_disponible) {
        char mensaje1[17];
        char mensaje2[17];

        linea_limpiar(mensaje1);
        linea_limpiar(mensaje2);
        linea_texto(mensaje1, 0U, "LAB 5 - B ACT 1");
        linea_texto(mensaje2, 0U, "INICIANDO...");
        lcd_linea(0U, mensaje1);
        lcd_linea(1U, mensaje2);
        _delay_ms(800);
    }

    timer0_init();
    timer1_init();

    valor_adc_actual = adc_leer_a0();
    referencia_adc_actividad = valor_adc_actual;
    referencia_adc_valida = 1U;
    ultima_actividad = 0UL;
    mostrar_activo();
    uart_init();
    (void)uart_enviar_texto("LAB5:B1\r\nEV:ACTIVO\r\n");

    sei();

    while (1) {
        /* Consumir tick y copiar tiempo en una sola seccion atomica. */
        uint8_t sreg = SREG;
        cli();
        ahora = tiempo_ms;
        bandera_reloj = 0U;
        SREG = sreg;

        /* Respaldo si el boton ya estaba bajo al habilitar una fuente. */
        if (!boton_presionado && (PIND & _BV(BOTON_PIN)) == 0U) {
            bandera_boton = 1U;
        }
        procesar_boton(ahora);
        procesar_uart(leer_tiempo());
        ahora = leer_tiempo();
        revisar_boton_mantenido(ahora);

        if (estado != ESTADO_SEGURO) {
            revisar_inactividad(ahora);
        }

        if (estado == ESTADO_ACTIVO) {
            ejecutar_tareas(ahora);
        } else {
            descartar_tareas();
        }
        gestionar_sueno();         /* Solo duerme en STANDBY o APAGADO. */
    }
}
