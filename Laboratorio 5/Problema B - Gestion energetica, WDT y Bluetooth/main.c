/*
 * LABORATORIO 5 - PROBLEMA B / ACTIVIDADES 1, 2 Y 3
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
 * - El protocolo completo de Bluetooth se desarrolla despues.
 *
 * Actividad 2:
 * - PRUEBA 1 ACTIVA: WDT solo interrupcion, 8 s nominales en POWER-DOWN.
 * - Tras despertar: registrar evento, leer ADC A0 y volver a dormir.
 * - PRUEBA 2 COMENTADA: WDT reset a 2 s y bloqueo por comando BLOQUEO.
 * - Seleccion MANUAL: comentar COMPLETA la PRUEBA 1 y descomentar
 *   COMPLETA la PRUEBA 2; recompilar despues del cambio.
 * - Para comentar PRUEBA 1, sustituir sus marcadores // de inicio/fin
 *   por una apertura y cierre de comentario de bloque (ver guia).
 * - D12/PB4 genera un pulso por WDT; medir entre flancos ascendentes.
 * - tiempo_ms NO mide el tiempo dormido ni se le suman 8000 ms inventados.
 * - Se conserva la FSM original, incluido ESTADO_SEGURO.
 *
 * Actividad 3:
 * - PRR centralizado, mascaras explicitas y comprobacion de lectura posterior.
 * - Se rechaza apagar un reloj necesario o un periferico ocupado.
 * - Timer2 solo se habilita durante la captura de pulsos del DHT11.
 * - UART: PRR + LF consulta el registro; avisos al cambiar de estado.
 * - El bit reservado 4 nunca se solicita. 1=sin reloj; 0=con reloj.
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
#include <avr/wdt.h>

#if F_CPU != 16000000UL
#error "Esta version requiere revisar los temporizadores si F_CPU no es 16 MHz"
#endif

#define LED_PIN                         PB5
#define SOUNDER_PIN                     PB3
#define DHT_PIN                         PD4
#define BOTON_PIN                       PD2
#define WDT_PRUEBA_PIN                  PB4 /* D12: marca observable del WDT. */

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

/* ACTIVIDAD 3: bits definidos del ATmega328P. Nunca usar PRR=0xFF. */
#define PRR_ADC        _BV(PRADC)       /* 0x01 */
#define PRR_USART      _BV(PRUSART0)    /* 0x02 */
#define PRR_SPI        _BV(PRSPI)       /* 0x04 */
#define PRR_TIMER1     _BV(PRTIM1)      /* 0x08 */
#define PRR_TIMER0     _BV(PRTIM0)      /* 0x20 */
#define PRR_TIMER2     _BV(PRTIM2)      /* 0x40 */
#define PRR_TWI        _BV(PRTWI)       /* 0x80 */
#define PRR_VALIDOS    (PRR_ADC | PRR_USART | PRR_SPI | PRR_TIMER1 | \
                        PRR_TIMER0 | PRR_TIMER2 | PRR_TWI) /* 0xEF */
#define PRR_ACTIVO     (PRR_SPI | PRR_TIMER2)             /* 0x44 */
#define PRR_STANDBY    (PRR_ADC | PRR_SPI | PRR_TIMER0 | PRR_TIMER2 | PRR_TWI)
                                                       /* 0xE5 */
#define PRR_SEGURO     (PRR_ADC | PRR_SPI | PRR_TIMER1 | PRR_TIMER2 | PRR_TWI)
                                                       /* 0xCD */
#define PRR_DORMIR     PRR_VALIDOS                        /* 0xEF */

#define TWI_LIBRE      0U
#define TWI_TRANSFIRIENDO 1U
#define TWI_STOP_PENDIENTE 2U
/* Diagnostico: ultimo rechazo conservado hasta el siguiente rechazo. */
#define PRR_OK        0U
#define PRR_ERR_BIT    1U
#define PRR_ERR_RELOJ  2U
#define PRR_ERR_DHT    3U
#define PRR_ERR_ADC    4U
#define PRR_ERR_UART   5U
#define PRR_ERR_TWI    6U
#define PRR_ERR_SPI    7U
#define PRR_ERR_LECTURA 8U

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

/* Diagnostico PRR en SRAM; no es una medicion de consumo. */
static uint8_t twi_fase = TWI_LIBRE;
static volatile uint8_t prr_ultimo_error = PRR_OK;
static volatile uint16_t prr_rechazos = 0U;
static volatile uint16_t prr_cambios = 0U;
static volatile uint8_t prr_ultimo_sueno = 0U;
/* UART se inicializa antes de la definicion de la interfaz PRR. */
static void prr_habilitar(uint8_t mascara);

/* Causa de reset: .noinit evita que la inicializacion de C borre esta copia. */
uint8_t causa_reinicio __attribute__((section(".noinit")));
/* Solo indica que habia un bloqueo de ensayo pendiente; NO prueba su causa. */
uint32_t marca_bloqueo __attribute__((section(".noinit")));
uint32_t marca_bloqueo_inversa __attribute__((section(".noinit")));
#define MARCA_BLOQUEO_WDT 0x57445432UL
static volatile uint8_t bandera_wdt = 0U;
static volatile uint32_t eventos_wdt = 0UL;
static uint8_t wdt_armado = 0U;

/* Interfaces de las pruebas; sus implementaciones se seleccionan al final. */
static void wdt_iniciar_prueba(void);
static void wdt_antes_powerdown(void);
static void wdt_despues_powerdown(void);
static void wdt_al_volver_a_ejecucion(void);
static void wdt_procesar_evento(void);
static void wdt_supervisar_ciclo(void);
static uint8_t wdt_solicitar_bloqueo(void);

/* Antes de .data/.bss: guardar MCUSR y detener el WDT que persista tras reset. */
void wdt_capturar_reset_temprano(void)
    __attribute__((naked, used, section(".init3")));

void wdt_capturar_reset_temprano(void)
{
    causa_reinicio = MCUSR;
    MCUSR = 0U;                    /* Limpiar WDRF ANTES de deshabilitar WDT. */
    wdt_disable();                 /* La macro respeta la secuencia protegida. */
    /* Sin return: .init3 continua con la inicializacion normal de C. */
}

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

ISR(WDT_vect)
{
    bandera_wdt = 1U;              /* Solo notifica; ADC y UART se atienden en main. */
    eventos_wdt++;
    PORTB |= _BV(WDT_PRUEBA_PIN);   /* Inicio de pulso para osciloscopio. */
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
    prr_habilitar(_BV(PRUSART0));
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
   ACTIVIDAD 3 - PRR: VALIDAR, DETENER Y DESPUES QUITAR EL RELOJ
   Todas las escrituras a PRR pasan por estas funciones.
   Las ISR no cambian PRR. Las comprobaciones y el cambio son atomicos.
   ========================================================= */

static void prr_registrar_error(uint8_t codigo)
{
    prr_ultimo_error = codigo;
    prr_rechazos++;
}

/* Solo cambia el registro; el llamador ya verifico y preparo los modulos. */
static uint8_t prr_escribir(uint8_t valor)
{
    valor &= PRR_VALIDOS;
    if (PRR != valor) {
        PRR = valor;
        prr_cambios++;
    }
    if (PRR != valor) {
        prr_registrar_error(PRR_ERR_LECTURA);
        return 0U;
    }
    return 1U;
}

/* Quitar el bit PRR habilita el RELOJ; el driver configura luego el modulo. */
static void prr_habilitar(uint8_t mascara)
{
    uint8_t sreg = SREG;
    cli();
    if ((mascara & (uint8_t)~PRR_VALIDOS) != 0U) {
        prr_registrar_error(PRR_ERR_BIT);
    } else {
        (void)prr_escribir((uint8_t)(PRR & (uint8_t)~mascara));
    }
    SREG = sreg;
}

/* Retorna un motivo de rechazo SIN detener ningun periferico. */
static uint8_t prr_validar(uint8_t destino)
{
    uint8_t nuevos = destino & (uint8_t)~PRR;
    if ((destino & (uint8_t)~PRR_VALIDOS) != 0U) {
        return PRR_ERR_BIT;
    }
    if (((destino & PRR_TIMER0) != 0U && modo_reloj == RELOJ_RAPIDO) ||
        ((destino & PRR_TIMER1) != 0U &&
         (modo_reloj == RELOJ_STANDBY ||
          (estado == ESTADO_ACTIVO && modo_reloj != RELOJ_DETENIDO)))) {
        return PRR_ERR_RELOJ;       /* Scheduler/ADC de ACTIVO o reloj de IDLE. */
    }
    if ((destino & PRR_TIMER2) != 0U &&
        estado_dht >= DHT_ESPERA_RESPUESTA_BAJA && estado_dht <= DHT_BIT_ALTO) {
        return PRR_ERR_DHT;
    }
    if ((nuevos & PRR_ADC) != 0U && (ADCSRA & _BV(ADSC)) != 0U) {
        return PRR_ERR_ADC;
    }
    if ((nuevos & PRR_USART) != 0U &&
        (estado != ESTADO_APAGADO || !uart_tx_terminada() ||
         uart_rx_cabeza != uart_rx_cola || uart_rx_error ||
         uart_linea_longitud != 0U || (UCSR0A & _BV(RXC0)) != 0U)) {
        return PRR_ERR_UART;
    }
    if ((nuevos & PRR_TWI) != 0U) {
        if (twi_fase == TWI_STOP_PENDIENTE && (TWCR & _BV(TWSTO)) == 0U) {
            twi_fase = TWI_LIBRE;  /* STOP termino despues de su espera limitada. */
        }
        if (twi_fase != TWI_LIBRE || (TWCR & _BV(TWSTO)) != 0U) {
            return PRR_ERR_TWI;
        }
    }
    /* SPI no se usa: si otro desarrollo lo habilita, no apagarlo a ciegas. */
    if ((nuevos & PRR_SPI) != 0U && (SPCR & _BV(SPE)) != 0U) {
        return PRR_ERR_SPI;
    }
    return PRR_OK;
}

static uint8_t prr_fijar_perfil(uint8_t destino)
{
    uint8_t sreg = SREG;
    uint8_t nuevos;
    uint8_t error;
    uint8_t resultado;
    cli();
    error = prr_validar(destino);
    if (error != PRR_OK) {
        prr_registrar_error(error);
        SREG = sreg;
        return 0U;                 /* Ningun apagado parcial si algo esta ocupado. */
    }
    nuevos = destino & (uint8_t)~PRR;
    /* No acceder a registros de modulos cuyo reloj ya estaba quitado. */
    if ((nuevos & PRR_ADC) != 0U) ADCSRA &= (uint8_t)~_BV(ADEN);
    if ((nuevos & PRR_USART) != 0U) UCSR0B = 0U;
    if ((nuevos & PRR_TIMER0) != 0U) { TCCR0B = 0U; TIMSK0 = 0U; }
    if ((nuevos & PRR_TIMER1) != 0U) { TCCR1B = _BV(WGM12); TIMSK1 = 0U; }
    if ((nuevos & PRR_TIMER2) != 0U) { TCCR2B = 0U; TIMSK2 = 0U; }
    if ((nuevos & PRR_TWI) != 0U) TWCR = 0U;
    resultado = prr_escribir(destino);
    SREG = sreg;
    return resultado;
}

static uint8_t prr_deshabilitar(uint8_t mascara)
{
    /* Validar primero la mascara original: no ocultar un bit reservado. */
    uint8_t sreg = SREG;
    uint8_t resultado;
    cli();
    resultado = prr_fijar_perfil((uint8_t)(PRR | mascara));
    SREG = sreg;
    return resultado;
}

static char prr_digito_hex(uint8_t valor)
{
    valor &= 0x0FU;
    return valor < 10U ? (char)('0' + valor) : (char)('A' + valor - 10U);
}

static void prr_informar_valor(const char *etiqueta, uint8_t valor)
{
    char mensaje[48];
    uint8_t i = 4U;
    strcpy(mensaje, "PRR:");
    while (*etiqueta != '\0' && i < 30U) mensaje[i++] = *etiqueta++;
    mensaje[i++] = '=';
    mensaje[i++] = '0';
    mensaje[i++] = 'x';
    mensaje[i++] = prr_digito_hex(valor >> 4);
    mensaje[i++] = prr_digito_hex(valor);
    strcpy(&mensaje[i], ",ERR=");
    i += 5U;
    mensaje[i++] = prr_digito_hex(prr_ultimo_error);
    mensaje[i++] = '\r';
    mensaje[i++] = '\n';
    mensaje[i] = '\0';
    (void)uart_enviar_texto(mensaje);
}

static void prr_informar(const char *etiqueta)
{
    prr_informar_valor(etiqueta, PRR);
}

/* =========================================================
   GPIO Y TEMPORIZADORES
   ========================================================= */

static void gpio_init(void)
{
    ACSR = _BV(ACD);                /* Comparador analogico no usado. */
    (void)prr_deshabilitar(PRR_SPI);              /* SPI no se usa en esta aplicacion. */
    DIDR0 |= _BV(ADC0D);            /* Evita entrada digital innecesaria en A0. */
    DDRB |= _BV(DDB5) | _BV(DDB3) | _BV(DDB4);
    PORTB &= (uint8_t)~(_BV(LED_PIN) | _BV(SOUNDER_PIN) | _BV(WDT_PRUEBA_PIN));

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
    prr_habilitar(_BV(PRTIM0));
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
    prr_habilitar(_BV(PRTIM1));
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
    prr_habilitar(_BV(PRTIM2));
    /* Modo normal, prescaler 8: 0,5 us por cuenta. */
    TCCR2A = 0U;
    TCCR2B = 0U;                  /* Se arrancara al liberar el pulso DHT. */
    TCNT2 = 0U;
    TIMSK2 = 0U;
    TIFR2 = (1U << TOV2);
    (void)prr_deshabilitar(PRR_TIMER2);
}

/* =========================================================
   ADC Y ACTIVIDAD DEL POTENCIOMETRO
   ========================================================= */

static void adc_habilitar(void)
{
    prr_habilitar(_BV(PRADC));
    ADMUX = (1U << REFS0);
    ADCSRA = (1U << ADEN) |
             (1U << ADPS2) | (1U << ADPS1) | (1U << ADPS0);
}

static void adc_deshabilitar(void)
{
    if ((PRR & PRR_ADC) == 0U && (ADCSRA & _BV(ADSC)) == 0U) {
        ADCSRA &= (uint8_t)~_BV(ADEN);
    }
}

static void adc_init(void)
{
    DDRC &= (uint8_t)~(1U << DDC0);
    PORTC &= (uint8_t)~(1U << PORTC0);
    adc_habilitar();
}

static uint16_t adc_leer_a0(void)
{
    if ((PRR & PRR_ADC) != 0U || (ADCSRA & _BV(ADEN)) == 0U) {
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
    prr_habilitar(_BV(PRTWI));
    DDRC &= (uint8_t)~((1U << DDC4) | (1U << DDC5));
    PORTC |= (1U << PORTC4) | (1U << PORTC5);

    TWSR = 0U;
    TWBR = (uint8_t)(((F_CPU / 100000UL) - 16UL) / 2UL);
    TWCR = (1U << TWEN);
}

static uint8_t twi_iniciar(uint8_t direccion_escritura)
{
    uint8_t estado_twi;

    twi_fase = TWI_TRANSFIRIENDO;
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

    twi_fase = TWI_STOP_PENDIENTE;
    TWCR = (1U << TWINT) | (1U << TWEN) | (1U << TWSTO);

    for (timeout = 0U; timeout < 3000U; timeout++) {
        if ((TWCR & (1U << TWSTO)) == 0U) {
            twi_fase = TWI_LIBRE;
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
    (void)prr_deshabilitar(PRR_TIMER2);
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
    prr_habilitar(PRR_TIMER2);     /* PRR=0x04 mientras se capturan pulsos. */
    TCCR2B = _BV(CS21);            /* 0,5 us/cuenta; configurar con reloj disponible. */
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
    (void)prr_deshabilitar(PRR_TIMER2); /* Termino captura: PRR vuelve a 0x44. */
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
    if ((PRR & PRR_TIMER0) == 0U) {
        TCCR0B = 0U;
        TIMSK0 = 0U;
        TIFR0 = _BV(OCF0A) | _BV(OCF0B) | _BV(TOV0);
    }
    if ((PRR & PRR_TIMER1) == 0U) {
        TCCR1B = _BV(WGM12);
        TIMSK1 = 0U;
        TIFR1 = _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);
    }
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
    (void)prr_deshabilitar(PRR_TIMER0 | PRR_TIMER2 | PRR_ADC | PRR_SPI);
    SREG = sreg;
}

static void restaurar_para_activo(void)
{
    reloj_rapido();
    /* SEGURO ya tenia Timer0 rapido, pero habia quitado el reloj de Timer1. */
    if ((PRR & PRR_TIMER1) != 0U) timer1_init();
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
    return (bandera_boton || bandera_reloj || bandera_wdt || uart_rx_error ||
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

    wdt_antes_powerdown();         /* Prueba 1 arma IRQ; prueba 2 pausa supervision. */
    prr_anterior = PRR;
    usart_anterior = UCSR0B;
    reloj_capturar_y_detener();
    if (!prr_fijar_perfil(PRR_DORMIR)) {
        /* Un recurso ocupado impide dormir: conservar main y su reloj. */
        reloj_standby();
        wdt_despues_powerdown();
        en_powerdown = 0U;
        int0_flanco_descendente();
        SREG = sreg;
        return;
    }
    uart_linea_longitud = 0U;
    uart_descartar_linea = 0U;
    PCMSK2 &= (uint8_t)~_BV(PCINT20);
    PCICR &= (uint8_t)~_BV(PCIE2);
    prr_ultimo_sueno = PRR;         /* Copia historica: UART ahora no tiene reloj. */

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();
    sleep_cpu();                   /* El pulsador debe mantenerse durante el arranque. */
    cli();
    sleep_disable();

    /* Se continua aqui tras el despertar; la SRAM conserva la FSM. */
    (void)prr_fijar_perfil(prr_anterior);
    uart_init();                   /* Reconfigurar USART tras devolverle el reloj. */
    UCSR0B = usart_anterior;
    reloj_standby();               /* Reloj disponible antes de validar el evento. */
    wdt_despues_powerdown();
    en_powerdown = 0U;
    int0_flanco_descendente();
    SREG = sreg;
    prr_informar_valor("ULTIMO_SUENO", prr_ultimo_sueno);
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
    wdt_al_volver_a_ejecucion();    /* Un usuario puede interrumpir la prueba dormida. */
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

    (void)prr_fijar_perfil(PRR_ACTIVO);
    (void)uart_enviar_texto("EV:ACTIVO\r\n");
    prr_informar("ACTIVO");
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
    (void)prr_fijar_perfil(PRR_STANDBY);
    (void)uart_enviar_texto("EV:STANDBY\r\n");
    prr_informar("STANDBY");
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
    (void)prr_fijar_perfil(PRR_STANDBY); /* T1 y UART hasta completar los avisos. */
    (void)uart_enviar_texto("EV:APAGADO\r\n");
    prr_informar("PREVIO_SUENO");
}

static void entrar_seguro(void)
{
    wdt_al_volver_a_ejecucion();
    restaurar_para_activo();       /* La alarma necesita Timer0 a 1 ms. */
    estado = ESTADO_SEGURO;
    adc_deshabilitar();
    cancelar_dht();
    lcd_encender_luz();
    lcd_comando(0x0CU);
    mostrar_seguro();
    iniciar_alarma();
    (void)prr_fijar_perfil(PRR_SEGURO); /* Timer0 mantiene alarma y pulsador. */
    (void)uart_enviar_texto("EV:SEGURO\r\n");
    prr_informar("SEGURO");
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
    if (strcmp(uart_linea, "BLOQUEO") == 0) {
        if (!wdt_solicitar_bloqueo()) {
            (void)uart_enviar_texto("ERROR:PRUEBA2_COMENTADA\r\n");
        }
    } else if (strcmp(uart_linea, "EMERGENCIA") == 0) {
        ultima_actividad = ahora;
        if (estado != ESTADO_SEGURO) {
            entrar_seguro();
        }
        (void)uart_enviar_texto("OK:EMERGENCIA\r\n");
    } else if (strcmp(uart_linea, "ACTIVO") == 0 ||
               strcmp(uart_linea, "ESTADO") == 0 ||
               strcmp(uart_linea, "PRR") == 0) {
        if (estado == ESTADO_SEGURO) {
            if (strcmp(uart_linea, "PRR") == 0) {
                prr_informar("SEGURO");
            } else if (strcmp(uart_linea, "ESTADO") == 0) {
                uart_informar_estado();
            } else {
                (void)uart_enviar_texto("BLOQUEADO:EMERGENCIA\r\n");
            }
        } else {
            if (estado != ESTADO_ACTIVO) {
                entrar_activo(ahora, 1U);
            }
            ultima_actividad = leer_tiempo();
            if (strcmp(uart_linea, "PRR") == 0) {
                prr_informar("ACTUAL");
            } else {
                uart_informar_estado();
            }
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
   ACTIVIDAD 2 - SELECCION MANUAL MEDIANTE COMENTARIOS
   Entrega: PRUEBA 1 activa y PRUEBA 2 comentada.
   Para cambiar: comentar COMPLETA la prueba activa y descomentar la otra.
   No hay selector automatico ni #define para deshabilitar la otra prueba.
   ========================================================= */

// PRUEBA_1_DESPERTAR_INICIO
// PRUEBA 1 ACTIVA: WDT SOLO INTERRUPCION, INTERVALO NOMINAL DE 8 S.

static void wdt_configurar_interrupcion_8s(void)
{
    uint8_t sreg = SREG;
    uint8_t permiso = _BV(WDCE) | _BV(WDE);
    uint8_t ajuste = _BV(WDIF) | _BV(WDIE) | _BV(WDP3) | _BV(WDP0);

    cli();
    wdt_reset();
    MCUSR &= (uint8_t)~_BV(WDRF);
    // Dos STS consecutivos garantizan la ventana protegida de cuatro ciclos.
    __asm__ __volatile__(
        "sts %[registro], %[permiso]\n\t"
        "sts %[registro], %[ajuste]\n\t"
        :
        : [registro] "n" (_SFR_MEM_ADDR(WDTCSR)),
          [permiso] "r" (permiso), [ajuste] "r" (ajuste)
        : "memory"
    );
    // WDIF se limpia escribiendo 1; WDE queda en 0, WDIE en 1.
    wdt_armado = 1U;
    SREG = sreg;
}

static uint8_t agregar_numero_u32(char *texto, uint8_t posicion, uint32_t numero)
{
    char inverso[10];
    uint8_t cantidad = 0U;
    do {
        inverso[cantidad++] = (char)('0' + (numero % 10UL));
        numero /= 10UL;
    } while (numero != 0UL);
    while (cantidad != 0U) {
        texto[posicion++] = inverso[--cantidad];
    }
    texto[posicion] = '\0';
    return posicion;
}

static void wdt_iniciar_prueba(void)
{
    wdt_disable();
    wdt_armado = 0U;
    bandera_wdt = 0U;
    (void)uart_enviar_texto("PRUEBA1:IRQ_8S\r\n");
    // Se arma al entrar realmente en Power-Down, no durante ACTIVO.
}

static void wdt_antes_powerdown(void)
{
    if (!wdt_armado) {
        wdt_configurar_interrupcion_8s();
    }
    // No reiniciar el contador en cada vuelta: el intervalo sigue corriendo.
}

static void wdt_despues_powerdown(void)
{
    // WDE=0: continua como interrupcion periodica, sin reiniciar el sistema.
}

static void wdt_al_volver_a_ejecucion(void)
{
    wdt_disable();
    wdt_armado = 0U;
    bandera_wdt = 0U;
    PORTB &= (uint8_t)~_BV(WDT_PRUEBA_PIN);
}

static void wdt_procesar_evento(void)
{
    uint8_t pendiente;
    uint8_t sreg = SREG;
    uint32_t numero_evento;
    uint8_t posicion;
    char mensaje[64];

    cli();
    pendiente = bandera_wdt;
    bandera_wdt = 0U;
    numero_evento = eventos_wdt;
    SREG = sreg;
    if (!pendiente) {
        return;
    }
    if (estado != ESTADO_APAGADO) {
        PORTB &= (uint8_t)~_BV(WDT_PRUEBA_PIN);
        return;                    // La pulsacion/emergencia tiene prioridad.
    }

    // La CPU YA desperto: habilitar PRADC/ADEN antes de adquirir A0.
    adc_habilitar();
    valor_adc_actual = adc_leer_a0();
    adc_deshabilitar();
    (void)prr_deshabilitar(PRR_ADC);
    PORTB &= (uint8_t)~_BV(WDT_PRUEBA_PIN); // Fin del trabajo de lectura.

    strcpy(mensaje, "WDT,evento=");
    posicion = agregar_numero_u32(mensaje, (uint8_t)strlen(mensaje), numero_evento);
    strcpy(&mensaje[posicion], ",ADC=");
    posicion = agregar_numero_u32(mensaje, (uint8_t)strlen(mensaje), valor_adc_actual);
    strcpy(&mensaje[posicion], ",nominal_ms=8000\r\n");
    (void)uart_enviar_texto(mensaje);
    // No cambiar estado, ultima_actividad ni inventar tiempo transcurrido.
    // main termina TX y vuelve a Power-Down mediante gestionar_sueno().
}

static void wdt_supervisar_ciclo(void)
{
    // La prueba 1 NO supervisa bloqueos y no reinicia el contador en main.
}

static uint8_t wdt_solicitar_bloqueo(void)
{
    return 0U;                     // La prueba de reset sigue comentada.
}
// PRUEBA_1_DESPERTAR_FIN

/* PRUEBA_2_RESET_COMENTADA_INICIO

// PRUEBA 2: WDT COMO RECUPERACION, RESET NOMINAL DE 2 SEGUNDOS.
// Para usar esta prueba: COMENTAR primero la PRUEBA 1 y descomentar esta.
// La peticion BLOQUEO provoca el bucle infinito SOLO cuando se recibe.

static void wdt_iniciar_prueba(void)
{
    wdt_armado = 0U;
    bandera_wdt = 0U;
    wdt_reset();
    wdt_enable(WDTO_2S);            // WDE=1, WDIE=0: reset, sin interrupcion.
    (void)uart_enviar_texto("PRUEBA2:RESET_2S\r\n");
}

static void wdt_antes_powerdown(void)
{
    // Un sueno intencional no debe confundirse con el bloqueo del programa.
    // La prueba de recuperacion supervisa ACTIVO, IDLE y SEGURO.
    wdt_disable();
}

static void wdt_despues_powerdown(void)
{
    wdt_reset();
    wdt_enable(WDTO_2S);            // Reanuda supervision al volver del sueno.
}

static void wdt_al_volver_a_ejecucion(void)
{
    // Se conserva la supervision: no se atiende el WDT desde ninguna ISR.
}

static void wdt_procesar_evento(void)
{
    // En modo reset no se espera ISR(WDT_vect).
    bandera_wdt = 0U;
}

static void wdt_supervisar_ciclo(void)
{
    // Solo main lo atiende, DESPUES de completar la iteracion normal.
    wdt_reset();
}

static uint8_t wdt_solicitar_bloqueo(void)
{
    (void)uart_enviar_texto("PRUEBA2:BLOQUEO_SIN_WDR\r\n");
    marca_bloqueo = MARCA_BLOQUEO_WDT;
    marca_bloqueo_inversa = ~MARCA_BLOQUEO_WDT;
    PORTB |= _BV(WDT_PRUEBA_PIN);   // Marca el comienzo del bloqueo de ensayo.
    wdt_reset();                   // Comienza un plazo completo de 2 s.
    for (;;) {
        // BLOQUEO INTENCIONAL: las ISR pueden seguir, pero main no supervisa.
        // No colocar wdt_reset() aqui ni en las interrupciones.
    }
    return 1U;                     // Inalcanzable; satisface tambien compiladores antiguos.
}
PRUEBA_2_RESET_COMENTADA_FIN */

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
        linea_texto(mensaje1, 0U, "LAB 5 - B PRR");
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
    uart_init();
    (void)uart_enviar_texto("LAB5:B3\r\n");
    if ((causa_reinicio & _BV(WDRF)) != 0U ||
        (((causa_reinicio & _BV(PORF)) == 0U) &&
         marca_bloqueo == MARCA_BLOQUEO_WDT &&
         marca_bloqueo_inversa == ~MARCA_BLOQUEO_WDT)) {
        entrar_seguro();           /* Tras WDT, no habilitar funcionamiento normal. */
        if ((causa_reinicio & _BV(WDRF)) != 0U) {
            (void)uart_enviar_texto("RESET:WDT\r\n");
        } else {
            /* No atribuir causa si el cargador de arranque borro MCUSR. */
            (void)uart_enviar_texto("RESET:TRAS_BLOQUEO_SIN_WDRF\r\n");
        }
        (void)uart_enviar_texto("RECUPERACION:SEGURO\r\n");
    } else {
        mostrar_activo();
        (void)prr_fijar_perfil(PRR_ACTIVO);
        (void)uart_enviar_texto("EV:ACTIVO\r\n");
        prr_informar("ACTIVO");
    }
    marca_bloqueo = 0UL;           /* Evita repetir la recuperacion en otro reset. */
    marca_bloqueo_inversa = 0UL;

    sei();
    wdt_iniciar_prueba();

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
        wdt_procesar_evento();     /* Lectura/registro fuera de la ISR. */
        wdt_supervisar_ciclo();    /* Solo la prueba 2 habilitada ejecuta WDR aqui. */
        gestionar_sueno();         /* Solo duerme en STANDBY o APAGADO. */
    }
}
