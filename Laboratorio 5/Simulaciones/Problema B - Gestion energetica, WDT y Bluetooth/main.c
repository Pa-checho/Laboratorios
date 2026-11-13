/*
 * Laboratorio 5 - Problema B
 * ATmega328P a 16 MHz
 *
 * En Proteus se usa la terminal virtual para probar la UART del HC-05.
 * El DHT11 de la simulacion esta conectado a PD3.
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
#include <avr/pgmspace.h>

#if F_CPU != 16000000UL
#error "Esta version requiere revisar los temporizadores si F_CPU no es 16 MHz"
#endif

#define LED_PIN                         PB5
#define SOUNDER_PIN                     PB3
#ifndef DHT_PIN
#define DHT_PIN                         PD4
#endif

#if DHT_PIN == PD3
#define DHT_DDR_PIN                     DDD3
#define DHT_PCINT_BIT                   PCINT19
#elif DHT_PIN == PD4
#define DHT_DDR_PIN                     DDD4
#define DHT_PCINT_BIT                   PCINT20
#else
#error "DHT_PIN debe ser PD3 o PD4"
#endif
#define BOTON_PIN                       PD2
#define WDT_PRUEBA_PIN                  PB4 /* D12: marca observable del WDT. */

#define WDT_PRUEBA_DESPERTADOR          1U
#define WDT_PRUEBA_RESET                2U

#ifndef WDT_PRUEBA_ACTIVA
#define WDT_PRUEBA_ACTIVA               WDT_PRUEBA_DESPERTADOR
#endif

#if WDT_PRUEBA_ACTIVA != WDT_PRUEBA_DESPERTADOR && \
    WDT_PRUEBA_ACTIVA != WDT_PRUEBA_RESET
#error "WDT_PRUEBA_ACTIVA debe ser WDT_PRUEBA_DESPERTADOR o WDT_PRUEBA_RESET"
#endif

#define LCD_RS                          0x01U
#define LCD_E                           0x04U
#define LCD_LUZ                         0x08U
#define LCD_DIRECCION_NATIVA            0x3EU
#define LCD_CONTROL_COMANDO             0x00U
#define LCD_CONTROL_DATO                0x40U
#define LCD_TIPO_NINGUNO                0U
#define LCD_TIPO_PCF8574                1U
#define LCD_TIPO_NATIVO                 2U

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


#define RELOJ_RAPIDO                    0U
#define RELOJ_STANDBY                   1U
#define RELOJ_DETENIDO                  2U
#define PASO_STANDBY_MS                 1000UL

#define UART_RX_TAM                     64U
#define UART_TX_TAM                     256U
#define UART_LINEA_TAM                  64U
#define UART_TIMEOUT_LINEA_MS           3000UL
#define UART_BAUD                       9600UL
#define UART_UBRR_9600                  (F_CPU / (16UL * UART_BAUD) - 1UL)
#define PROTO_RX_PRESUPUESTO            16U

#if UART_TX_TAM != 256U
#error "TX requiere 256 bytes para su envoltura uint8_t"
#endif

#if UART_RX_TAM < 2U || UART_RX_TAM > 256U || (UART_RX_TAM & (UART_RX_TAM - 1U))
#error "RX debe ser potencia de dos entre 2 y 256"
#endif

#define PROTO_TRAMA_TAM                 96U
#define PROTO_RESERVA_TX                192U /* Una orden pendiente, sin bloquear. */
#define UART_ERR_FRAME                  0x01U
#define UART_ERR_OVERRUN                0x02U
#define UART_ERR_PARIDAD                0x04U
#define UART_ERR_BUFFER                 0x08U
#define LED_AUTOMATICO                  0U
#define LED_FORZADO_OFF                 1U
#define LED_FORZADO_ON                  2U
#define ALARMA_LOCAL                    1U
#define ALARMA_REMOTA                   2U
#define ALARMA_WDT                      3U
#define ALARMA_TRAS_BLOQUEO              4U
#define ALARMAS_TAM                     4U
#define ES(texto, literal) (texto_comparar_P((texto), PSTR(literal)) == 0)
#define COPIAR(destino, literal) texto_copiar_P((destino), PSTR(literal))
#define ERROR_PROTO(literal) protocolo_error(PSTR(literal))

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

#define DHT_INICIO_BAJO_MS              20UL

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
static volatile uint16_t contador_led_ms = 0U, contador_lcd_ms = 0U, contador_dht_ms = 0U;
static volatile uint16_t tiempo_sonido_ms = 0U;

static volatile uint8_t bandera_led = 0U, bandera_adc = 0U, bandera_lcd = 0U;
static volatile uint8_t bandera_dht = 0U, bandera_boton = 0U, alarma_continua = 0U;

static estado_sistema_t estado = ESTADO_ACTIVO;

static uint32_t ultima_actividad = 0UL, inicio_boton = 0UL, ultimo_boton = 0UL;

static uint8_t boton_presionado = 0U, pulsacion_larga_lista = 0U, lcd_disponible = 0U;
static uint8_t lectura_dht_correcta = 0U, error_dht = 0U, temperatura = 0U;
static uint8_t humedad = 0U;
static uint16_t valor_adc_actual = 0U, referencia_adc_actividad = 0U;
static uint8_t referencia_adc_valida = 0U;

static uint8_t direccion_lcd = 0x27U, luz_lcd = LCD_LUZ;
static uint8_t tipo_lcd = LCD_TIPO_NINGUNO;

/* Variables de la lectura temporizada del DHT11. */
static volatile uint8_t estado_dht = DHT_REPOSO;
static volatile uint8_t dht_datos[5] = {0U, 0U, 0U, 0U, 0U};
static uint32_t dht_marca_ms = 0UL;

/* Estado temporal y de comunicacion agregado para B1. */
static volatile uint8_t modo_reloj = RELOJ_RAPIDO, bandera_reloj = 0U;
static uint16_t fraccion_reloj_us = 0U;
static volatile uint8_t en_powerdown = 0U;

static volatile uint8_t uart_rx[UART_RX_TAM];
static volatile uint8_t uart_rx_cabeza = 0U, uart_rx_cola = 0U, uart_rx_error = 0U;
static volatile uint8_t uart_tx[UART_TX_TAM];
static volatile uint8_t uart_tx_cabeza = 0U, uart_tx_cola = 0U, uart_tx_iniciada = 0U;
static char uart_linea[UART_LINEA_TAM];
static uint8_t uart_linea_longitud = 0U, uart_descartar_linea = 0U;
static uint32_t uart_inicio_linea = 0UL;

/* Actividad 4: parser y diagnostico. No se usa memoria dinamica. */
static uint8_t uart_en_trama = 0U, uart_trama_pendiente = 0U;
static uint8_t modo_led = LED_AUTOMATICO, solicitud_sensores = 0U, sensores_listos = 0U;
static uint8_t dht_intento_previo = 0U;
static uint32_t dht_ultimo_inicio_ms = 0UL, dht_ultima_lectura_ms = 0UL;
static uint16_t protocolo_aceptadas = 0U, protocolo_errores = 0U, uart_tx_descartadas = 0U;
typedef struct {
    uint32_t numero, tiempo;
    uint8_t causa;
} alarma_evento_t;
static alarma_evento_t alarmas[ALARMAS_TAM];
static uint32_t alarmas_total = 0UL;
static uint8_t alarmas_cabeza = 0U, alarmas_guardadas = 0U;

/* Diagnostico PRR en SRAM; no es una medicion de consumo. */
static uint8_t twi_fase = TWI_LIBRE;
static volatile uint8_t prr_ultimo_error = PRR_OK;
static volatile uint16_t prr_rechazos = 0U, prr_cambios = 0U;
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

void wdt_capturar_reset_temprano(void) {
    causa_reinicio = MCUSR;
    MCUSR = 0U;                    /* Limpiar WDRF ANTES de deshabilitar WDT. */
    wdt_disable();                 /* La macro respeta la secuencia protegida. */
    /* Sin return: .init3 continua con la inicializacion normal de C. */
}

/* TEXTOS FLASH: implementaciones locales, sin funciones _P externas. */
static char *texto_copiar_P(char *destino, const char *origen_P) {
    char *inicio = destino;
    uint8_t c;
    do {
        c = pgm_read_byte(origen_P++);
        *destino++ = (char)c;
    } while (c != 0U);
    return inicio;
}

static char *texto_anexar_P(char *destino, const char *origen_P) {
    char *fin = destino;
    while (*fin != '\0') fin++;
    (void)texto_copiar_P(fin, origen_P);
    return destino;
}

static int texto_comparar_P(const char *ram, const char *flash) {
    uint8_t a, b;
    do {
        a = (uint8_t)*ram++;
        b = pgm_read_byte(flash++);
        if (a != b) return (int)a - (int)b;
    } while (a != 0U);
    return 0;
}

/* INTERRUPCIONES */

ISR(TIMER0_COMPA_vect) {
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

ISR(TIMER1_COMPA_vect) {
    if (modo_reloj == RELOJ_STANDBY) {
        tiempo_ms += PASO_STANDBY_MS; /* Mantiene la inactividad en IDLE. */
        bandera_reloj = 1U;
    } else if (modo_reloj == RELOJ_RAPIDO) {
        bandera_adc = 1U;             /* Funcion original de A en ACTIVO. */
    }
}

ISR(WDT_vect) {
    bandera_wdt = 1U;              /* Solo notifica; ADC y UART se atienden en main. */
    eventos_wdt++;
    PORTB ^= _BV(WDT_PRUEBA_PIN);
}

ISR(INT0_vect) {
    bandera_boton = 1U;
    if (en_powerdown) {
        /* El nivel bajo podria repetir la IRQ mientras se mantiene pulsado. */
        EIMSK &= (uint8_t)~_BV(INT0);
    }
}

ISR(USART_RX_vect) {
    uint8_t estado_uart = UCSR0A;     /* Leer errores ANTES de UDR0. */
    uint8_t dato = UDR0;
    uint8_t siguiente = (uint8_t)((uart_rx_cabeza + 1U) & (UART_RX_TAM - 1U));
    uint8_t error = 0U;
    if ((estado_uart & _BV(FE0)) != 0U) error |= UART_ERR_FRAME;
    if ((estado_uart & _BV(DOR0)) != 0U) error |= UART_ERR_OVERRUN;
    if ((estado_uart & _BV(UPE0)) != 0U) error |= UART_ERR_PARIDAD;
    if (siguiente == uart_rx_cola) error |= UART_ERR_BUFFER;
    if (error != 0U) {
        uart_rx_error |= error;     /* No valida ni ejecuta comandos en la ISR. */
    } else {
        uart_rx[uart_rx_cabeza] = dato;
        uart_rx_cabeza = siguiente;
    }
}

ISR(USART_UDRE_vect) {
    if (uart_tx_cola != uart_tx_cabeza) {
        UCSR0A = _BV(TXC0);          /* Limpia TXC: aun falta transmitir. */
        UDR0 = uart_tx[uart_tx_cola];
        uart_tx_cola = (uint8_t)((uart_tx_cola + 1U) & (UART_TX_TAM - 1U));
        uart_tx_iniciada = 1U;
    } else {
        UCSR0B &= (uint8_t)~_BV(UDRIE0);
    }
}

/* TIEMPO Y SALIDAS */

static uint32_t leer_tiempo(void) {
    uint8_t sreg = SREG;
    uint32_t copia;
    cli();
    copia = tiempo_ms;
    SREG = sreg;
    return copia;
}

static void configurar_sonido(uint16_t duracion_ms, uint8_t continua) {
    uint8_t sreg = SREG;
    cli();
    tiempo_sonido_ms = duracion_ms;
    alarma_continua = continua;
    if (!continua) PORTB &= (uint8_t)~_BV(SOUNDER_PIN);
    SREG = sreg;
}
static void iniciar_sonido(uint16_t duracion_ms) { configurar_sonido(duracion_ms, 0U); }

static void iniciar_alarma(void) { configurar_sonido(0U, 1U); }

static void apagar_sonido(void) { configurar_sonido(0U, 0U); }

/* ACTIVIDAD 4 - UART 9600 8N1 Y TRANSMISION DE TRAMAS
 * Las ISR solo trasladan bytes. Emisor y parser se ejecutan en main. */

static void uart_init(void) {
    prr_habilitar(_BV(PRUSART0));
    PORTD |= _BV(PD0);              /* RX definido cuando no hay modulo. */
    UCSR0A = 0U;                    /* Velocidad normal, U2X0=0. */
    UBRR0H = (uint8_t)(UART_UBRR_9600 >> 8);
    UBRR0L = (uint8_t)UART_UBRR_9600; /* 103 a 16 MHz; formato 8N1. */
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8 bits, sin paridad, 1 stop. */
    UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);
}

static char protocolo_hex(uint8_t valor) {
    valor &= 15U;
    return valor < 10U ? (char)('0' + valor) : (char)('A' + valor - 10U);
}

static uint8_t uart_tx_libre(void) {
    return (uint8_t)(uart_tx_cola - uart_tx_cabeza - 1U);
}

/* Solo main produce TX; la ISR consume exclusivamente los bytes publicados. */
typedef struct {
    uint8_t cabeza, usados, libres;
    uint8_t checksum;
} uart_constructor_t;

/* Se integra en el emisor para evitar una llamada de funcion por cada byte. */
static inline __attribute__((always_inline))
uint8_t uart_anexar_cuerpo(uart_constructor_t *tx, uint8_t dato) {
    /* Reservar siempre *HH, CR y LF. No publicar una trama incompleta. */
    if (tx->usados >= PROTO_TRAMA_TAM - 6U ||
        (uint16_t)tx->usados + 6U > tx->libres) return 0U;
    uart_tx[tx->cabeza++] = dato; /* TX=256: el indice uint8_t vuelve a cero. */
    tx->usados++;
    tx->checksum ^= dato;
    return 1U;
}

/* Comando en Flash; dato en RAM o Flash segun dato_flash. Sin trama temporal. */
static uint8_t uart_enviar_trama_fuente(const char *comando_flash,
                                      const char *dato, uint8_t dato_flash) {
    uart_constructor_t tx;
    uint8_t c, sreg;
    tx.cabeza = uart_tx_cabeza;
    tx.usados = 1U;
    tx.libres = uart_tx_libre();
    tx.checksum = 0U;
    if (tx.libres < 6U) goto rechazar;
    uart_tx[tx.cabeza++] = '@';
    while ((c = pgm_read_byte(comando_flash++)) != 0U) {
        if (!uart_anexar_cuerpo(&tx, c)) goto rechazar;
    }
    if (!uart_anexar_cuerpo(&tx, ',')) goto rechazar;
    for (;;) {
        c = dato_flash ? pgm_read_byte(dato) : (uint8_t)*dato;
        dato++;
        if (c == 0U || c == '\r' || c == '\n') break;
        if (c == '@' || c == '*' || !uart_anexar_cuerpo(&tx, c)) goto rechazar;
    }
    uart_tx[tx.cabeza++] = '*';
    uart_tx[tx.cabeza++] = protocolo_hex(tx.checksum >> 4);
    uart_tx[tx.cabeza++] = protocolo_hex(tx.checksum);
    uart_tx[tx.cabeza++] = '\r';
    uart_tx[tx.cabeza++] = '\n';
    sreg = SREG;
    cli();
    uart_tx_cabeza = tx.cabeza; /* Publicar solo despues del checksum y el fin. */
    UCSR0B |= _BV(UDRIE0);
    SREG = sreg;
    return 1U;
rechazar:
    /* Los bytes escritos en espacio libre siguen invisibles para la ISR. */
    uart_tx_descartadas++;
    return 0U;
}

static uint8_t uart_enviar_trama(const char *comando_flash, const char *dato_ram) {
    return uart_enviar_trama_fuente(comando_flash, dato_ram, 0U);
}

static uint8_t uart_enviar_trama_P(const char *comando_flash, const char *dato_flash) {
    return uart_enviar_trama_fuente(comando_flash, dato_flash, 1U);
}

static uint8_t uart_enviar_texto(const char *texto_ram) {
    return uart_enviar_trama(PSTR("EVENTO"), texto_ram);
}

static uint8_t uart_enviar_texto_P(const char *texto_flash) {
    return uart_enviar_trama_P(PSTR("EVENTO"), texto_flash);
}

static void protocolo_error(const char *motivo_flash) {
    protocolo_errores++;
    (void)uart_enviar_trama_P(PSTR("ERROR"), motivo_flash);
}

static void texto_agregar_u32(char *texto, uint32_t valor) {
    char digitos[10];
    uint8_t n = 0U;
    uint8_t i = (uint8_t)strlen(texto);
    do {
        digitos[n++] = (char)('0' + valor % 10UL);
        valor /= 10UL;
    } while (valor != 0UL);
    while (n != 0U) texto[i++] = digitos[--n];
    texto[i] = '\0';
}

static const char *nombre_estado_P(void) {
    if (estado == ESTADO_ACTIVO) return PSTR("ACTIVO");
    if (estado == ESTADO_STANDBY) return PSTR("STANDBY");
    if (estado == ESTADO_APAGADO) return PSTR("APAGADO");
    return PSTR("SEGURO");
}

static void registrar_alarma(uint8_t causa) {
    alarma_evento_t *registro = &alarmas[alarmas_cabeza];
    alarmas_total++;
    registro->numero = alarmas_total;
    registro->tiempo = leer_tiempo(); /* Reloj de firmware: excluye Power-Down. */
    registro->causa = causa;
    alarmas_cabeza = (uint8_t)((alarmas_cabeza + 1U) % ALARMAS_TAM);
    if (alarmas_guardadas < ALARMAS_TAM) alarmas_guardadas++;
}

static const char *nombre_alarma_P(uint8_t causa) {
    if (causa == ALARMA_LOCAL) return PSTR("LOCAL");
    if (causa == ALARMA_REMOTA) return PSTR("REMOTA");
    if (causa == ALARMA_WDT) return PSTR("WDT");
    return PSTR("TRAS_BLOQUEO");
}

static uint8_t uart_tomar_byte(uint8_t *dato) {
    if (uart_rx_cola == uart_rx_cabeza) return 0U;
    *dato = uart_rx[uart_rx_cola];
    uart_rx_cola = (uint8_t)((uart_rx_cola + 1U) & (UART_RX_TAM - 1U));
    return 1U;
}

static uint8_t uart_tx_terminada(void) {
    return (uart_tx_cola == uart_tx_cabeza &&
            (!uart_tx_iniciada || (UCSR0A & _BV(TXC0)) != 0U)) ? 1U : 0U;
}

static void uart_informar_estado(void) {
    char respuesta[64];
    COPIAR(respuesta, "ESTADO;EST=");
    texto_anexar_P(respuesta, nombre_estado_P());
    texto_anexar_P(respuesta, PSTR(";PRR="));
    {
        uint8_t n = (uint8_t)strlen(respuesta);
        respuesta[n++] = protocolo_hex(PRR >> 4);
        respuesta[n++] = protocolo_hex(PRR);
        respuesta[n] = '\0';
    }
    texto_anexar_P(respuesta, PSTR(";EMERG="));
    texto_agregar_u32(respuesta, estado == ESTADO_SEGURO ? 1U : 0U);
    (void)uart_enviar_trama(PSTR("OK"), respuesta);
}

/* ACTIVIDAD 3 - PRR: VALIDAR, DETENER Y DESPUES QUITAR EL RELOJ
 * Todas las escrituras a PRR pasan por estas funciones.
 * Las ISR no cambian PRR. Las comprobaciones y el cambio son atomicos. */

static void prr_registrar_error(uint8_t codigo) {
    prr_ultimo_error = codigo;
    prr_rechazos++;
}

/* Solo cambia el registro; el llamador ya verifico y preparo los modulos. */
static uint8_t prr_escribir(uint8_t valor) {
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
static void prr_habilitar(uint8_t mascara) {
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
static uint8_t prr_validar(uint8_t destino) {
    uint8_t nuevos = destino & (uint8_t)~PRR;
    if ((destino & (uint8_t)~PRR_VALIDOS) != 0U) return PRR_ERR_BIT;
    if (((destino & PRR_TIMER0) != 0U && modo_reloj == RELOJ_RAPIDO) ||
        ((destino & PRR_TIMER1) != 0U &&
         (modo_reloj == RELOJ_STANDBY ||
          (estado == ESTADO_ACTIVO && modo_reloj != RELOJ_DETENIDO)))) return PRR_ERR_RELOJ;       /* Scheduler/ADC de ACTIVO o reloj de IDLE. */
    if ((destino & PRR_TIMER2) != 0U &&
        estado_dht >= DHT_ESPERA_RESPUESTA_BAJA && estado_dht <= DHT_BIT_ALTO) return PRR_ERR_DHT;
    if ((nuevos & PRR_ADC) != 0U && (ADCSRA & _BV(ADSC)) != 0U) return PRR_ERR_ADC;
    if ((nuevos & PRR_USART) != 0U &&
        (estado != ESTADO_APAGADO || !uart_tx_terminada() ||
         uart_rx_cabeza != uart_rx_cola || uart_rx_error ||
         uart_linea_longitud != 0U || uart_en_trama || uart_trama_pendiente ||
         (UCSR0A & _BV(RXC0)) != 0U)) return PRR_ERR_UART;
    if ((nuevos & PRR_TWI) != 0U) {
        if (twi_fase == TWI_STOP_PENDIENTE && (TWCR & _BV(TWSTO)) == 0U) {
            twi_fase = TWI_LIBRE;  /* STOP termino despues de su espera limitada. */
        }
        if (twi_fase != TWI_LIBRE || (TWCR & _BV(TWSTO)) != 0U) return PRR_ERR_TWI;
    }
    /* SPI no se usa: si otro desarrollo lo habilita, no apagarlo a ciegas. */
    if ((nuevos & PRR_SPI) != 0U && (SPCR & _BV(SPE)) != 0U) return PRR_ERR_SPI;
    return PRR_OK;
}

static uint8_t prr_fijar_perfil(uint8_t destino) {
    uint8_t sreg = SREG, nuevos, error;
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

static uint8_t prr_deshabilitar(uint8_t mascara) {
    /* Validar primero la mascara original: no ocultar un bit reservado. */
    uint8_t sreg = SREG, resultado;
    cli();
    resultado = prr_fijar_perfil((uint8_t)(PRR | mascara));
    SREG = sreg;
    return resultado;
}

static void prr_informar_valor(const char *etiqueta, uint8_t valor) {
    char mensaje[48];
    uint8_t i = 4U;
    texto_copiar_P(mensaje, PSTR("PRR:"));
    uint8_t c;
    while (i < 30U && (c = pgm_read_byte(etiqueta++)) != 0U) {
        mensaje[i++] = (char)c;
    }
    mensaje[i++] = '=';
    mensaje[i++] = '0';
    mensaje[i++] = 'x';
    mensaje[i++] = protocolo_hex(valor >> 4);
    mensaje[i++] = protocolo_hex(valor);
    texto_copiar_P(&mensaje[i], PSTR(",ERR="));
    i += 5U;
    mensaje[i++] = protocolo_hex(prr_ultimo_error);
    mensaje[i++] = '\r';
    mensaje[i++] = '\n';
    mensaje[i] = '\0';
    (void)uart_enviar_texto(mensaje);
}

static void prr_informar(const char *etiqueta) {
    prr_informar_valor(etiqueta, PRR);
}

/* GPIO Y TEMPORIZADORES */

static void gpio_init(void) {
    ACSR = _BV(ACD);                /* Comparador analogico no usado. */
    (void)prr_deshabilitar(PRR_SPI);              /* SPI no se usa en esta aplicacion. */
    DIDR0 |= _BV(ADC0D);            /* Evita entrada digital innecesaria en A0. */
    DDRB |= _BV(DDB5) | _BV(DDB3) | _BV(DDB4);
    PORTB &= (uint8_t)~(_BV(LED_PIN) | _BV(SOUNDER_PIN) | _BV(WDT_PRUEBA_PIN));
    DDRD &= (uint8_t)~(1U << DHT_DDR_PIN);
    PORTD |= (1U << DHT_PIN);
    DDRD &= (uint8_t)~(1U << DDD2);
    PORTD |= (1U << BOTON_PIN);
    EICRA = (uint8_t)((EICRA &
             (uint8_t)~((1U << ISC01) | (1U << ISC00))) |
             (1U << ISC01));
    EIFR = (1U << INTF0);
    EIMSK |= (1U << INT0);
}

static void timer0_init(void) {
    prr_habilitar(_BV(PRTIM0));
    TIFR0 = _BV(OCF0A) | _BV(OCF0B) | _BV(TOV0);
    /* 16 MHz / 64 = 250 kHz; 250 cuentas = 1 ms. */
    TCCR0A = (1U << WGM01);
    TCCR0B = (1U << CS01) | (1U << CS00);
    OCR0A = 249U;
    TCNT0 = 0U;
    TIMSK0 = (1U << OCIE0A);
}

static void timer1_init(void) {
    prr_habilitar(_BV(PRTIM1));
    TIFR1 = _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);
    TCCR1A = 0U;
    if (modo_reloj == RELOJ_STANDBY) {
        /* 16 MHz / 256 = 62,5 kHz; 62500 cuentas = 1 segundo. */
        TCCR1B = _BV(WGM12) | _BV(CS12);
        OCR1A = 62499U;
    } else {
        /* 16 MHz / 64 = 250 kHz; 25000 cuentas = 100 ms. */
        TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10);
        OCR1A = 24999U;
    }
    TCNT1 = 0U;
    TIMSK1 = (1U << OCIE1A);
}

static void timer2_init(void) {
    prr_habilitar(_BV(PRTIM2));
    /* Modo normal, prescaler 8: 0,5 us por cuenta. */
    TCCR2A = 0U;
    TCCR2B = 0U;                  /* Se arrancara al liberar el pulso DHT. */
    TCNT2 = 0U;
    TIMSK2 = 0U;
    TIFR2 = (1U << TOV2);
    (void)prr_deshabilitar(PRR_TIMER2);
}

/* ADC Y ACTIVIDAD DEL POTENCIOMETRO */

static void adc_habilitar(void) {
    prr_habilitar(_BV(PRADC));
    ADMUX = (1U << REFS0);
    ADCSRA = (1U << ADEN) |
             (1U << ADPS2) | (1U << ADPS1) | (1U << ADPS0);
}

static void adc_deshabilitar(void) {
    if ((PRR & PRR_ADC) == 0U && (ADCSRA & _BV(ADSC)) == 0U) {
        ADCSRA &= (uint8_t)~_BV(ADEN);
    }
}

static void adc_init(void) {
    DDRC &= (uint8_t)~(1U << DDC0);
    PORTC &= (uint8_t)~(1U << PORTC0);
    adc_habilitar();
}

static uint16_t adc_leer_a0(void) {
    if ((PRR & PRR_ADC) != 0U || (ADCSRA & _BV(ADEN)) == 0U) {
        adc_habilitar();
    }
    ADMUX = (1U << REFS0);
    ADCSRA |= (1U << ADSC);
    while ((ADCSRA & (1U << ADSC)) != 0U) {
        /* A 125 kHz: 104 us normales; primera tras ADEN, 200 us. */
    }
    return ADC;
}

static void revisar_actividad_adc(uint16_t nueva_lectura, uint32_t ahora) {
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

/* TWI / LCD 16x2 CON PCF8574 */

static uint8_t twi_esperar(void) {
    uint16_t timeout;
    for (timeout = 0U; timeout < 3000U; timeout++) {
        if ((TWCR & (1U << TWINT)) != 0U) return 1U;
        _delay_us(1);
    }
    return 0U;
}

static void twi_init(void) {
    prr_habilitar(_BV(PRTWI));
    DDRC &= (uint8_t)~((1U << DDC4) | (1U << DDC5));
    PORTC |= (1U << PORTC4) | (1U << PORTC5);
    TWSR = 0U;
    TWBR = (uint8_t)(((F_CPU / 100000UL) - 16UL) / 2UL);
    TWCR = (1U << TWEN);
}

static uint8_t twi_iniciar(uint8_t direccion_escritura) {
    uint8_t estado_twi;
    twi_fase = TWI_TRANSFIRIENDO;
    TWCR = (1U << TWINT) | (1U << TWSTA) | (1U << TWEN);
    if (!twi_esperar()) return 0U;
    estado_twi = TWSR & 0xF8U;
    if (estado_twi != TW_START && estado_twi != TW_REP_START) return 0U;
    TWDR = direccion_escritura;
    TWCR = (1U << TWINT) | (1U << TWEN);
    if (!twi_esperar()) return 0U;
    return ((TWSR & 0xF8U) == TW_MT_SLA_ACK) ? 1U : 0U;
}

static uint8_t twi_enviar(uint8_t dato) {
    TWDR = dato;
    TWCR = (1U << TWINT) | (1U << TWEN);
    if (!twi_esperar()) return 0U;
    return ((TWSR & 0xF8U) == TW_MT_DATA_ACK) ? 1U : 0U;
}

static void twi_stop(void) {
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

static uint8_t lcd_expansor(uint8_t valor) {
    uint8_t correcto;
    correcto = twi_iniciar((uint8_t)(direccion_lcd << 1));
    if (correcto) {
        correcto = twi_enviar((uint8_t)(valor | luz_lcd));
    }
    twi_stop();
    return correcto;
}

static uint8_t lcd_nativo_enviar(uint8_t control, uint8_t valor) {
    uint8_t correcto;
    correcto = twi_iniciar((uint8_t)(LCD_DIRECCION_NATIVA << 1));
    if (correcto) correcto = twi_enviar(control);
    if (correcto) correcto = twi_enviar(valor);
    twi_stop();
    return correcto;
}

static void lcd_encender_luz(void) {
    if (!lcd_disponible) return;
    if (tipo_lcd != LCD_TIPO_PCF8574) return;
    luz_lcd = LCD_LUZ;
    (void)lcd_expansor(0U);
}

static void lcd_apagar_luz(void) {
    if (!lcd_disponible) return;
    if (tipo_lcd != LCD_TIPO_PCF8574) return;
    luz_lcd = 0U;
    (void)lcd_expansor(0U);
}

static uint8_t lcd_pulso(uint8_t valor) {
    if (!lcd_expansor((uint8_t)(valor | LCD_E))) return 0U;
    _delay_us(1);
    if (!lcd_expansor((uint8_t)(valor & (uint8_t)~LCD_E))) return 0U;
    _delay_us(50);
    return 1U;
}

static uint8_t lcd_medio_byte(uint8_t valor) {
    if (!lcd_expansor(valor)) return 0U;
    return lcd_pulso(valor);
}

static uint8_t lcd_byte(uint8_t valor, uint8_t es_dato) {
    uint8_t control = es_dato ? LCD_RS : 0U;
    if (!lcd_medio_byte((uint8_t)((valor & 0xF0U) | control))) return 0U;
    if (!lcd_medio_byte((uint8_t)(((valor << 4) & 0xF0U) | control))) return 0U;
    return 1U;
}

static void lcd_comando(uint8_t comando) {
    if (!lcd_disponible) return;
    if (tipo_lcd == LCD_TIPO_NATIVO) {
        if (!lcd_nativo_enviar(LCD_CONTROL_COMANDO, comando)) {
            lcd_disponible = 0U;
            return;
        }
    } else if (!lcd_byte(comando, 0U)) {
        lcd_disponible = 0U;
        return;
    }
    if (comando == 0x01U || comando == 0x02U) {
        _delay_ms(2);
    } else if (tipo_lcd == LCD_TIPO_NATIVO) {
        _delay_us(50);
    }
}

static void lcd_dato(uint8_t dato) {
    if (!lcd_disponible) return;
    if (tipo_lcd == LCD_TIPO_NATIVO) {
        if (!lcd_nativo_enviar(LCD_CONTROL_DATO, dato)) {
            lcd_disponible = 0U;
            return;
        }
        _delay_us(50);
    } else if (!lcd_byte(dato, 1U)) {
        lcd_disponible = 0U;
    }
}

static uint8_t lcd_init(void) {
    static const uint8_t direcciones_pcf[] = {0x27U, 0x3FU};
    uint8_t indice, encontrado = 0U;
    _delay_ms(50);

    /* Proteus JHD-2X16-I2C/AiP31068L: direccion de 7 bits 0x3E. */
    if (twi_iniciar((uint8_t)(LCD_DIRECCION_NATIVA << 1))) {
        twi_stop();
        direccion_lcd = LCD_DIRECCION_NATIVA;
        tipo_lcd = LCD_TIPO_NATIVO;
        lcd_disponible = 1U;
        lcd_comando(0x38U);
        lcd_comando(0x08U);
        lcd_comando(0x01U);
        lcd_comando(0x06U);
        lcd_comando(0x0CU);
        return lcd_disponible;
    }
    twi_stop();

    /* LCD fisico con mochila PCF8574: direcciones habituales 0x27/0x3F. */
    for (indice = 0U; indice < (uint8_t)sizeof(direcciones_pcf); indice++) {
        if (twi_iniciar((uint8_t)(direcciones_pcf[indice] << 1))) {
            direccion_lcd = direcciones_pcf[indice];
            encontrado = 1U;
            twi_stop();
            break;
        }
        twi_stop();
    }
    if (!encontrado) return 0U;

    tipo_lcd = LCD_TIPO_PCF8574;
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

static void linea_limpiar(char linea[17]) {
    uint8_t i;
    for (i = 0U; i < 16U; i++) {
        linea[i] = ' ';
    }
    linea[16] = '\0';
}

static void linea_texto_P(char linea[17], uint8_t posicion, const char *texto_P) {
    uint8_t c;
    while (posicion < 16U && (c = pgm_read_byte(texto_P++)) != 0U) {
        linea[posicion++] = (char)c;
    }
}

static void linea_numero(char linea[17], uint8_t posicion,
                         uint16_t numero, uint8_t cifras) {
    while (cifras > 0U) {
        cifras--;
        linea[posicion + cifras] = (char)('0' + (numero % 10U));
        numero /= 10U;
    }
}

static void lcd_linea(uint8_t fila, const char linea[17]) {
    uint8_t i;
    lcd_comando(fila == 0U ? 0x80U : 0xC0U);
    for (i = 0U; i < 16U; i++) {
        lcd_dato((uint8_t)linea[i]);
    }
}

static void mostrar_activo(void) {
    char linea1[17];
    char linea2[17];
    linea_limpiar(linea1);
    linea_limpiar(linea2);
    linea_texto_P(linea1, 0U, PSTR("ACTIVO ADC:"));
    linea_numero(linea1, 11U, valor_adc_actual, 4U);
    if (lectura_dht_correcta) {
        linea_texto_P(linea2, 0U, PSTR("T:"));
        linea_numero(linea2, 2U, temperatura, 2U);
        linea_texto_P(linea2, 4U, PSTR("C H:"));
        linea_numero(linea2, 8U, humedad, 3U);
        linea2[11] = '%';
    } else if (error_dht != 0U) {
        linea_texto_P(linea2, 0U, PSTR("DHT ERROR:"));
        linea_numero(linea2, 11U, error_dht, 1U);
    } else {
        linea_texto_P(linea2, 0U, PSTR("DHT: ESPERANDO"));
    }
    lcd_linea(0U, linea1);
    lcd_linea(1U, linea2);
}

/* Un buffer y una rutina para los tres mensajes de estado. */
static void mostrar_mensaje_estado_P(const char *titulo_P, const char *detalle_P) {
    char linea[17];
    linea_limpiar(linea);
    linea_texto_P(linea, 0U, titulo_P);
    lcd_linea(0U, linea);
    linea_limpiar(linea);
    linea_texto_P(linea, 0U, detalle_P);
    lcd_linea(1U, linea);
}

/* DHT11: pulso inicial programado y captura breve por sondeo. */

static void liberar_dht(void) {
    DDRD &= (uint8_t)~(1U << DHT_DDR_PIN);
    PORTD |= (1U << DHT_PIN);
}

static void cancelar_dht(void) {
    uint8_t sreg = SREG;
    cli();
    PCMSK2 &= (uint8_t)~(1U << DHT_PCINT_BIT);
    estado_dht = DHT_REPOSO;
    SREG = sreg;
    liberar_dht();
    (void)prr_deshabilitar(PRR_TIMER2);
    bandera_dht = 0U;
}

/* Espera a que DATA deje el nivel indicado, con un limite en microsegundos. */
static uint8_t dht_esperar_mientras(uint8_t nivel_alto, uint16_t timeout_us) {
    while (((PIND & _BV(DHT_PIN)) != 0U) == (nivel_alto != 0U)) {
        if (timeout_us == 0U) return 0U;
        timeout_us--;
        _delay_us(1);
    }
    return 1U;
}

/* La parte critica dura cerca de 4 ms. Se deshabilitan interrupciones para
 * que UART o timers no alteren la medida de los pulsos del DHT11. */
static uint8_t dht_capturar_datos(void) {
    uint8_t i;
    uint8_t correcto = 1U;
    uint8_t sreg = SREG;

    cli();

    /* Respuesta inicial del sensor: 80 us bajo y 80 us alto. */
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

        /* A los 40 us, un cero ya termino y un uno sigue en nivel alto. */
        _delay_us(40);
        dht_datos[i / 8U] <<= 1U;
        if ((PIND & _BV(DHT_PIN)) != 0U) {
            dht_datos[i / 8U] |= 1U;
            if (!dht_esperar_mientras(1U, 100U)) {
                correcto = 0U;
                break;
            }
        }
    }

    SREG = sreg;
    return correcto;
}

static void dht_comenzar(uint32_t ahora) {
    uint8_t sreg = SREG;
    cli();
    PCMSK2 &= (uint8_t)~(1U << DHT_PCINT_BIT);
    dht_datos[0] = 0U;
    dht_datos[1] = 0U;
    dht_datos[2] = 0U;
    dht_datos[3] = 0U;
    dht_datos[4] = 0U;
    estado_dht = DHT_INICIO_BAJO;
    SREG = sreg;
    /* Pulso inicial sin espera activa: el main volvera despues de 20 ms. */
    DDRD |= (1U << DHT_DDR_PIN);
    PORTD &= (uint8_t)~(1U << DHT_PIN);
    dht_marca_ms = ahora;
    dht_ultimo_inicio_ms = ahora;
    dht_intento_previo = 1U;
}

static void dht_liberar_y_capturar(uint32_t ahora) {
    liberar_dht();
    _delay_us(20);

    if (dht_capturar_datos()) {
        estado_dht = DHT_COMPLETO;
    } else {
        estado_dht = DHT_ERROR;
    }

    dht_marca_ms = ahora;
}

static void dht_procesar_resultado(void) {
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
        dht_ultima_lectura_ms = leer_tiempo();
        error_dht = 0U;
    } else {
        lectura_dht_correcta = 0U;
        error_dht = 6U;
    }
    if (solicitud_sensores) sensores_listos = 1U;
    bandera_lcd = 1U;
}

static void tarea_dht(uint32_t ahora) {
    uint8_t copia_estado, sreg;
    if (estado != ESTADO_ACTIVO) {
        cancelar_dht();
        return;
    }
    sreg = SREG;
    cli();
    copia_estado = estado_dht;
    SREG = sreg;
    if (bandera_dht && copia_estado == DHT_REPOSO &&
        (!dht_intento_previo ||
         (uint32_t)(ahora - dht_ultimo_inicio_ms) >= PERIODO_DHT_MS)) {
        bandera_dht = 0U;
        dht_comenzar(ahora);
    } else if (copia_estado == DHT_INICIO_BAJO &&
               (uint32_t)(ahora - dht_marca_ms) >= DHT_INICIO_BAJO_MS) {
        dht_liberar_y_capturar(ahora);
    } else if (copia_estado == DHT_COMPLETO) {
        dht_procesar_resultado();
    } else if (copia_estado == DHT_ERROR) {
        lectura_dht_correcta = 0U;
        error_dht = 7U;
        cancelar_dht();
        if (solicitud_sensores) sensores_listos = 1U;
        bandera_lcd = 1U;
    }
}

/* RELOJ Y RESTAURACION DE PERIFERICOS - ACTIVIDAD 1 */

/* Llamar con interrupciones deshabilitadas. Conserva la fraccion transcurrida. */
static void reloj_capturar_y_detener(void) {
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
        parcial_us += (uint32_t)TCNT1 * 16UL;
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

static void reloj_rapido(void) {
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

static void reloj_standby(void) {
    uint8_t sreg = SREG;
    cli();
    reloj_capturar_y_detener();
    bandera_reloj = 0U;
    modo_reloj = RELOJ_STANDBY;
    timer1_init();                 /* Solo 1 IRQ/s para contar la inactividad. */
    (void)prr_deshabilitar(PRR_TIMER0 | PRR_TIMER2 | PRR_ADC | PRR_SPI);
    SREG = sreg;
}

static void restaurar_para_activo(void) {
    reloj_rapido();
    /* SEGURO ya tenia Timer0 rapido, pero habia quitado el reloj de Timer1. */
    if ((PRR & PRR_TIMER1) != 0U) timer1_init();
    twi_init();                    /* Antes de intentar escribir a la LCD. */
    adc_habilitar();
}

static void int0_flanco_descendente(void) {
    EIMSK &= (uint8_t)~_BV(INT0);
    EICRA = (uint8_t)((EICRA & (uint8_t)~(_BV(ISC01) | _BV(ISC00))) |
                     _BV(ISC01));
    EIFR = _BV(INTF0);
    EIMSK |= _BV(INT0);
}

/* Esta comprobacion se llama con IRQ deshabilitadas. */
static uint8_t hay_evento_pendiente(void) {
    return (bandera_boton || bandera_reloj || bandera_wdt || uart_rx_error ||
            uart_rx_cola != uart_rx_cabeza || (UCSR0A & _BV(RXC0)) != 0U ||
            (PIND & _BV(BOTON_PIN)) == 0U) ? 1U : 0U;
}

static void dormir_idle(void) {
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

static void dormir_powerdown(void) {
    uint8_t sreg, prr_anterior, usart_anterior;
    if (!uart_tx_terminada() || uart_en_trama || uart_trama_pendiente) return;                    /* Termina de enviar el aviso sin bloquear main. */
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
    PCMSK2 &= (uint8_t)~_BV(DHT_PCINT_BIT);
    PCICR &= (uint8_t)~_BV(PCIE2);
    prr_ultimo_sueno = PRR;         /* Copia historica: UART ahora no tiene reloj. */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
#if defined(BODS) && defined(BODSE)
    sleep_bod_disable();
#endif
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
    prr_informar_valor(PSTR("ULTIMO_SUENO"), prr_ultimo_sueno);
    /* main valida el boton y entrar_activo restaura el resto. */
}

static void gestionar_sueno(void) {
    if (estado == ESTADO_STANDBY) {
        dormir_idle();
    } else if (estado == ESTADO_APAGADO) {
        dormir_powerdown();
    }
    /* En SEGURO se conserva el reloj rapido y la alarma original. */
}

/* FSM DEL SISTEMA - SIN SWITCH-CASE */

static void cancelar_solicitud_sensores(void) {
    if (solicitud_sensores) {
        solicitud_sensores = sensores_listos = 0U;
        ERROR_PROTO("SENSORES_CANCELADOS");
    }
}

static void entrar_activo(uint32_t ahora, uint8_t registrar_actividad) {
    modo_led = LED_AUTOMATICO;
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
    (void)uart_enviar_texto_P(PSTR("EV:ACTIVO\r\n"));
    prr_informar(PSTR("ACTIVO"));
    bandera_adc = 1U;
    bandera_lcd = 1U;
    bandera_dht = 1U;
}

static void entrar_standby(void) {
    cancelar_solicitud_sensores();
    modo_led = LED_AUTOMATICO;
    estado = ESTADO_STANDBY;
    adc_deshabilitar();
    cancelar_dht();
    PORTB &= (uint8_t)~(1U << LED_PIN);
    twi_init();                    /* Puede proceder de un despertar APAGADO. */
    lcd_encender_luz();
    lcd_comando(0x0CU);
    mostrar_mensaje_estado_P(PSTR("STANDBY"), PSTR("Esperando..."));
    apagar_sonido();
    reloj_standby();
    (void)prr_fijar_perfil(PRR_STANDBY);
    (void)uart_enviar_texto_P(PSTR("EV:STANDBY\r\n"));
    prr_informar(PSTR("STANDBY"));
}

static void entrar_apagado(void) {
    cancelar_solicitud_sensores();
    modo_led = LED_AUTOMATICO;
    estado = ESTADO_APAGADO;
    adc_deshabilitar();
    cancelar_dht();
    apagar_sonido();
    PORTB &= (uint8_t)~(1U << LED_PIN);
    twi_init();                    /* TWI estaba detenido en STANDBY. */
    lcd_comando(0x0CU);
    mostrar_mensaje_estado_P(PSTR("APAGADO"), PSTR("Pulse el boton"));
    _delay_ms(5);
    lcd_comando(0x08U);
    lcd_apagar_luz();
    reloj_standby();               /* Tambien permite ACTIVO -> APAGADO por orden. */
    (void)prr_fijar_perfil(PRR_STANDBY); /* T1 y UART hasta completar los avisos. */
    (void)uart_enviar_texto_P(PSTR("EV:APAGADO\r\n"));
    prr_informar(PSTR("PREVIO_SUENO"));
}

static void entrar_seguro(uint8_t causa) {
    cancelar_solicitud_sensores();
    registrar_alarma(causa);
    modo_led = LED_AUTOMATICO;
    wdt_al_volver_a_ejecucion();
    restaurar_para_activo();       /* La alarma necesita Timer0 a 1 ms. */
    estado = ESTADO_SEGURO;
    adc_deshabilitar();
    cancelar_dht();
    lcd_encender_luz();
    lcd_comando(0x0CU);
    mostrar_mensaje_estado_P(PSTR("EMERGENCIA"), PSTR("Pulse para salir"));
    iniciar_alarma();
    (void)prr_fijar_perfil(PRR_SEGURO); /* Timer0 mantiene alarma y pulsador. */
    (void)uart_enviar_texto_P(PSTR("EV:SEGURO\r\n"));
    prr_informar(PSTR("SEGURO"));
}

static void revisar_inactividad(uint32_t ahora) {
    uint32_t inactividad = (uint32_t)(ahora - ultima_actividad);
    if (estado == ESTADO_ACTIVO &&
        inactividad >= TIEMPO_STANDBY_MS) {
        entrar_standby();
    } else if (estado == ESTADO_STANDBY &&
               inactividad >= TIEMPO_APAGADO_MS) {
        entrar_apagado();
    }
}

/* PULSADOR */

static void procesar_boton(uint32_t ahora) {
    uint8_t sreg = SREG, evento, salia_de_seguro;
    cli();
    evento = bandera_boton;
    bandera_boton = 0U;
    SREG = sreg;
    if (!evento || boton_presionado || (PIND & _BV(BOTON_PIN)) != 0U) return;
    if ((uint32_t)(ahora - ultimo_boton) < ANTI_REBOTE_MS) return;
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

static void revisar_boton_mantenido(uint32_t ahora) {
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
        entrar_seguro(ALARMA_LOCAL);
    }
}

/* ACTIVIDAD 4 - PARSER Y COMANDOS, TODO FUERA DE LAS ISR
 * @COMANDO,DATO*HH + LF; XOR de COMANDO,DATO; CR opcional antes de LF.
 * Maximo 63 caracteres entre @ y LF; timeout total de 500 ms.
 * Una trama recibida puede esperar espacio de TX sin bloquear main. */

static void protocolo_actividad(uint8_t activar) {
    protocolo_aceptadas++;
    if (activar && estado != ESTADO_ACTIVO && estado != ESTADO_SEGURO) {
        entrar_activo(leer_tiempo(), 1U);
    }
    ultima_actividad = leer_tiempo();
}

static void protocolo_datos_sensores(char *respuesta) {
    COPIAR(respuesta, "ADC=");
    texto_agregar_u32(respuesta, valor_adc_actual);
    texto_anexar_P(respuesta, PSTR(";T="));
    if (lectura_dht_correcta) texto_agregar_u32(respuesta, temperatura);
    else texto_anexar_P(respuesta, PSTR("NA"));
    texto_anexar_P(respuesta, PSTR(";H="));
    if (lectura_dht_correcta) texto_agregar_u32(respuesta, humedad);
    else texto_anexar_P(respuesta, PSTR("NA"));
    texto_anexar_P(respuesta, PSTR(";DHT="));
    texto_agregar_u32(respuesta, lectura_dht_correcta);
    texto_anexar_P(respuesta, PSTR(";EDAD_MS="));
    if (lectura_dht_correcta) texto_agregar_u32(respuesta,
                      (uint32_t)(leer_tiempo() - dht_ultima_lectura_ms));
    else texto_anexar_P(respuesta, PSTR("NA"));
    texto_anexar_P(respuesta, PSTR(";ERR="));
    texto_agregar_u32(respuesta, error_dht);
}

static void protocolo_notificar_sensores(void) {
    char respuesta[80];
    if (solicitud_sensores && sensores_listos &&
        uart_tx_libre() >= PROTO_TRAMA_TAM) {
        protocolo_datos_sensores(respuesta);
        if (uart_enviar_trama(PSTR("SENSORES"), respuesta)) {
            solicitud_sensores = sensores_listos = 0U;
        }
    }
}

static void protocolo_comando(const char *comando, const char *dato) {
    char respuesta[80];
    if (ES(comando, "ESTADO")) {
        if (!ES(dato, "?")) { ERROR_PROTO("DATO"); return; }
        protocolo_actividad(1U);
        uart_informar_estado();
    } else if (ES(comando, "SENSORES")) {
        if (!ES(dato, "?")) { ERROR_PROTO("DATO"); return; }
        if (estado == ESTADO_SEGURO) { ERROR_PROTO("SEGURO"); return; }
        protocolo_actividad(1U);
        valor_adc_actual = adc_leer_a0();
        solicitud_sensores = 1U;
        sensores_listos = 0U;
        bandera_dht = 1U;           /* La FSM respeta >=2 s entre intentos DHT. */
        (void)uart_enviar_trama_P(PSTR("OK"), PSTR("SENSORES;PENDIENTE"));
    } else if (ES(comando, "MODO")) {
        uint8_t destino;
        if (ES(dato, "ACTIVO")) destino = ESTADO_ACTIVO;
        else if (ES(dato, "STANDBY")) destino = ESTADO_STANDBY;
        else if (ES(dato, "APAGADO")) destino = ESTADO_APAGADO;
        else { ERROR_PROTO("DATO"); return; }
        if (estado == ESTADO_SEGURO) { ERROR_PROTO("SEGURO"); return; }
        protocolo_actividad(0U);
        if (estado != destino) {
            if (destino == ESTADO_ACTIVO) entrar_activo(leer_tiempo(), 1U);
            else if (destino == ESTADO_STANDBY) entrar_standby();
            else entrar_apagado();
        }
        COPIAR(respuesta, "MODO;");
        texto_anexar_P(respuesta, nombre_estado_P());
        (void)uart_enviar_trama(PSTR("OK"), respuesta);
    } else if (ES(comando, "LED")) {
        uint8_t ajuste;
        if (ES(dato, "0")) ajuste = LED_FORZADO_OFF;
        else if (ES(dato, "1")) ajuste = LED_FORZADO_ON;
        else if (ES(dato, "AUTO")) ajuste = LED_AUTOMATICO;
        else { ERROR_PROTO("DATO"); return; }
        if (estado == ESTADO_SEGURO) { ERROR_PROTO("SEGURO"); return; }
        protocolo_actividad(1U);
        modo_led = ajuste;
        if (ajuste == LED_FORZADO_OFF) PORTB &= (uint8_t)~_BV(LED_PIN);
        else if (ajuste == LED_FORZADO_ON) PORTB |= _BV(LED_PIN);
        COPIAR(respuesta, "LED;"); strcat(respuesta, dato);
        (void)uart_enviar_trama(PSTR("OK"), respuesta);
    } else if (ES(comando, "SONIDO")) {
        if (!ES(dato, "0") && !ES(dato, "1")) { ERROR_PROTO("DATO"); return; }
        if (estado == ESTADO_SEGURO) { ERROR_PROTO("SEGURO"); return; }
        protocolo_actividad(1U);
        if (ES(dato, "1")) iniciar_sonido(DURACION_SOUNDER_MS);
        else apagar_sonido();
        COPIAR(respuesta, "SONIDO;"); strcat(respuesta, dato);
        (void)uart_enviar_trama(PSTR("OK"), respuesta);
    } else if (ES(comando, "EMERGENCIA")) {
        if (!ES(dato, "1")) { ERROR_PROTO("DATO"); return; }
        protocolo_actividad(0U);
        if (estado != ESTADO_SEGURO) entrar_seguro(ALARMA_REMOTA);
        (void)uart_enviar_trama_P(PSTR("OK"), PSTR("EMERGENCIA;SEGURO"));
    } else if (ES(comando, "ALARMAS")) {
        if (ES(dato, "?")) {
            protocolo_actividad(1U);
            COPIAR(respuesta, "ALARMAS;TOTAL=");
            texto_agregar_u32(respuesta, alarmas_total);
            texto_anexar_P(respuesta, PSTR(";GUARDADAS="));
            texto_agregar_u32(respuesta, alarmas_guardadas);
            texto_anexar_P(respuesta, PSTR(";ACTIVA="));
            texto_agregar_u32(respuesta, estado == ESTADO_SEGURO ? 1U : 0U);
        } else if (dato[0] >= '0' && dato[0] <= '3' && dato[1] == '\0') {
            uint8_t indice = (uint8_t)(dato[0] - '0');
            const alarma_evento_t *registro;
            if (indice >= alarmas_guardadas) { ERROR_PROTO("SIN_EVENTO"); return; }
            protocolo_actividad(1U);
            indice = (uint8_t)((alarmas_cabeza + ALARMAS_TAM - 1U - indice) % ALARMAS_TAM);
            registro = &alarmas[indice];
            COPIAR(respuesta, "ALARMA;ID="); texto_agregar_u32(respuesta, registro->numero);
            texto_anexar_P(respuesta, PSTR(";CAUSA=")); texto_anexar_P(respuesta, nombre_alarma_P(registro->causa));
            texto_anexar_P(respuesta, PSTR(";MS_RELOJ=")); texto_agregar_u32(respuesta, registro->tiempo);
        } else { ERROR_PROTO("DATO"); return; }
        (void)uart_enviar_trama(PSTR("OK"), respuesta);
    } else if (ES(comando, "PRR")) {
        if (!ES(dato, "?")) { ERROR_PROTO("DATO"); return; }
        protocolo_actividad(1U);
        prr_informar(PSTR("ACTUAL"));
    } else if (ES(comando, "BLOQUEO")) {
        if (!ES(dato, "1")) { ERROR_PROTO("DATO"); return; }
        if (estado == ESTADO_SEGURO) { ERROR_PROTO("SEGURO"); return; }
        if (!wdt_solicitar_bloqueo()) ERROR_PROTO("PRUEBA2_NO_ACTIVA");
    } else {
        ERROR_PROTO("COMANDO");
    }
}

static uint8_t protocolo_desde_hex(char c) {
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0xFFU;
}

static void uart_ejecutar_linea(void) {
    uint8_t i, calculado = 0U, alto;
    uint8_t bajo;
    char *asterisco = NULL;
    char *coma = NULL;
    for (i = 0U; i < uart_linea_longitud; i++) {
        if (!asterisco && uart_linea[i] == '*') asterisco = &uart_linea[i];
        if (!coma && uart_linea[i] == ',') coma = &uart_linea[i];
    }
    /* Validar forma y checksum completos ANTES de modificar cualquier estado. */
    if (!asterisco || !coma || coma == uart_linea || coma >= asterisco ||
        coma + 1 == asterisco || (uint8_t)(asterisco - uart_linea) + 3U != uart_linea_longitud) {
        ERROR_PROTO("FORMATO"); return;
    }
    alto = protocolo_desde_hex(asterisco[1]);
    bajo = protocolo_desde_hex(asterisco[2]);
    if (alto == 0xFFU || bajo == 0xFFU) { ERROR_PROTO("FORMATO"); return; }
    for (i = 0U; &uart_linea[i] < asterisco; i++) calculado ^= (uint8_t)uart_linea[i];
    if (calculado != (uint8_t)((alto << 4) | bajo)) { ERROR_PROTO("CHECKSUM"); return; }
    *asterisco = '\0';
    *coma = '\0';
    protocolo_comando(uart_linea, coma + 1);
}

static void protocolo_reiniciar_rx(void) {
    uart_en_trama = 0U;
    uart_trama_pendiente = 0U;
    uart_linea_longitud = 0U;
    uart_descartar_linea = 0U;
}

static void procesar_uart(uint32_t ahora) {
    uint8_t dato, cantidad, sreg = SREG;
    uint8_t error;
    cli();
    error = uart_rx_error;
    uart_rx_error = 0U;
    if (error) uart_rx_cola = uart_rx_cabeza; /* No unir datos a ambos lados de una perdida. */
    SREG = sreg;
    if (error) {
        char motivo[8] = "RX_00";
        protocolo_reiniciar_rx();
        motivo[3] = protocolo_hex(error >> 4);
        motivo[4] = protocolo_hex(error);
        protocolo_errores++;
        (void)uart_enviar_trama(PSTR("ERROR"), motivo);
    }
    if (uart_en_trama && (uint32_t)(ahora - uart_inicio_linea) >= UART_TIMEOUT_LINEA_MS) {
        protocolo_reiniciar_rx();
        ERROR_PROTO("TIMEOUT");
    }
    for (cantidad = 0U; cantidad < PROTO_RX_PRESUPUESTO; cantidad++) {
        if (uart_rx_error) return; /* main vacia RX al comenzar la proxima vuelta. */
        if (uart_trama_pendiente) {
            if (uart_tx_libre() < PROTO_RESERVA_TX) return;
            uart_linea[uart_linea_longitud] = '\0';
            uart_ejecutar_linea();
            protocolo_reiniciar_rx();
            return;               /* Ceder a boton, sensores y WDT tras una orden. */
        }
        if (!uart_tomar_byte(&dato)) return;
        if (dato == '@') {
            if (uart_en_trama) ERROR_PROTO("SIN_FIN");
            protocolo_reiniciar_rx();
            uart_en_trama = 1U;
            uart_inicio_linea = leer_tiempo();
        } else if (dato == '\r' || dato == '\n') {
            if (uart_en_trama) {
                uart_en_trama = 0U;
                uart_trama_pendiente = 1U;
            } else if (uart_descartar_linea) {
                uart_descartar_linea = 0U;
                ERROR_PROTO("INICIO");
            }
        } else if (!uart_en_trama) {
            uart_descartar_linea = 1U;
        } else if (dato < 0x20U || dato > 0x7EU) {
            protocolo_reiniciar_rx();
            ERROR_PROTO("CARACTER");
        } else if (uart_linea_longitud >= UART_LINEA_TAM - 1U) {
            protocolo_reiniciar_rx();
            ERROR_PROTO("LONGITUD");
        } else {
            uart_linea[uart_linea_longitud++] = (char)dato;
            uart_inicio_linea = leer_tiempo();
        }
    }
}

/* TAREAS COOPERATIVAS */

static void ejecutar_tareas(uint32_t ahora) {
    if (bandera_led) {
        bandera_led = 0U;
        if (modo_led == LED_AUTOMATICO) PORTB ^= (1U << LED_PIN);
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

static void descartar_tareas(void) {
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

/* ACTIVIDAD 2 - DOS PRUEBAS INDEPENDIENTES.
 * WDT_PRUEBA_ACTIVA=1: interrupcion de 8 s como despertador.
 * WDT_PRUEBA_ACTIVA=2: reset de 2 s como recuperacion ante bloqueo. */

#if WDT_PRUEBA_ACTIVA == WDT_PRUEBA_DESPERTADOR


static void wdt_configurar_interrupcion_8s(void) {
    uint8_t sreg = SREG;
    uint8_t permiso = _BV(WDCE) | _BV(WDE);
    uint8_t ajuste = _BV(WDIF) | _BV(WDIE) | _BV(WDP3) | _BV(WDP0);
    cli();
    wdt_reset();
    MCUSR &= (uint8_t)~_BV(WDRF);

    __asm__ __volatile__(
        "sts %[registro], %[permiso]\n\t"
        "sts %[registro], %[ajuste]\n\t"
        :
        : [registro] "n" (_SFR_MEM_ADDR(WDTCSR)),
          [permiso] "r" (permiso), [ajuste] "r" (ajuste)
        : "memory"
    );

    wdt_armado = 1U;
    SREG = sreg;
}

static void wdt_iniciar_prueba(void) {
    wdt_disable();
    wdt_armado = 0U;
    bandera_wdt = 0U;
    (void)uart_enviar_texto_P(PSTR("PRUEBA1:IRQ_8S\r\n"));

}

static void wdt_antes_powerdown(void) {
    if (!wdt_armado) {
        wdt_configurar_interrupcion_8s();
    }

}

static void wdt_despues_powerdown(void) {
    /* WDIE fue borrado por hardware al ejecutar ISR(WDT_vect).
       El rearme se realiza en main al procesar el evento. */
}

static void wdt_al_volver_a_ejecucion(void) {
    wdt_disable();
    wdt_armado = 0U;
    bandera_wdt = 0U;
    PORTB &= (uint8_t)~_BV(WDT_PRUEBA_PIN);
}

static void wdt_procesar_evento(void) {
    uint8_t pendiente, sreg = SREG;
    uint32_t numero_evento;
    char mensaje[64];
    cli();
    pendiente = bandera_wdt;
    bandera_wdt = 0U;
    numero_evento = eventos_wdt;
    SREG = sreg;
    if (!pendiente) return;
    wdt_armado = 0U;
    if (estado != ESTADO_APAGADO) {
        PORTB &= (uint8_t)~_BV(WDT_PRUEBA_PIN);
        return;
    }

    adc_habilitar();
    valor_adc_actual = adc_leer_a0();
    adc_deshabilitar();
    (void)prr_deshabilitar(PRR_ADC);
    texto_copiar_P(mensaje, PSTR("WDT,evento="));
    texto_agregar_u32(mensaje, numero_evento);
    texto_anexar_P(mensaje, PSTR(",ADC="));
    texto_agregar_u32(mensaje, valor_adc_actual);
    texto_anexar_P(mensaje, PSTR(",nominal_ms=8000\r\n"));
    (void)uart_enviar_texto(mensaje);
    wdt_configurar_interrupcion_8s();


}

static void wdt_supervisar_ciclo(void) {

}

static uint8_t wdt_solicitar_bloqueo(void) {
    return 0U;
}
#else







static void wdt_iniciar_prueba(void) {
    wdt_armado = 0U;
    bandera_wdt = 0U;
    wdt_reset();
    wdt_enable(WDTO_2S);
    (void)uart_enviar_texto_P(PSTR("PRUEBA2:RESET_2S\r\n"));
}

static void wdt_antes_powerdown(void) {


    wdt_disable();
}

static void wdt_despues_powerdown(void) {
    wdt_reset();
    wdt_enable(WDTO_2S);
}

static void wdt_al_volver_a_ejecucion(void) {

}

static void wdt_procesar_evento(void) {

    bandera_wdt = 0U;
}

static void wdt_supervisar_ciclo(void) {

    wdt_reset();
}

static uint8_t wdt_solicitar_bloqueo(void) {
    protocolo_actividad(0U);
    (void)uart_enviar_texto_P(PSTR("PRUEBA2:BLOQUEO_SIN_WDR\r\n"));
    marca_bloqueo = MARCA_BLOQUEO_WDT;
    marca_bloqueo_inversa = ~MARCA_BLOQUEO_WDT;
    PORTB |= _BV(WDT_PRUEBA_PIN);
    wdt_reset();
    for (;;) {


    }
    return 1U;
}


#endif

/* PROGRAMA PRINCIPAL */

int main(void) {
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
        linea_texto_P(mensaje1, 0U, PSTR("LAB 5 - B UART"));
        linea_texto_P(mensaje2, 0U, PSTR("INICIANDO..."));
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
    (void)uart_enviar_texto_P(PSTR("LAB5:B4\r\n"));
    if ((causa_reinicio & _BV(WDRF)) != 0U ||
        (((causa_reinicio & _BV(PORF)) == 0U) &&
         marca_bloqueo == MARCA_BLOQUEO_WDT &&
         marca_bloqueo_inversa == ~MARCA_BLOQUEO_WDT)) {
        entrar_seguro((causa_reinicio & _BV(WDRF)) != 0U ?
                      ALARMA_WDT : ALARMA_TRAS_BLOQUEO);
        if ((causa_reinicio & _BV(WDRF)) != 0U) {
            (void)uart_enviar_texto_P(PSTR("RESET:WDT\r\n"));
        } else {
            /* No atribuir causa si el cargador de arranque borro MCUSR. */
            (void)uart_enviar_texto_P(PSTR("RESET:TRAS_BLOQUEO_SIN_WDRF\r\n"));
        }
        (void)uart_enviar_texto_P(PSTR("RECUPERACION:SEGURO\r\n"));
    } else {
        mostrar_activo();
        (void)prr_fijar_perfil(PRR_ACTIVO);
        (void)uart_enviar_texto_P(PSTR("EV:ACTIVO\r\n"));
        prr_informar(PSTR("ACTIVO"));
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
        protocolo_notificar_sensores();
        wdt_procesar_evento();     /* Lectura/registro fuera de la ISR. */
        wdt_supervisar_ciclo();    /* Solo la prueba 2 habilitada ejecuta WDR aqui. */
        gestionar_sueno();         /* Solo duerme en STANDBY o APAGADO. */
    }
}
