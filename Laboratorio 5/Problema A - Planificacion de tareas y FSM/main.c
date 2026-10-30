/*
 * LABORATORIO 5 - PROBLEMA A
 * ATmega328P / Arduino Uno - 16 MHz - avr-gcc
 *
 * Arquitectura:
 * - Las ISR solo actualizan contadores, banderas o capturan pulsos breves.
 * - Las tareas se ejecutan en el lazo principal.
 * - La FSM utiliza if/else; no utiliza switch-case.
 * - La lectura del DHT11 se inicia de forma temporal y sus pulsos se
 *   capturan con PCINT20 + Timer2, sin detener Timer0 ni Timer1.
 *
 * Pines:
 *   D13 / PB5 -> LED
 *   D11 / PB3 -> sounder pasivo
 *   D4  / PD4 -> DATA DHT11 (requiere pull-up)
 *   D2  / PD2 -> pulsador INT0 contra GND
 *   A0  / PC0 -> cursor del potenciometro
 *   A4  / PC4 -> SDA LCD I2C
 *   A5  / PC5 -> SCL LCD I2C
 */

/*
 * F_CPU no configura fisicamente el reloj.
 * Solo le informa al compilador que el micro trabaja a 16 MHz, para que
 * funciones como _delay_ms() calculen correctamente sus tiempos.
 */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/*
 * avr/io.h: nombres de registros y bits del ATmega328P.
 * avr/interrupt.h: definicion de ISR, sei() y cli().
 * util/delay.h: retardos breves necesarios para LCD e inicializacion.
 * util/twi.h: codigos de estado del periferico TWI/I2C.
 * stdint.h: tipos de tamano conocido, por ejemplo uint8_t y uint32_t.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdint.h>
#include <stdio.h>

/*
 * Se usan nombres del puerto del ATmega328P y no numeros de Arduino:
 * PB5 corresponde a D13, PB3 a D11, PD4 a D4 y PD2 a D2.
 */
#define LED_PIN                         PB5
#define SOUNDER_PIN                     PB3
#define DHT_PIN                         PD4
#define BOTON_PIN                       PD2

/*
 * Mascaras del expansor PCF8574 del LCD:
 * RS selecciona comando/dato, E genera el pulso de habilitacion y
 * LCD_LUZ controla la retroiluminacion.
 */
#define LCD_RS                          0x01U
#define LCD_E                           0x04U
#define LCD_LUZ                         0x08U

/*
 * Todos estos tiempos se expresan en milisegundos y se comparan contra
 * la base temporal de 1 ms generada por Timer0.
 */
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

/* Timer2: 16 MHz / 8 = 2 MHz, 0,5 us por cuenta. */
#define DHT_RESPUESTA_MIN_CUENTAS       100U
#define DHT_RESPUESTA_MAX_CUENTAS       220U
#define DHT_BIT_BAJO_MIN_CUENTAS        50U
#define DHT_BIT_BAJO_MAX_CUENTAS        150U
#define DHT_UMBRAL_BIT_UNO_CUENTAS      100U
#define DHT_INICIO_BAJO_MS              20UL
#define DHT_TIMEOUT_MS                  10UL

/*
 * Estados de la maquina principal.
 * Los tres primeros los pide la consigna. ESTADO_SEGURO representa
 * la condicion de emergencia solicitada para cualquier estado.
 */
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

/*
 * volatile indica que estas variables pueden cambiar dentro de una ISR.
 * De esta manera el compilador siempre vuelve a leer su valor real.
 * tiempo_ms es de 32 bits para poder contar intervalos largos.
 */
static volatile uint32_t tiempo_ms = 0UL;
static volatile uint16_t contador_led_ms = 0U;
static volatile uint16_t contador_lcd_ms = 0U;
static volatile uint16_t contador_dht_ms = 0U;
static volatile uint16_t tiempo_sonido_ms = 0U;

/*
 * Las banderas conectan las interrupciones con el lazo principal:
 * la ISR coloca una bandera en 1 y main ejecuta la tarea pendiente.
 */
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
static uint32_t dht_marca_ms = 0UL;

/* =========================================================
   INTERRUPCIONES
   ========================================================= */

/*
 * Timer0 interrumpe cada 1 ms.
 * La ISR cuenta tiempo y levanta banderas; no actualiza el LCD ni lee
 * sensores, porque ese trabajo demoraria demasiado dentro de una ISR.
 */
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
/*
 * Timer1 vence cada 100 ms. Solo solicita la tarea del ADC.
 * La conversion se realiza despues, fuera de la interrupcion.
 */
ISR(TIMER1_COMPA_vect)
{
    bandera_adc = 1U;
}

/*
 * INT0 detecta el flanco descendente producido al pulsar el boton.
 * El boton esta contra GND y utiliza la resistencia pull-up interna.
 */
ISR(INT0_vect)
{
    bandera_boton = 1U;
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
            estado_dht = DHT_ERROR;
            PCMSK2 &= (uint8_t)~(1U << PCINT20);
        }
    } else if (estado_dht == DHT_RESPUESTA_ALTA) {
        if (nivel == 0U &&
            duracion >= DHT_RESPUESTA_MIN_CUENTAS &&
            duracion <= DHT_RESPUESTA_MAX_CUENTAS) {
            estado_dht = DHT_BIT_BAJO;
        } else {
            estado_dht = DHT_ERROR;
            PCMSK2 &= (uint8_t)~(1U << PCINT20);
        }
    } else if (estado_dht == DHT_BIT_BAJO) {
        if (nivel == 1U &&
            duracion >= DHT_BIT_BAJO_MIN_CUENTAS &&
            duracion <= DHT_BIT_BAJO_MAX_CUENTAS) {
            estado_dht = DHT_BIT_ALTO;
        } else {
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
            estado_dht = DHT_ERROR;
            PCMSK2 &= (uint8_t)~(1U << PCINT20);
        }
    }
}

/* =========================================================
   TIEMPO Y SALIDAS
   ========================================================= */

/*
 * tiempo_ms ocupa 32 bits y el AVR es de 8 bits.
 * Se deshabilitan las interrupciones unos instantes para copiarlo sin que
 * Timer0 lo modifique a mitad de la lectura. Luego se restaura SREG.
 */
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
   GPIO Y TEMPORIZADORES
   ========================================================= */

static void gpio_init(void)
{
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
    /*
     * Modo CTC: el contador vuelve a cero al coincidir con OCR0A.
     * 16 MHz / 64 = 250 kHz.
     * Cada cuenta dura 4 us y 250 cuentas (0 a 249) forman 1 ms.
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
     * Timer1 tambien trabaja en CTC con prescaler 64.
     * 16 MHz / 64 = 250 kHz y 25000 cuentas forman 100 ms.
     * Se escribe 24999 porque el contador comienza en cero.
     */
    TCCR1A = 0U;
    TCCR1B = (1U << WGM12) | (1U << CS11) | (1U << CS10);
    OCR1A = 24999U;
    TCNT1 = 0U;
    TIMSK1 = (1U << OCIE1A);
}

static void timer2_init(void)
{
    /*
     * Timer2 se usa como cronometro de pulsos del DHT11.
     * 16 MHz / 8 = 2 MHz, por lo tanto cada cuenta representa 0,5 us.
     * No necesita una ISR propia: PCINT lee directamente TCNT2.
     */
    TCCR2A = 0U;
    TCCR2B = (1U << CS21);
    TCNT2 = 0U;
    TIMSK2 = 0U;
    TIFR2 = (1U << TOV2);
}

/* =========================================================
   ADC Y ACTIVIDAD DEL POTENCIOMETRO
   ========================================================= */

static void ADC_init(void)
{
    DDRC &= (uint8_t)~(1U << DDC0);
    PORTC &= (uint8_t)~(1U << PORTC0);

    /* AVcc como referencia y reloj del ADC a 125 kHz. */
    ADMUX = (1U << REFS0);
    ADCSRA = (1U << ADEN) |
             (1U << ADPS2) | (1U << ADPS1) | (1U << ADPS0);
}

static void ADC_disable(void)
{
    ADCSRA &= (uint8_t)~(1U << ADEN);
}

static uint16_t ADC_read(uint8_t canal)
{
    if ((ADCSRA & (1U << ADEN)) == 0U) {
        ADC_init();
    }

    canal &= 0x07U;
    ADMUX = (uint8_t)((ADMUX & 0xF0U) | canal);
    ADCSRA |= (1U << ADSC);

    while ((ADCSRA & (1U << ADSC)) != 0U) {
        /* Espera breve propia de la conversion. */
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
   I2C / LCD 16x2 CON PCF8574
   ========================================================= */

static uint8_t i2c_esperar(void)
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

/*
 * Inicializa el TWI, que es el nombre que AVR utiliza para el bus I2C.
 * A4 es SDA y A5 es SCL. El bus queda configurado aproximadamente a 100 kHz.
 */
static void i2c_init(void)
{
    DDRC &= (uint8_t)~((1U << DDC4) | (1U << DDC5));
    PORTC |= (1U << PORTC4) | (1U << PORTC5);

    TWSR = 0U;
    TWBR = (uint8_t)(((F_CPU / 100000UL) - 16UL) / 2UL);
    TWCR = (1U << TWEN);
}

static uint8_t i2c_start(uint8_t direccion_escritura)
{
    uint8_t estado_twi;

    TWCR = (1U << TWINT) | (1U << TWSTA) | (1U << TWEN);
    if (!i2c_esperar()) {
        return 0U;
    }

    estado_twi = TWSR & 0xF8U;
    if (estado_twi != TW_START && estado_twi != TW_REP_START) {
        return 0U;
    }

    TWDR = direccion_escritura;
    TWCR = (1U << TWINT) | (1U << TWEN);
    if (!i2c_esperar()) {
        return 0U;
    }

    return ((TWSR & 0xF8U) == TW_MT_SLA_ACK) ? 1U : 0U;
}

static uint8_t i2c_write(uint8_t dato)
{
    TWDR = dato;
    TWCR = (1U << TWINT) | (1U << TWEN);

    if (!i2c_esperar()) {
        return 0U;
    }

    return ((TWSR & 0xF8U) == TW_MT_DATA_ACK) ? 1U : 0U;
}

static void i2c_stop(void)
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

    correcto = i2c_start((uint8_t)(direccion_lcd << 1));
    if (correcto) {
        correcto = i2c_write((uint8_t)(valor | luz_lcd));
    }
    i2c_stop();

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

static uint8_t lcd_enviar_nibble(uint8_t valor)
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

static uint8_t lcd_i2c_write(uint8_t dato, uint8_t es_dato)
{
    uint8_t control = es_dato ? LCD_RS : 0U;

    if (!lcd_enviar_nibble((uint8_t)((dato & 0xF0U) | control))) {
        return 0U;
    }

    return lcd_enviar_nibble(
        (uint8_t)(((dato << 4) & 0xF0U) | control));
}

static void lcd_comando(uint8_t comando)
{
    if (!lcd_disponible) {
        return;
    }

    if (!lcd_i2c_write(comando, 0U)) {
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

    if (!lcd_i2c_write(dato, 1U)) {
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
        if (i2c_start((uint8_t)(direcciones_posibles[indice] << 1))) {
            direccion_lcd = direcciones_posibles[indice];
            encontrado = 1U;
            i2c_stop();
            break;
        }
        i2c_stop();
    }

    if (!encontrado) {
        return 0U;
    }

    lcd_disponible = 1U;

    (void)lcd_enviar_nibble(0x30U);
    _delay_ms(5);
    (void)lcd_enviar_nibble(0x30U);
    _delay_us(150);
    (void)lcd_enviar_nibble(0x30U);
    (void)lcd_enviar_nibble(0x20U);

    lcd_comando(0x28U);
    lcd_comando(0x0CU);
    lcd_comando(0x01U);
    lcd_comando(0x06U);

    return lcd_disponible;
}

static void lcd_linea(uint8_t fila, const char *linea)
{
    uint8_t i;
    uint8_t fin = 0U;

    lcd_comando(fila == 0U ? 0x80U : 0xC0U);
    for (i = 0U; i < 16U; i++) {
        if (!fin && *linea != '\0') {
            lcd_dato((uint8_t)*linea++);
        } else {
            fin = 1U;
            lcd_dato((uint8_t)' ');
        }
    }
}

static void mostrar_activo(void)
{
    char linea1[17];
    char linea2[17];

    (void)snprintf(linea1, sizeof(linea1),
                   "ACTIVO ADC:%4u", valor_adc_actual);

    if (lectura_dht_correcta) {
        (void)snprintf(linea2, sizeof(linea2),
                       "T:%uC H:%u%%", temperatura, humedad);
    } else {
        (void)snprintf(linea2, sizeof(linea2), "DHT: ESPERANDO");
    }

    lcd_linea(0U, linea1);
    lcd_linea(1U, linea2);
}

static void mostrar_standby(void)
{
    lcd_linea(0U, "STANDBY");
    lcd_linea(1U, "Esperando...");
}

static void mostrar_apagado(void)
{
    lcd_linea(0U, "APAGADO");
    lcd_linea(1U, "Pulse el boton");
}

static void mostrar_seguro(void)
{
    lcd_linea(0U, "EMERGENCIA");
    lcd_linea(1U, "Pulse para salir");
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
    }

    bandera_lcd = 1U;
}

static void tarea_dht(uint32_t ahora)
{
    uint8_t copia_estado;
    uint8_t sreg;

    if (estado != ESTADO_ACTIVO) {
        cancelar_dht();
        return;
    }

    sreg = SREG;
    cli();
    copia_estado = estado_dht;
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
        cancelar_dht();
        bandera_lcd = 1U;
    } else if (copia_estado >= DHT_ESPERA_RESPUESTA_BAJA &&
               copia_estado <= DHT_BIT_ALTO &&
               (uint32_t)(ahora - dht_marca_ms) >= DHT_TIMEOUT_MS) {
        cancelar_dht();
        bandera_lcd = 1U;
    }
}

/* =========================================================
   FSM DEL SISTEMA - SIN SWITCH-CASE
   ========================================================= */

static void entrar_activo(uint32_t ahora, uint8_t registrar_actividad)
{
    uint8_t estado_interrupciones;

    alarma_continua = 0U;
    ADC_init();
    lcd_encender_luz();
    lcd_comando(0x0CU);

    estado = ESTADO_ACTIVO;
    PORTB &= (uint8_t)~(1U << LED_PIN);

    if (registrar_actividad) {
        ultima_actividad = ahora;
    }

    /*
     * El ADC y el LCD se actualizan apenas el sistema vuelve a ACTIVO.
     */
    bandera_adc = 1U;
    bandera_lcd = 1U;

    /*
     * El DHT11 no se lee inmediatamente despues de despertar.
     * Reiniciamos el contador para esperar nuevamente 2 segundos,
     * igual que despues de encender o reiniciar el Arduino.
     */
    estado_interrupciones = SREG;
    cli();
    contador_dht_ms = 0U;
    bandera_dht = 0U;
    SREG = estado_interrupciones;
}

static void entrar_standby(void)
{
    estado = ESTADO_STANDBY;
    ADC_disable();
    cancelar_dht();
    PORTB &= (uint8_t)~(1U << LED_PIN);
    lcd_encender_luz();
    lcd_comando(0x0CU);
    mostrar_standby();
}

static void entrar_apagado(void)
{
    estado = ESTADO_APAGADO;
    ADC_disable();
    cancelar_dht();
    apagar_sonido();
    PORTB &= (uint8_t)~(1U << LED_PIN);

    lcd_comando(0x0CU);
    mostrar_apagado();
    _delay_ms(5);
    lcd_comando(0x08U);
    lcd_apagar_luz();
}

static void entrar_seguro(void)
{
    estado = ESTADO_SEGURO;
    ADC_disable();
    cancelar_dht();
    lcd_encender_luz();
    lcd_comando(0x0CU);
    mostrar_seguro();
    iniciar_alarma();
}

/*
 * Se resta tiempo actual menos ultima actividad.
 * Al usar enteros sin signo, esta forma sigue funcionando aunque el contador
 * de milisegundos desborde despues de mucho tiempo.
 */
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

/*
 * La ISR avisa que hubo un flanco y esta funcion interpreta el evento.
 * El filtro de 50 ms evita que los rebotes mecanicos cuenten como varias
 * pulsaciones diferentes.
 */
static void procesar_boton(uint32_t ahora)
{
    if (!bandera_boton) {
        return;
    }

    bandera_boton = 0U;

    if ((uint32_t)(ahora - ultimo_boton) < ANTI_REBOTE_MS) {
        return;
    }

    ultimo_boton = ahora;

    if (estado == ESTADO_SEGURO) {
        apagar_sonido();
        boton_presionado = 1U;
        pulsacion_larga_lista = 1U;
        inicio_boton = ahora;
        entrar_activo(ahora, 1U);
        iniciar_sonido(DURACION_SOUNDER_MS);
    } else {
        boton_presionado = 1U;
        pulsacion_larga_lista = 0U;
        inicio_boton = ahora;

        if (estado != ESTADO_ACTIVO) {
            entrar_activo(ahora, 1U);
        } else {
            ultima_actividad = ahora;
        }

        iniciar_sonido(DURACION_SOUNDER_MS);
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
        (uint32_t)(ahora - inicio_boton) >=
            PULSACION_EMERGENCIA_MS) {
        pulsacion_larga_lista = 1U;
        entrar_seguro();
    }
}

/* =========================================================
   TAREAS COOPERATIVAS
   ========================================================= */

/*
 * Planificador cooperativo: cada tarea se ejecuta solo si su bandera esta
 * activa. Al terminar, la bandera vuelve a cero.
 */
static void ejecutar_tareas(uint32_t ahora)
{
    if (bandera_led) {
        bandera_led = 0U;
        PORTB ^= (1U << LED_PIN);
    }

    if (bandera_adc) {
        bandera_adc = 0U;
        valor_adc_actual = ADC_read(0U);
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

/*
 * main primero configura el hardware y despues entra en un lazo infinito.
 * El programa embebido no termina: siempre atiende eventos y tareas.
 */
int main(void)
{
    uint32_t ahora;

    gpio_init();
    ADC_init();
    timer2_init();
    i2c_init();
    (void)lcd_init();

    if (lcd_disponible) {
        lcd_linea(0U, "LAB 5 - PROB A");
        lcd_linea(1U, "INICIANDO...");
        _delay_ms(800);
    }

    timer0_init();
    timer1_init();

    valor_adc_actual = ADC_read(0U);
    referencia_adc_actividad = valor_adc_actual;
    referencia_adc_valida = 1U;
    ultima_actividad = 0UL;
    mostrar_activo();

    /*
     * sei() habilita globalmente las interrupciones despues de terminar
     * toda la configuracion. A partir de aqui empiezan los ticks y eventos.
     */
    sei();

    while (1) {
        ahora = leer_tiempo();

        procesar_boton(ahora);
        revisar_boton_mantenido(ahora);

        if (estado != ESTADO_SEGURO) {
            revisar_inactividad(ahora);
        }

        /*
         * Las tareas periodicas completas solo funcionan en ACTIVO.
         * En los otros estados se limpian las solicitudes pendientes para
         * no ejecutarlas todas juntas cuando el sistema vuelva a despertar.
         */
        if (estado == ESTADO_ACTIVO) {
            ejecutar_tareas(ahora);
        } else {
            descartar_tareas();
        }
    }
}
