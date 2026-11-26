#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdint.h>

/*
 * AUTO CONCURSO - Arduino Uno / ATmega328P a 16 MHz
 *
 * Bluetooth HC-05: TX del modulo -> D0 (RX del Arduino), 9600 baudios.
 *
 * L298N:
 *   Motor izquierdo: ENA = D5, IN1 = D7,  IN2 = D8
 *   Motor derecho:   ENB = D6, IN3 = D11, IN4 = D12
 *
 * Servos:
 *   D9  -> servo corto, abre y cierra la garra.
 *   D10 -> servo largo, sube y baja el brazo.
 *
 * Sensor HC-SR04: ECHO = A0, TRIG = A1.
 *
 * Comandos de la app:
 *   M<marcha>,<giro> + salto de linea, cada 50 ms.
 *   U = subir, B = bajar, V = mantener altura, G = alternar garra.
 *   El Arduino responde P,<distancia_cm>,<alerta> cada 200 ms.
 */

// Ajustes del montaje
#define PWM_IZQUIERDO       175
#define PWM_DERECHO         190

#define GARRA_ABIERTA_US   1420
#define GARRA_CERRADA_US   1300

#define BRAZO_ABAJO_US     2200
#define BRAZO_MEDIO_US     1475
#define BRAZO_ARRIBA_US    750

#define PASO_SERVO_US        10
#define PERIODO_SERVO_MS     20
#define TIEMPO_CORTE_MS     300

#define DISTANCIA_ALERTA_CM   20
#define DISTANCIA_LIBERAR_CM  25
#define PERIODO_MEDICION_MS   70
#define PERIODO_ENVIO_MS     200
#define TIMEOUT_ECO_US     30000UL
#define TX_BUFFER_TAM          16

#if TX_BUFFER_TAM < 10
#error El buffer TX no alcanza para la trama de proximidad
#endif

#define SENSOR_REPOSO          0
#define SENSOR_ESPERA_SUBIDA   1
#define SENSOR_ESPERA_BAJADA   2

static volatile uint32_t reloj_ms = 0;
static uint32_t ultima_trama_ms = 0;
static uint32_t ultimo_paso_ms = 0;
static uint8_t enlace_activo = 0;

static uint16_t brazo_actual = BRAZO_MEDIO_US;
static uint16_t brazo_objetivo = BRAZO_MEDIO_US;
static uint16_t garra_actual = GARRA_ABIERTA_US;
static uint16_t garra_objetivo = GARRA_ABIERTA_US;
static uint8_t garra_abierta = 1;

static uint8_t estado_sensor = SENSOR_REPOSO;
static uint32_t inicio_espera_eco_us = 0;
static uint32_t inicio_pulso_eco_us = 0;
static uint32_t ultima_medicion_ms = 0;
static uint32_t ultimo_envio_ms = 0;
static uint16_t distancia_cm = 0xFFFFU;
static uint8_t alerta_proximidad = 0;

static volatile char tx_buffer[TX_BUFFER_TAM];
static volatile uint8_t tx_longitud = 0;
static volatile uint8_t tx_posicion = 0;

enum { RX_LIBRE, RX_MOVIMIENTO, RX_DESCARTAR };
static uint8_t estado_rx = RX_LIBRE;
static char trama[16];
static uint8_t longitud = 0;

// ---------------- RELOJ DE 1 ms ----------------

ISR(TIMER2_COMPA_vect)
{
    reloj_ms++;
}

static uint32_t tiempo_ms(void)
{
    uint32_t copia;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        copia = reloj_ms;
    }

    return copia;
}

static uint32_t tiempo_us(void)
{
    uint32_t milisegundos;
    uint8_t cuenta;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        milisegundos = reloj_ms;
        cuenta = TCNT2;

        if ((TIFR2 & (1 << OCF2A)) && cuenta < 249U) {
            milisegundos++;
            cuenta = TCNT2;
        }
    }

    return (milisegundos * 1000UL) + ((uint32_t)cuenta * 4UL);
}

static void iniciar_reloj(void)
{
    TCCR2B = 0;
    TCCR2A = (1 << WGM21);
    TCNT2 = 0;
    OCR2A = 249;
    TIFR2 = (1 << OCF2A);
    TIMSK2 = (1 << OCIE2A);
    TCCR2B = (1 << CS22);  // 16 MHz / 64 / 250 = 1000 Hz
}

// ---------------- MOTORES ----------------

static void iniciar_motores(void)
{
    // Arranque con las dos salidas del puente detenidas.
    PORTD &= ~((1 << PD5) | (1 << PD6) | (1 << PD7));
    PORTB &= ~((1 << PB0) | (1 << PB3) | (1 << PB4));

    DDRD |= (1 << PD5) | (1 << PD6) | (1 << PD7);
    DDRB |= (1 << PB0) | (1 << PB3) | (1 << PB4);

    // Timer0 genera PWM en D5 y D6. El reloj del programa usa Timer2.
    TCCR0B = 0;
    TIMSK0 = 0;
    TCNT0 = 0;
    OCR0A = 0;
    OCR0B = 0;
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00);
    TCCR0B = (1 << CS01) | (1 << CS00);
}

static int16_t limitar_porcentaje(int16_t valor)
{
    if (valor > 100) return 100;
    if (valor < -100) return -100;
    return valor;
}

static void motor_izquierdo(int16_t pwm)
{
    PORTD &= ~(1 << PD7);  // IN1
    PORTB &= ~(1 << PB0);  // IN2

    if (pwm > 0) PORTD |= (1 << PD7);
    if (pwm < 0) PORTB |= (1 << PB0);

    OCR0B = (uint8_t)(pwm < 0 ? -pwm : pwm);  // ENA, D5
}

static void motor_derecho(int16_t pwm)
{
    PORTB &= ~((1 << PB3) | (1 << PB4));  // IN3 e IN4

    if (pwm > 0) PORTB |= (1 << PB3);
    if (pwm < 0) PORTB |= (1 << PB4);

    OCR0A = (uint8_t)(pwm < 0 ? -pwm : pwm);  // ENB, D6
}

static void mover_auto(int16_t marcha, int16_t giro)
{
    int16_t avance = -marcha;
    int16_t izquierda = limitar_porcentaje(avance + giro);
    int16_t derecha = limitar_porcentaje(avance - giro);

    motor_izquierdo(izquierda * PWM_IZQUIERDO / 100);
    motor_derecho(derecha * PWM_DERECHO / 100);
}

// ---------------- BRAZO Y GARRA ----------------

static void iniciar_servos(void)
{
    // Timer1: pulsos cada 20 ms. Cada cuenta equivale a 0,5 us.
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;

    DDRB |= (1 << PB1) | (1 << PB2);
    ICR1 = 39999;
    OCR1A = garra_actual * 2U - 1U;
    OCR1B = brazo_actual * 2U - 1U;

    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
}

static uint16_t acercar(uint16_t actual, uint16_t objetivo)
{
    if (actual < objetivo) {
        if (objetivo - actual <= PASO_SERVO_US) return objetivo;
        return actual + PASO_SERVO_US;
    }

    if (actual > objetivo) {
        if (actual - objetivo <= PASO_SERVO_US) return objetivo;
        return actual - PASO_SERVO_US;
    }

    return actual;
}

static void actualizar_servos(uint32_t ahora)
{
    if ((uint32_t)(ahora - ultimo_paso_ms) < PERIODO_SERVO_MS) return;

    ultimo_paso_ms = ahora;

    garra_actual = acercar(garra_actual, garra_objetivo);
    brazo_actual = acercar(brazo_actual, brazo_objetivo);

    OCR1A = garra_actual * 2U - 1U;  // D9
    OCR1B = brazo_actual * 2U - 1U;  // D10
}

// ---------------- SENSOR ULTRASONICO ----------------

static void iniciar_sensor(void)
{
    // TRIG en A1 y ECHO en A0.
    DDRC |= (1 << PC1);
    PORTC &= ~(1 << PC1);

    DDRC &= ~(1 << PC0);
    PORTC &= ~(1 << PC0);
}

static void iniciar_medicion(void)
{
    PORTC &= ~(1 << PC1);
    _delay_us(2);
    PORTC |= (1 << PC1);
    _delay_us(10);
    PORTC &= ~(1 << PC1);

    inicio_espera_eco_us = tiempo_us();
    estado_sensor = SENSOR_ESPERA_SUBIDA;
}

static void medicion_sin_eco(void)
{
    distancia_cm = 0xFFFFU;
    alerta_proximidad = 0;
    estado_sensor = SENSOR_REPOSO;
}

static void actualizar_sensor(uint32_t ahora_ms)
{
    uint32_t ahora_us = tiempo_us();

    if (estado_sensor == SENSOR_REPOSO) {
        if ((uint32_t)(ahora_ms - ultima_medicion_ms) >=
            PERIODO_MEDICION_MS) {
            ultima_medicion_ms = ahora_ms;
            iniciar_medicion();
        }

        return;
    }

    if (estado_sensor == SENSOR_ESPERA_SUBIDA) {
        if (PINC & (1 << PC0)) {
            inicio_pulso_eco_us = ahora_us;
            estado_sensor = SENSOR_ESPERA_BAJADA;
        } else if ((uint32_t)(ahora_us - inicio_espera_eco_us) >=
                   TIMEOUT_ECO_US) {
            medicion_sin_eco();
        }

        return;
    }

    if (estado_sensor == SENSOR_ESPERA_BAJADA) {
        if (!(PINC & (1 << PC0))) {
            uint32_t duracion_us = ahora_us - inicio_pulso_eco_us;
            distancia_cm = (uint16_t)(duracion_us / 58UL);

            if (!alerta_proximidad &&
                distancia_cm > 0U &&
                distancia_cm <= DISTANCIA_ALERTA_CM) {
                alerta_proximidad = 1;
            } else if (alerta_proximidad &&
                       distancia_cm >= DISTANCIA_LIBERAR_CM) {
                alerta_proximidad = 0;
            }

            estado_sensor = SENSOR_REPOSO;
        } else if ((uint32_t)(ahora_us - inicio_pulso_eco_us) >=
                   TIMEOUT_ECO_US) {
            medicion_sin_eco();
        }
    }
}

// ---------------- BLUETOOTH ----------------

static void iniciar_bluetooth(void)
{
    UCSR0B = 0;
    UCSR0A = 0;
    UBRR0H = 0;
    UBRR0L = 103;  // 9600 baudios a 16 MHz
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8 bits, 1 stop
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
}

ISR(USART_UDRE_vect)
{
    if (tx_posicion < tx_longitud) {
        UDR0 = tx_buffer[tx_posicion++];
    } else {
        UCSR0B &= ~(1 << UDRIE0);
        tx_longitud = 0;
        tx_posicion = 0;
    }
}

static uint8_t agregar_entero(char *mensaje,
                              uint8_t posicion,
                              uint16_t valor)
{
    char digitos[5];
    uint8_t cantidad = 0;

    do {
        digitos[cantidad++] = (char)('0' + valor % 10U);
        valor /= 10U;
    } while (valor > 0U);

    while (cantidad > 0U) {
        mensaje[posicion++] = digitos[--cantidad];
    }

    return posicion;
}

static uint8_t comenzar_envio(const char *mensaje, uint8_t cantidad)
{
    uint8_t disponible = 0;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (tx_longitud == 0U && cantidad <= TX_BUFFER_TAM) {
            for (uint8_t i = 0; i < cantidad; i++) {
                tx_buffer[i] = mensaje[i];
            }

            tx_posicion = 0;
            tx_longitud = cantidad;
            UCSR0B |= (1 << UDRIE0);
            disponible = 1;
        }
    }

    return disponible;
}

static void enviar_proximidad(uint32_t ahora)
{
    char mensaje[TX_BUFFER_TAM];
    uint8_t posicion = 0;

    if ((uint32_t)(ahora - ultimo_envio_ms) < PERIODO_ENVIO_MS) {
        return;
    }

    mensaje[posicion++] = 'P';
    mensaje[posicion++] = ',';

    if (distancia_cm == 0xFFFFU) {
        mensaje[posicion++] = '-';
        mensaje[posicion++] = '1';
    } else {
        posicion = agregar_entero(mensaje, posicion, distancia_cm);
    }

    mensaje[posicion++] = ',';
    mensaje[posicion++] = alerta_proximidad ? '1' : '0';
    mensaje[posicion++] = '\n';

    // Si el envio anterior sigue activo, se intenta en la proxima vuelta.
    if (comenzar_envio(mensaje, posicion)) {
        ultimo_envio_ms = ahora;
    }
}

// Lee un entero entre -100 y 100 y deja el cursor en el separador.
static uint8_t leer_porcentaje(const char **cursor, int16_t *resultado)
{
    const char *p = *cursor;
    int16_t valor = 0;
    uint8_t negativo = (*p == '-');

    if (*p == '-' || *p == '+') p++;
    if (*p < '0' || *p > '9') return 0;

    while (*p >= '0' && *p <= '9') {
        valor = valor * 10 + (*p - '0');

        if (valor > 100) return 0;

        p++;
    }

    *resultado = negativo ? -valor : valor;
    *cursor = p;

    return 1;
}

static void procesar_movimiento(void)
{
    int16_t marcha;
    int16_t giro;
    const char *p = trama;

    trama[longitud] = '\0';

    if (!leer_porcentaje(&p, &marcha)) return;
    if (*p != ',') return;

    p++;

    if (!leer_porcentaje(&p, &giro)) return;
    if (*p != '\0') return;

    mover_auto(marcha, giro);
    ultima_trama_ms = tiempo_ms();
    enlace_activo = 1;
}

static void procesar_boton(uint8_t dato)
{
    if (dato == 'U' || dato == 'u') {
        brazo_objetivo = BRAZO_ARRIBA_US;
    } else if (dato == 'B' || dato == 'b') {
        brazo_objetivo = BRAZO_ABAJO_US;
    } else if (dato == 'V' || dato == 'v') {
        brazo_objetivo = brazo_actual;
    } else if (dato == 'G' || dato == 'g') {
        garra_abierta = !garra_abierta;
        garra_objetivo = garra_abierta
                             ? GARRA_ABIERTA_US
                             : GARRA_CERRADA_US;
    } else {
        return;
    }

    // Cubre un boton pulsado antes de recibir la primera trama de marcha.
    if (!enlace_activo) {
        ultima_trama_ms = tiempo_ms();
        enlace_activo = 1;
    }
}

static void recibir_byte(uint8_t dato)
{
    if (dato == 'M') {
        estado_rx = RX_MOVIMIENTO;
        longitud = 0;
        return;
    }

    if (dato == '\n' || dato == '\r') {
        if (estado_rx == RX_MOVIMIENTO) {
            procesar_movimiento();
        }

        estado_rx = RX_LIBRE;
        return;
    }

    if (estado_rx == RX_LIBRE) {
        procesar_boton(dato);
        return;
    }

    if (estado_rx == RX_MOVIMIENTO) {
        uint8_t permitido =
            (dato >= '0' && dato <= '9') ||
            dato == ',' ||
            dato == '-' ||
            dato == '+';

        if (!permitido || longitud >= sizeof(trama) - 1) {
            estado_rx = RX_DESCARTAR;
        } else {
            trama[longitud++] = (char)dato;
        }
    }
}

// ---------------- PROGRAMA PRINCIPAL ----------------

int main(void)
{
    cli();

    iniciar_motores();
    iniciar_servos();
    iniciar_sensor();
    iniciar_bluetooth();
    iniciar_reloj();

    sei();

    while (1) {
        if (UCSR0A & (1 << RXC0)) {
            uint8_t estado = UCSR0A;
            uint8_t dato = UDR0;

            if (estado & ((1 << FE0) |
                          (1 << DOR0) |
                          (1 << UPE0))) {
                estado_rx = RX_DESCARTAR;
            } else {
                recibir_byte(dato);
            }
        }

        uint32_t ahora = tiempo_ms();

        // Si la app deja de enviar datos, el auto se detiene.
        if (enlace_activo &&
            (uint32_t)(ahora - ultima_trama_ms) >= TIEMPO_CORTE_MS) {
            mover_auto(0, 0);
            brazo_objetivo = brazo_actual;
            garra_objetivo = garra_actual;
            estado_rx = RX_LIBRE;
            enlace_activo = 0;
        }

        actualizar_servos(ahora);
        actualizar_sensor(ahora);
        enviar_proximidad(ahora);
    }
}
