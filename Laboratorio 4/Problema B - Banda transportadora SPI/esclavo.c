/* PROBLEMA B - ESCLAVO - AVR-GCC / Microchip Studio / ATmega328P.
 * Compilar como OTRO proyecto: cada micro tiene su propio main().
 * Timer0: 1 ms. Timer1: servo 50 Hz. Timer2: PWM del motor en OC2B/PD3.
 */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#if F_CPU != 16000000UL && F_CPU != 8000000UL
#error "Esta version admite reloj real de 8 o 16 MHz"
#endif
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#define CMD_START       1
#define CMD_STOP        2
#define CMD_VELOCIDAD   3
#define CMD_PISTON      4
#define CMD_EMERGENCIA  5
#define SERVO_REPOSO_US 1000UL                 // Ajustar a la mecanica.
#define SERVO_ACTIVO_US 1500UL                 // Recorrido inicial moderado.
#define SERVO_TICKS(us) ((us) * (F_CPU / 8000000UL))
#define TIEMPO_EXTENDIDO_MS 350
#define TIEMPO_RETORNO_MS   350
#define LIMITE_ENLACE_MS   1000

/* Todo el estado se modifica dentro de ISRs, sin interrupciones anidadas. */
static volatile uint8_t marcha, alarma;
static volatile uint8_t fase_servo;           // 0=reposo, 1=extendido, 2=retorno.
static volatile uint16_t tiempo_servo, sin_trama;
static volatile uint8_t rx_cantidad, rx_comando, rx_valor;

static void motor_pwm(uint8_t duty)
{
    if (duty == 0) {
        TCCR2A &= ~(1 << COM2B1);             // Desconecta OC2B: cero real.
        PORTD &= ~(1 << PD3);                // Evita pulso residual a OCR=0.
        OCR2B = 0;
    } else {
        OCR2B = duty;
        TCCR2A |= (1 << COM2B1);             // PWM no invertido en ENA.
    }
}

static void parar(uint8_t con_alarma)
{
    marcha = 0; alarma = con_alarma;
    motor_pwm(0);
    PORTD &= ~((1 << PD4) | (1 << PD5) | (1 << PD6));
    OCR1A = (uint16_t)SERVO_TICKS(SERVO_REPOSO_US);
    fase_servo = 0; tiempo_servo = 0;
    if (con_alarma) PORTD |= (1 << PD7) | (1 << PD2);
    else PORTD &= ~((1 << PD7) | (1 << PD2));
}

static void ejecutar(uint8_t cmd, uint8_t valor)
{
    // Solo comandos reconocidos reinician el tiempo de supervision.
    if (cmd < CMD_START || cmd > CMD_EMERGENCIA) return;
    sin_trama = 0;
    switch (cmd) {
        case CMD_START:
            if (!alarma) {                  // STOP debe reconocer una falla.
                marcha = 1;
                PORTD |= (1 << PD4) | (1 << PD6); // IN1=1, LED verde=1.
                PORTD &= ~(1 << PD5);       // IN2=0: un solo sentido.
                motor_pwm(valor);
            }
            break;
        case CMD_STOP:
            parar(0);
            break;
        case CMD_VELOCIDAD:
            if (marcha && !alarma) motor_pwm(valor);
            break;
        case CMD_PISTON:
            if (marcha && !alarma && fase_servo == 0) {
                OCR1A = (uint16_t)SERVO_TICKS(SERVO_ACTIVO_US);
                fase_servo = 1; tiempo_servo = 0;
            }
            break;
        case CMD_EMERGENCIA:
            parar(1);
            break;
    }
}

ISR(SPI_STC_vect)
{
    uint8_t dato = SPDR;
    if (!(PINB & (1 << PB2))) {               // Acepta bytes solo seleccionado.
        if (rx_cantidad == 0) rx_comando = dato;
        else if (rx_cantidad == 1) rx_valor = dato;
        if (rx_cantidad < 3) rx_cantidad++;   // 3 significa trama demasiado larga.
    }
    SPDR = 0;                               // Esta version no implementa ACK.
}

ISR(PCINT0_vect)
{
    // SS delimita cada trama. No conserva medias tramas entre selecciones.
    if (PINB & (1 << PB2)) {
        if (rx_cantidad == 2) ejecutar(rx_comando, rx_valor);
    }
    rx_cantidad = 0;
}

ISR(TIMER0_COMPA_vect)
{
    if (sin_trama < LIMITE_ENLACE_MS) {
        sin_trama++;
        if (sin_trama == LIMITE_ENLACE_MS) parar(1);
    }
    if (fase_servo) {
        tiempo_servo++;
        if (fase_servo == 1 && tiempo_servo >= TIEMPO_EXTENDIDO_MS) {
            OCR1A = (uint16_t)SERVO_TICKS(SERVO_REPOSO_US);
            fase_servo = 2; tiempo_servo = 0;
        } else if (fase_servo == 2 && tiempo_servo >= TIEMPO_RETORNO_MS) {
            fase_servo = 0;
        }
    }
}

int main(void)
{
    // PD2=buzzer, PD3=ENA/PWM, PD4=IN1, PD5=IN2, PD6=verde, PD7=rojo.
    PORTD = 0;
    DDRD |= (1 << PD2) | (1 << PD3) | (1 << PD4) |
            (1 << PD5) | (1 << PD6) | (1 << PD7);
    DDRB |= (1 << PB1) | (1 << PB4);          // Servo OC1A y MISO.
    DDRB &= ~((1 << PB2) | (1 << PB3) | (1 << PB5));
    PORTB |= (1 << PB2);                     // SS alto sin maestro.

    // Timer1: Fast PWM modo 14, TOP=ICR1, salida OC1A/PB1.
    TCCR1A = 0;
    TCCR1B = 0;
    ICR1 = (uint16_t)(F_CPU / 8UL / 50UL - 1UL);
    OCR1A = (uint16_t)SERVO_TICKS(SERVO_REPOSO_US);
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

    // Timer2: Fast PWM de 8 bits, prescaler 64; 976.56 Hz a 16 MHz.
    TCCR2A = (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS22);
    parar(0);

    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = (uint8_t)(F_CPU / 64UL / 1000UL - 1UL);
    TIMSK0 = (1 << OCIE0A);

    SPCR = (1 << SPE) | (1 << SPIE);          // Esclavo SPI, modo 0, MSB primero.
    SPDR = 0;
    PCMSK0 = (1 << PCINT2);                  // Detecta cambios de SS/PB2.
    PCIFR = (1 << PCIF0);
    PCICR = (1 << PCIE0);
    sei();
    for (;;) { }                            // ISRs reciben y ejecutan las ordenes.
}
