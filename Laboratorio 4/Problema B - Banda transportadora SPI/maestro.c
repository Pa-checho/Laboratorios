/* PROBLEMA B - MAESTRO - AVR-GCC / Microchip Studio / ATmega328P.
 * Reloj previsto: 16 MHz (UNO/Nano). F_CPU NO configura el reloj fisico.
 * LCD paralelo HD44780, 4 bits. No usa librerias de Arduino.
 * Crear un proyecto independiente para este archivo.
 */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#if F_CPU != 16000000UL && F_CPU != 8000000UL
#error "Esta version admite reloj real de 8 o 16 MHz"
#endif
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdint.h>
#include <stdio.h>

/* Deben coincidir con los comandos de esclavo.c. */
#define CMD_START       1
#define CMD_STOP        2
#define CMD_VELOCIDAD   3
#define CMD_PISTON      4
#define CMD_EMERGENCIA  5

/* Valores elegidos para el proyecto, ajustables al montaje. */
#define TEMP_LIMITE       40     // Emergencia si T > 40 grados C.
#define TEMP_REARME       37     // Permite reconocer la emergencia si T <= 37.
#define LDR_OSCURO       400     // Pieza presente si ADC < 400.
#define LDR_CLARO        550     // Rearma deteccion si ADC > 550.
#define DHT_PERIODO_MS  2200UL
#define PISTON_CICLO_MS  800UL   // Separacion minima entre ordenes de piston.

typedef enum { PARADO, MARCHA, EMERGENCIA } Estado;
static Estado estado = PARADO;
static volatile uint32_t ticks_ms;
static uint8_t velocidad, temperatura, temp_valida, fallos_dht;
static uint8_t pieza_presente, piston_usado;
static uint16_t lectura_ldr;
static uint32_t piezas, ultima_temp, ultimo_piston;
static uint8_t evento_tx = CMD_STOP, valor_tx;
static uint8_t fallo_sensor;

ISR(TIMER0_COMPA_vect) { ticks_ms++; } // Base temporal: 1 ms.

static uint32_t millis(void)
{
    uint32_t copia;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { copia = ticks_ms; }
    return copia;
}

static void timers_init(void)
{
    TCCR0A = (1 << WGM01);                     // Timer0 en CTC.
    TCCR0B = (1 << CS01) | (1 << CS00);        // Prescaler 64.
    OCR0A = (uint8_t)(F_CPU / 64UL / 1000UL - 1UL);
    TIMSK0 = (1 << OCIE0A);
    TCCR1A = 0;                               // Timer1 mide pulsos DHT11.
    TCCR1B = (1 << CS11);                     // Prescaler 8, libre.
}

/* LCD: RS=PD4, E=PD5, D4=PD6, D5=PD7, D6=PB0, D7=PB1. */
static void lcd_nibble(uint8_t n)
{
    PORTD = (PORTD & ~((1 << PD6) | (1 << PD7))) | ((n & 3) << 6);
    PORTB = (PORTB & ~((1 << PB0) | (1 << PB1))) | ((n >> 2) & 3);
    PORTD |= (1 << PD5);
    _delay_us(1);
    PORTD &= ~(1 << PD5);
    _delay_us(50);
}

static void lcd_byte(uint8_t dato, uint8_t es_texto)
{
    if (es_texto) PORTD |= (1 << PD4);
    else PORTD &= ~(1 << PD4);
    lcd_nibble(dato >> 4);
    lcd_nibble(dato & 15);
}

static void lcd_init(void)
{
    DDRD |= (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);
    DDRB |= (1 << PB0) | (1 << PB1);
    _delay_ms(40);
    lcd_nibble(3); _delay_ms(5);
    lcd_nibble(3); _delay_us(150);
    lcd_nibble(3); lcd_nibble(2);              // Pasa a interfaz de 4 bits.
    lcd_byte(0x28, 0);                        // 2 filas, matriz 5x8.
    lcd_byte(0x0C, 0);                        // Pantalla ON, cursor OFF.
    lcd_byte(0x06, 0);                        // Cursor avanza al escribir.
    lcd_byte(0x01, 0); _delay_ms(2);
}

static void lcd_linea(uint8_t fila, const char *s)
{
    lcd_byte(fila ? 0xC0 : 0x80, 0);
    uint8_t fin = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (!fin && *s == '\0') fin = 1;
        lcd_byte(fin ? ' ' : (uint8_t)*s++, 1); // Rellena restos de texto.
    }
}

static uint16_t adc_leer(uint8_t canal)
{
    ADMUX = (1 << REFS0) | canal;             // Referencia AVCC.
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) { }
    return ADC;
}

static void spi_init(void)
{
    PORTB |= (1 << PB2);                     // SS inactivo antes de ser salida.
    DDRB |= (1 << PB2) | (1 << PB3) | (1 << PB5);
    DDRB &= ~(1 << PB4);                     // MISO es entrada.
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
    SPSR = 0;                               // Modo 0, reloj F_CPU/128.
}

static void spi_byte(uint8_t dato)
{
    SPDR = dato;
    while (!(SPSR & (1 << SPIF))) { }
    (void)SPDR;                              // Completa limpieza de SPIF.
}

static void enviar(uint8_t cmd, uint8_t valor, uint8_t mostrar)
{
    PORTB &= ~(1 << PB2);                    // Una seleccion por trama.
    _delay_us(50);
    spi_byte(cmd);
    _delay_us(50);                           // Tiempo para ISR del esclavo.
    spi_byte(valor);
    _delay_us(50);
    PORTB |= (1 << PB2);                     // Fin: esclavo aplica la trama.
    _delay_us(50);
    if (mostrar) { evento_tx = cmd; valor_tx = valor; }
}

/* Espera un nivel con limite. Timer1 evita bucles sin salida ante desconexion.
 * Las interrupciones siguen activas; solo existe la breve ISR de Timer0.
 */
static uint8_t dht_esperar(uint8_t nivel, uint16_t limite_us)
{
    uint16_t inicio = TCNT1;
    while (((PIND & (1 << PD2)) != 0) != nivel) {
        if ((uint16_t)(TCNT1 - inicio) >=
            limite_us * (F_CPU / 8000000UL)) return 0;
    }
    return 1;
}

static uint8_t dht_leer(uint8_t *temp)
{
    uint8_t datos[5] = {0, 0, 0, 0, 0};
    PORTD &= ~(1 << PD2);
    DDRD |= (1 << PD2);                      // Solicitud del maestro.
    _delay_ms(18);
    DDRD &= ~(1 << PD2);                     // Libera DATA; pull-up externo.
    PORTD |= (1 << PD2);
    if (!dht_esperar(0, 120) || !dht_esperar(1, 120) ||
        !dht_esperar(0, 120)) return 0;       // Respuesta inicial del sensor.
    for (uint8_t i = 0; i < 40; i++) {
        if (!dht_esperar(1, 100)) return 0;
        uint16_t inicio = TCNT1;
        if (!dht_esperar(0, 120)) return 0;
        uint16_t ancho = (uint16_t)(TCNT1 - inicio);
        datos[i / 8] <<= 1;
        if (ancho > 45UL * (F_CPU / 8000000UL)) datos[i / 8] |= 1;
    }
    if ((uint8_t)(datos[0] + datos[1] + datos[2] + datos[3]) != datos[4])
        return 0;                           // Rechaza checksum incorrecto.
    if (datos[2] > 50 || datos[3] > 9) return 0; // DHT11 clasico: 0..50 C.
    *temp = datos[2];                        // Redondea hacia arriba fraccion.
    if (datos[3]) (*temp)++;
    return 1;
}

/* Un evento por pulsacion; antirrebote de 30 ms. Boton PD3-GND. */
static uint8_t boton_pulsado(uint32_t ahora)
{
    static uint8_t anterior = 1, estable = 1;
    static uint32_t cambio;
    uint8_t actual = (PIND & (1 << PD3)) != 0;
    if (actual != anterior) { anterior = actual; cambio = ahora; }
    if (actual != estable && (uint32_t)(ahora - cambio) >= 30) {
        estable = actual;
        return !estable;
    }
    return 0;
}

static void detener_emergencia(uint8_t sensor)
{
    fallo_sensor = sensor;
    if (estado != EMERGENCIA) {
        estado = EMERGENCIA;
        enviar(CMD_EMERGENCIA, 0, 1);
    }
}

static void mostrar_lcd(uint32_t ahora)
{
    char linea[17];
    char e = estado == MARCHA ? 'M' : estado == PARADO ? 'P' : 'E';
    unsigned porcentaje = (uint16_t)velocidad * 100U / 255U;

    // Primera fila: construye los 16 caracteres directamente.
    // Evita una cadena temporal y el formato anidado de la temperatura.
    for (uint8_t i = 0; i < 16; i++) linea[i] = ' ';
    linea[16] = '\0';
    linea[0] = e;
    linea[2] = 'T';
    linea[3] = ':';
    if (temp_valida) {
        linea[4] = '0' + temperatura / 10;    // Decenas: usa la misma T del control.
        linea[5] = '0' + temperatura % 10;    // Unidades.
    } else {
        linea[4] = '-'; linea[5] = '-';       // Todavia no hay lectura valida.
    }
    linea[6] = 'C';
    linea[8] = 'V';
    linea[9] = ':';
    if (porcentaje == 100) linea[10] = '1';
    if (porcentaje >= 10) linea[11] = '0' + (porcentaje / 10) % 10;
    linea[12] = '0' + porcentaje % 10;
    linea[13] = '%';
    lcd_linea(0, linea);
    if ((ahora / 1200UL) % 2 == 0) {
        snprintf(linea, sizeof(linea), "P:%05lu L:%04u",
                 (unsigned long)piezas, (unsigned)lectura_ldr);
    } else {
        const char *nombre = "STOP";
        switch (evento_tx) {
            case CMD_START: nombre = "START"; break;
            case CMD_VELOCIDAD: nombre = "VEL"; break;
            case CMD_PISTON: nombre = "PISTON"; break;
            case CMD_EMERGENCIA: nombre = fallo_sensor ? "ERR DHT" : "EMERG"; break;
        }
        snprintf(linea, sizeof(linea), "TX:%s %3u", nombre, (unsigned)valor_tx);
    }
    lcd_linea(1, linea);
}

int main(void)
{
    DDRD &= ~((1 << PD2) | (1 << PD3));
    PORTD |= (1 << PD2) | (1 << PD3);         // Entradas con pull-up.
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    DIDR0 = (1 << ADC0D) | (1 << ADC1D);
    timers_init(); spi_init(); lcd_init();
    sei();
    enviar(CMD_STOP, 0, 1);                   // Al arrancar siempre parado.
    uint32_t t_adc = 0, t_dht = 0, t_tx = 0, t_lcd = 0;
    uint8_t velocidad_anterior = 0;
    for (;;) {
        uint32_t ahora = millis();
        if ((uint32_t)(ahora - t_dht) >= DHT_PERIODO_MS) {
            t_dht = ahora;                   // Tambien espera al encender DHT.
            uint8_t nueva;
            if (dht_leer(&nueva)) {
                temperatura = nueva; temp_valida = 1; fallos_dht = 0;
                ultima_temp = millis();
                if (temperatura > TEMP_LIMITE) detener_emergencia(0);
            } else {
                if (fallos_dht < 3) fallos_dht++;
                if (fallos_dht >= 3) {
                    temp_valida = 0;
                    detener_emergencia(1);
                }
            }
            ahora = millis();
        }
        if (temp_valida && (uint32_t)(ahora - ultima_temp) > 7000UL) {
            temp_valida = 0;
            detener_emergencia(1);
        }
        if ((uint32_t)(ahora - t_adc) >= 20) {
            t_adc = ahora;
            velocidad = (uint8_t)(adc_leer(0) >> 2); // ADC 0..1023 -> PWM 0..255.
            lectura_ldr = adc_leer(1);
            if (!pieza_presente && lectura_ldr < LDR_OSCURO) {
                pieza_presente = 1;          // Un solo evento por oscurecimiento.
                if (estado == MARCHA && (!piston_usado ||
                    (uint32_t)(ahora - ultimo_piston) >= PISTON_CICLO_MS)) {
                    if (piezas < 99999UL) piezas++;
                    ultimo_piston = ahora; piston_usado = 1;
                    enviar(CMD_PISTON, 0, 1);
                }
            } else if (lectura_ldr > LDR_CLARO) pieza_presente = 0;
        }
        if (boton_pulsado(ahora)) {
            switch (estado) {
                case PARADO:
                    if (temp_valida && !fallos_dht && temperatura <= TEMP_LIMITE) {
                        estado = MARCHA;
                        enviar(CMD_START, velocidad, 1);
                    }
                    break;
                case MARCHA:
                    estado = PARADO;
                    enviar(CMD_STOP, 0, 1);
                    break;
                case EMERGENCIA:
                    if (temp_valida && !fallos_dht && temperatura <= TEMP_REARME) {
                        estado = PARADO;      // Otro toque inicia la marcha.
                        enviar(CMD_STOP, 0, 1);
                    }
                    break;
            }
        }
        if ((uint32_t)(ahora - t_tx) >= 100) {
            t_tx = ahora;                   // Reenvio mantiene vivo el enlace.
            uint8_t cmd = estado == MARCHA ? CMD_VELOCIDAD :
                          estado == EMERGENCIA ? CMD_EMERGENCIA : CMD_STOP;
            enviar(cmd, estado == MARCHA ? velocidad : 0,
                   estado == MARCHA && velocidad != velocidad_anterior);
            velocidad_anterior = velocidad;
        }
        if ((uint32_t)(ahora - t_lcd) >= 200) {
            t_lcd = ahora;
            mostrar_lcd(ahora);
        }
    }
}
