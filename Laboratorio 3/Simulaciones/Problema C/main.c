/*
 * Clasificador de colores para Microchip Studio / AVR-GCC.
 * Destino asumido: ATmega328P a 16 MHz.
 *
 * Pines:
 *   LDR       = A0 / PC0
 *   NeoPixel  = D6 / PD6
 *   Servo     = D9 / PB1 / OC1A
 *   Boton     = D2 / PD2 (pull-up externo)
 *   LCD RGB   = bus TWI/I2C
 *
 * LCD asumido: DFRobot RGB LCD 16x2, direcciones 0x3E y 0x62.
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>

#define N_PIXELES 64
#define PIN_WS PD6
#define PIN_BOTON PD2
#define BRILLO 200
#define N_COLORES 4
#define SIN_MUESTRA 4
#define N_MUESTRAS 24
#define TOLERANCIA 50
#define SERVO_MIN_US 544
#define SERVO_MAX_US 2400
#define FIRMA 0xC031
#define VERSION 4

static const uint8_t angulos[N_COLORES] = {0, 65, 115, 180};

static const uint8_t colores[N_COLORES][3] = {
    {BRILLO, 0, 0},
    {0, BRILLO, 0},
    {0, 0, BRILLO},
    {BRILLO, BRILLO, 0}
};

static const char *nombres[5] = {
    "ROJO",
    "VERDE",
    "AZUL",
    "AMARILLO",
    "SIN MUESTRA"
};

typedef struct {
    uint16_t firma;
    uint8_t version;
    uint8_t brillo;
    uint16_t referencia[N_COLORES][3];
    uint16_t crc;
} Calibracion;

static Calibracion calibracion;
static Calibracion candidata;
static Calibracion EEMEM eeprom_calibracion;

static bool lcd_disponible;
static bool calibrado;
static bool comando_largo;
static bool boton_atendido;

static uint8_t modo;
static uint8_t paso_cal;
static uint8_t resultado = SIN_MUESTRA;
static uint8_t mas_cercano;

static int angulo_actual = 90;
static uint16_t lectura[3];
static char comando[64];
static char linea_lcd[17];
static uint8_t longitud;

/* UART0 a 9600 baudios, formato 8N1. */
static void uart_init(void) {
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0A = 0;
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
}

static void uart_putc(char caracter) {
    while (!(UCSR0A & _BV(UDRE0))) {
    }

    UDR0 = caracter;
}

static void uart_print(const char *texto) {
    while (*texto) {
        uart_putc(*texto++);
    }
}

static void uart_num(int32_t numero) {
    char digitos[12];
    uint8_t cantidad = 0;
    uint32_t valor;

    if (numero < 0) {
        uart_putc('-');
        valor = 0u - (uint32_t)numero;
    } else {
        valor = (uint32_t)numero;
    }

    do {
        digitos[cantidad++] = (char)('0' + (valor % 10));
        valor /= 10;
    } while (valor != 0);

    while (cantidad > 0) {
        uart_putc(digitos[--cantidad]);
    }
}

static void uart_line(const char *texto) {
    uart_print(texto);
    uart_print("\r\n");
}

/* ADC con referencia AVcc y entrada ADC0. */
static void adc_init(void) {
    ADMUX = _BV(REFS0);
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    DIDR0 = _BV(ADC0D);
}

static uint16_t adc_read(void) {
    ADMUX = _BV(REFS0);
    ADCSRA |= _BV(ADSC);

    while (ADCSRA & _BV(ADSC)) {
    }

    return ADC;
}

/* Bus TWI a 100 kHz. */
static void twi_init(void) {
    TWSR = 0;
    TWBR = 72;
    TWCR = _BV(TWEN);
}

static bool twi_start(uint8_t direccion) {
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);

    while (!(TWCR & _BV(TWINT))) {
    }

    uint8_t estado = TWSR & 0xF8;
    if (estado != TW_START && estado != TW_REP_START) {
        return false;
    }

    TWDR = direccion;
    TWCR = _BV(TWINT) | _BV(TWEN);

    while (!(TWCR & _BV(TWINT))) {
    }

    return (TWSR & 0xF8) == TW_MT_SLA_ACK;
}

static bool twi_write(uint8_t dato) {
    TWDR = dato;
    TWCR = _BV(TWINT) | _BV(TWEN);

    while (!(TWCR & _BV(TWINT))) {
    }

    return (TWSR & 0xF8) == TW_MT_DATA_ACK;
}

static void twi_stop(void) {
    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
}

static bool i2c_write(uint8_t direccion, const uint8_t *datos, uint8_t cantidad) {
    if (!twi_start(direccion << 1)) {
        twi_stop();
        return false;
    }

    while (cantidad > 0) {
        if (!twi_write(*datos++)) {
            twi_stop();
            return false;
        }
        cantidad--;
    }

    twi_stop();
    return true;
}

static void lcd_command(uint8_t comando_lcd) {
    uint8_t datos[2] = {0x80, comando_lcd};
    (void)i2c_write(0x3E, datos, 2);
    _delay_ms(2);
}

static void lcd_data(uint8_t dato) {
    uint8_t datos[2] = {0x40, dato};
    (void)i2c_write(0x3E, datos, 2);
}

static void lcd_rgb(uint8_t rojo, uint8_t verde, uint8_t azul);

static void lcd_init(void) {
    _delay_ms(50);
    lcd_command(0x38);
    lcd_command(0x39);
    lcd_command(0x14);
    lcd_command(0x70);
    lcd_command(0x56);
    lcd_command(0x6C);
    _delay_ms(200);
    lcd_command(0x38);
    lcd_command(0x0C);
    lcd_command(0x01);
    _delay_ms(2);

    uint8_t rgb[2] = {0x00, 0x00};
    (void)i2c_write(0x62, rgb, 2);

    rgb[0] = 0x01;
    rgb[1] = 0x05;
    (void)i2c_write(0x62, rgb, 2);

    rgb[0] = 0x08;
    rgb[1] = 0xAA;
    (void)i2c_write(0x62, rgb, 2);

    lcd_rgb(255, 0, 0);
    lcd_disponible = true;
}

static void lcd_rgb(uint8_t rojo, uint8_t verde, uint8_t azul) {
    uint8_t dato[2] = {0x04, rojo};
    (void)i2c_write(0x62, dato, 2);

    dato[0] = 0x03;
    dato[1] = verde;
    (void)i2c_write(0x62, dato, 2);

    dato[0] = 0x02;
    dato[1] = azul;
    (void)i2c_write(0x62, dato, 2);
}

static void lcd_print(const char *texto) {
    while (*texto) {
        lcd_data((uint8_t)*texto++);
    }
}

static void pantalla(const char *linea1, const char *linea2) {
    if (!lcd_disponible) {
        return;
    }

    lcd_command(0x01);
    _delay_ms(2);
    lcd_command(0x80);
    lcd_print(linea1);
    lcd_command(0xC0);
    lcd_print(linea2);
}

/* WS2812B en PD6, con pulsos medidos para un reloj de 16 MHz. */
static void ws_send_byte(uint8_t valor, uint8_t nivel_alto, uint8_t nivel_bajo) {
    uint8_t bits = 8;

    /*
     * Cada bit dura 20 ciclos (1,25 us). El cero mantiene la señal alta
     * 6 ciclos; el uno, 12 ciclos. Las interrupciones deben estar desactivadas.
     */
    asm volatile (
        "1:                         \n\t"
        "out %[puerto], %[alto]     \n\t"
        "lsl %[dato]                \n\t"
        "brcs 2f                    \n\t"
        "nop                        \n\t"
        "nop                        \n\t"
        "out %[puerto], %[bajo]     \n\t"
        "rjmp .+0                   \n\t"
        "rjmp .+0                   \n\t"
        "rjmp 3f                    \n\t"
        "2:                         \n\t"
        "rjmp .+0                   \n\t"
        "rjmp .+0                   \n\t"
        "rjmp .+0                   \n\t"
        "nop                        \n\t"
        "out %[puerto], %[bajo]     \n\t"
        "3:                         \n\t"
        "nop                        \n\t"
        "nop                        \n\t"
        "nop                        \n\t"
        "nop                        \n\t"
        "nop                        \n\t"
        "dec %[bits]                \n\t"
        "brne 1b                    \n\t"
        : [dato] "+&r" (valor), [bits] "+&r" (bits)
        : [puerto] "I" (_SFR_IO_ADDR(PORTD)),
          [alto] "r" (nivel_alto),
          [bajo] "r" (nivel_bajo)
        : "cc"
    );
}

static void ws_color(uint8_t rojo, uint8_t verde, uint8_t azul) {
    uint8_t estado_registro = SREG;
    uint8_t nivel_alto = PORTD | _BV(PIN_WS);
    uint8_t nivel_bajo = PORTD & (uint8_t)~_BV(PIN_WS);
    cli();

    for (uint8_t pixel = 0; pixel < N_PIXELES; pixel++) {
        if (pixel == 0) {
            ws_send_byte(verde, nivel_alto, nivel_bajo);
            ws_send_byte(rojo, nivel_alto, nivel_bajo);
            ws_send_byte(azul, nivel_alto, nivel_bajo);
        } else {
            ws_send_byte(0, nivel_alto, nivel_bajo);
            ws_send_byte(0, nivel_alto, nivel_bajo);
            ws_send_byte(0, nivel_alto, nivel_bajo);
        }
    }

    SREG = estado_registro;
    _delay_us(80);
}

static void ws_all(uint8_t rojo, uint8_t verde, uint8_t azul) {
    uint8_t estado_registro = SREG;
    uint8_t nivel_alto = PORTD | _BV(PIN_WS);
    uint8_t nivel_bajo = PORTD & (uint8_t)~_BV(PIN_WS);
    cli();

    for (uint8_t pixel = 0; pixel < N_PIXELES; pixel++) {
        ws_send_byte(verde, nivel_alto, nivel_bajo);
        ws_send_byte(rojo, nivel_alto, nivel_bajo);
        ws_send_byte(azul, nivel_alto, nivel_bajo);
    }

    SREG = estado_registro;
    _delay_us(80);
}

/* Servo en OC1A/PB1; Timer1 genera PWM de 50 Hz. */
static void servo_init(void) {
    DDRB |= _BV(PB1);
    TCCR1A = _BV(COM1A1) | _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
    ICR1 = 39999;
}

static void mover_servo(int angulo) {
    if (angulo < 0) {
        angulo = 0;
    }
    if (angulo > 180) {
        angulo = 180;
    }

    uint16_t pulso_us = (uint16_t)(SERVO_MIN_US +
        ((uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * angulo) / 180);

    OCR1A = (uint16_t)(pulso_us * 2);
    angulo_actual = angulo;
    _delay_ms(350);
}

static bool boton_presionado(void) {
    if (PIND & _BV(PIN_BOTON)) {
        boton_atendido = false;
        return false;
    }

    if (!boton_atendido) {
        boton_atendido = true;
        return true;
    }

    return false;
}

static const char *nombre(uint8_t codigo) {
    if (codigo < 5) {
        return nombres[codigo];
    }
    return nombres[4];
}

static uint16_t calcular_crc(const Calibracion *datos) {
    const uint8_t *bytes = (const uint8_t *)datos;
    uint16_t crc = 0xFFFF;

    for (size_t indice = 0; indice < offsetof(Calibracion, crc); indice++) {
        crc ^= bytes[indice];

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static void medir_rgb(uint16_t destino[3]) {
    const uint8_t luces[3][3] = {
        {BRILLO, 0, 0},
        {0, BRILLO, 0},
        {0, 0, BRILLO}
    };

    for (uint8_t canal = 0; canal < 3; canal++) {
        ws_color(luces[canal][0], luces[canal][1], luces[canal][2]);
        _delay_ms(250);

        uint32_t suma = 0;
        for (uint8_t muestra = 0; muestra < N_MUESTRAS; muestra++) {
            suma += adc_read();
            _delay_ms(10);
        }

        destino[canal] = (uint16_t)(suma / N_MUESTRAS);
    }

    ws_all(0, 0, 0);
}

static uint8_t clasificar(const uint16_t valores[3], uint8_t *cercano) {
    uint32_t mejor_distancia = 0xFFFFFFFFUL;
    uint8_t mejor_patron = 0;

    for (uint8_t patron = 0; patron < N_COLORES; patron++) {
        uint32_t distancia = 0;

        for (uint8_t canal = 0; canal < 3; canal++) {
            int32_t diferencia = (int32_t)valores[canal] -
                (int32_t)calibracion.referencia[patron][canal];
            distancia += (uint32_t)(diferencia * diferencia);
        }

        if (distancia < mejor_distancia) {
            mejor_distancia = distancia;
            mejor_patron = patron;
        }
    }

    *cercano = mejor_patron;

    for (uint8_t canal = 0; canal < 3; canal++) {
        int32_t diferencia = (int32_t)valores[canal] -
            (int32_t)calibracion.referencia[mejor_patron][canal];

        if (diferencia < 0) {
            diferencia = -diferencia;
        }

        if (diferencia >= TOLERANCIA) {
            return SIN_MUESTRA;
        }
    }

    return mejor_patron;
}

static void mostrar_menu(void) {
    modo = 0;
    ws_all(0, 0, 0);
    uart_line("");
    uart_line("=== CLASIFICADOR DE COLORES ===");
    uart_line("Escriba:");
    uart_line("  calibrar       para calibrar los 4 colores");
    uart_line("  detectar color para esperar el boton");
    uart_line("  referencias    para ver la calibracion");
    uart_line("  menu           para volver al menu");
    pantalla("Clasificador", "Ver menu Serial");
}

static void pedir_muestra(void) {
    if (paso_cal >= N_COLORES) {
        return;
    }

    uart_print("Coloque una muestra ");
    uart_print(nombre(paso_cal));
    uart_line(" y escriba listo.");
    pantalla("Calibrar:", nombre(paso_cal));
}

static void tomar_patron(void) {
    if (paso_cal >= N_COLORES) {
        return;
    }

    uart_print("Midiendo patron ");
    uart_line(nombre(paso_cal));
    pantalla("Midiendo patron", nombre(paso_cal));
    medir_rgb(lectura);

    for (uint8_t canal = 0; canal < 3; canal++) {
        candidata.referencia[paso_cal][canal] = lectura[canal];
    }

    paso_cal++;
    if (paso_cal < N_COLORES) {
        pedir_muestra();
        return;
    }

    candidata.firma = FIRMA;
    candidata.version = VERSION;
    candidata.brillo = BRILLO;
    candidata.crc = calcular_crc(&candidata);

    calibracion = candidata;
    eeprom_update_block(
        &calibracion,
        &eeprom_calibracion,
        sizeof(calibracion)
    );
    calibrado = true;
    modo = 0;

    uart_line("Calibracion completa y guardada en EEPROM.");
    pantalla("Calibracion", "completa");
}

static void imprimir_referencias(void) {
    if (!calibrado) {
        uart_line("Todavia no hay una calibracion valida.");
        return;
    }

    uart_line("=== REFERENCIAS RGB ===");

    for (uint8_t patron = 0; patron < N_COLORES; patron++) {
        uart_print(nombre(patron));
        uart_print(": R=");
        uart_num(calibracion.referencia[patron][0]);
        uart_print(" G=");
        uart_num(calibracion.referencia[patron][1]);
        uart_print(" B=");
        uart_num(calibracion.referencia[patron][2]);
        uart_line("");
    }
}

static void mostrar_resultado(void) {
    snprintf(linea_lcd, sizeof(linea_lcd), "Angulo: %d", angulo_actual);
    pantalla(nombre(resultado), linea_lcd);
}

static void detectar_color(void) {
    pantalla("MIDIENDO RGB...", "Mantenga muestra");
    uart_line("Midiendo muestra...");

    medir_rgb(lectura);
    resultado = clasificar(lectura, &mas_cercano);

    if (resultado < N_COLORES) {
        mover_servo(angulos[resultado]);
        ws_all(
            colores[resultado][0],
            colores[resultado][1],
            colores[resultado][2]
        );
    } else {
        mover_servo(90);
        ws_all(0, 0, 0);
    }

    uart_print("Resultado: ");
    uart_line(nombre(resultado));
    uart_print("Angulo aplicado: ");
    uart_num(angulo_actual);
    uart_line("");

    if (resultado == SIN_MUESTRA) {
        uart_print("Color calibrado mas cercano: ");
    } else {
        uart_print("Patron usado: ");
    }
    uart_line(nombre(mas_cercano));

    for (uint8_t canal = 0; canal < 3; canal++) {
        uart_putc("RGB"[canal]);
        uart_print(" | Lectura=");
        uart_num(lectura[canal]);
        uart_print(" | Patron=");
        uart_num(calibracion.referencia[mas_cercano][canal]);
        uart_print(" | Diferencia=");
        uart_num(
            (int)lectura[canal] -
            (int)calibracion.referencia[mas_cercano][canal]
        );
        uart_line("");
    }

    if (resultado == SIN_MUESTRA) {
        uart_line("Fuera del umbral: servo centrado a 90 grados.");
    }

    mostrar_resultado();
    uart_line("Presione el boton para detectar otro color.");
}

static void procesar_comando(char *texto) {
    while (*texto == ' ') {
        texto++;
    }

    size_t cantidad = strlen(texto);
    while (cantidad > 0 && texto[cantidad - 1] == ' ') {
        texto[--cantidad] = '\0';
    }

    for (size_t i = 0; i < cantidad; i++) {
        texto[i] = (char)tolower((unsigned char)texto[i]);
    }

    if (cantidad == 0) {
        return;
    }

    if (strcmp(texto, "menu") == 0) {
        mostrar_menu();
    } else if (strcmp(texto, "referencias") == 0) {
        imprimir_referencias();
    } else if (strcmp(texto, "calibrar") == 0 || strcmp(texto, "1") == 0) {
        modo = 1;
        paso_cal = 0;
        memset(&candidata, 0, sizeof(candidata));
        ws_all(0, 0, 0);
        pedir_muestra();
    } else if (strcmp(texto, "detectar color") == 0 || strcmp(texto, "2") == 0) {
        if (!calibrado) {
            uart_line("Primero complete la calibracion.");
            return;
        }

        modo = 2;
        boton_atendido = false;
        uart_line("Deteccion activa. Presione el boton para leer.");
        pantalla("Listo para medir", "Presione boton");
    } else if (modo == 1 && strcmp(texto, "listo") == 0) {
        tomar_patron();
    } else {
        uart_line("Comando no valido. Use calibrar, detectar color, referencias o menu.");
    }
}

static void leer_comandos(void) {
    while (UCSR0A & _BV(RXC0)) {
        char caracter = (char)UDR0;

        if (caracter == '\n' || caracter == '\r') {
            if (comando_largo) {
                uart_line("Comando demasiado largo. Escribalo de nuevo.");
            } else if (longitud > 0) {
                comando[longitud] = '\0';
                procesar_comando(comando);
            }

            longitud = 0;
            comando_largo = false;
        } else if (caracter == 8 || caracter == 127) {
            if (longitud > 0) {
                longitud--;
            }
        } else if (longitud < sizeof(comando) - 1) {
            comando[longitud++] = caracter;
        } else {
            comando_largo = true;
        }
    }
}

int main(void) {
    uart_init();
    adc_init();
    twi_init();

    DDRD |= _BV(PIN_WS);
    PORTD &= (uint8_t)~_BV(PIN_WS);
    DDRD &= (uint8_t)~_BV(PIN_BOTON);

    servo_init();
    mover_servo(90);
    lcd_init();

    eeprom_read_block(
        &calibracion,
        &eeprom_calibracion,
        sizeof(calibracion)
    );

    calibrado = calibracion.firma == FIRMA &&
                calibracion.version == VERSION &&
                calibracion.brillo == BRILLO &&
                calibracion.crc == calcular_crc(&calibracion);

    mostrar_menu();

    for (;;) {
        leer_comandos();

        if (modo == 2 && boton_presionado()) {
            detectar_color();
        }
    }
}
