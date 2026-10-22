#define F_CPU 16000000UL

#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <string.h>
#include <util/delay.h>

/*
 * Laboratorio 4 - Problema A
 * Cajero automatico con ATmega328P, RC522 y tarjetas MIFARE.
 *
 * El programa utiliza polling: no se habilitan interrupciones del
 * ATmega328P. Los datos persistentes de cada cuenta se guardan en EEPROM.
 */

#define UART(texto) uart_flash(PSTR(texto))
#define LCD(fila, texto) lcd_linea_flash((fila), PSTR(texto))
#define LCD_VALOR(fila, titulo, valor) \
    lcd_valor_flash((fila), PSTR(titulo), (valor))

/* ==================== UART ==================== */

#define BAUDIOS 9600UL
#define UBRR_VALOR ((F_CPU / (16UL * BAUDIOS)) - 1UL)

void uart_iniciar(void)
{
    /* UART a 9600 baudios, formato 8N1 y solo transmision.
       No se habilita RX ni la interrupcion de transmision. */
    UBRR0H = (uint8_t)(UBRR_VALOR >> 8);
    UBRR0L = (uint8_t)UBRR_VALOR;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_caracter(char dato)
{
    /* Polling: espera que el registro de transmision quede libre.
       UDRE0 es un flag consultado por software, no una interrupcion. */
    while (!(UCSR0A & (1 << UDRE0))) {
    }
    UDR0 = dato;
}

void uart_texto(const char *texto)
{
    /* Envia una cadena almacenada en SRAM hasta encontrar '\0'. */
    while (*texto != '\0') {
        uart_caracter(*texto++);
    }
}

void uart_flash(PGM_P texto)
{
    /* Envia un texto guardado en Flash mediante PSTR/PROGMEM. */
    char dato;

    while ((dato = (char)pgm_read_byte(texto++)) != '\0') {
        uart_caracter(dato);
    }
}

void uart_hex(uint8_t valor)
{
    /* Convierte un byte en dos caracteres hexadecimales para diagnostico. */
    static const char digitos[] PROGMEM = "0123456789ABCDEF";

    uart_caracter((char)pgm_read_byte(&digitos[valor >> 4]));
    uart_caracter((char)pgm_read_byte(&digitos[valor & 0x0FU]));
}

void uart_numero(uint32_t numero)
{
    /* Convierte un entero decimal a texto sin usar printf. */
    char texto[11];
    uint8_t posicion = sizeof(texto) - 1U;

    texto[posicion] = '\0';
    do {
        texto[--posicion] = (char)('0' + numero % 10U);
        numero /= 10U;
    } while (numero != 0);
    uart_texto(&texto[posicion]);
}

/* ==================== SPI ==================== */

void spi_iniciar(void)
{
    /* D10 debe ser salida para conservar al ATmega328P como maestro SPI. */
    DDRB |= (1 << PB2) | (1 << PB3) | (1 << PB5);
    DDRB &= (uint8_t)~(1 << PB4);
    PORTB |= (1 << PB2);

    /* Modo 0, MSB primero y reloj lento de 125 kHz para diagnostico.
       No se habilita SPIE: la transferencia se atiende por polling. */
    SPCR = (1 << SPE) | (1 << MSTR) |
           (1 << SPR1) | (1 << SPR0);
    SPSR = 0;
}

uint8_t spi_transferir(uint8_t dato)
{
    /* SPI es full duplex: al escribir un byte tambien se recibe uno.
       Se consulta SPIF hasta que termina; no existe una ISR de SPI. */
    SPDR = dato;
    while (!(SPSR & (1 << SPIF))) {
    }
    return SPDR;
}

/* ==================== LECTOR RC522 ==================== */

#define RC522_RST PB1
#define RC522_SS  PB2

#define COMMAND_REG     0x01U
#define COM_IRQ_REG     0x04U
#define DIV_IRQ_REG     0x05U
#define ERROR_REG       0x06U
#define FIFO_DATA_REG   0x09U
#define FIFO_LEVEL_REG  0x0AU
#define BIT_FRAMING_REG 0x0DU
#define COLL_REG        0x0EU
#define MODE_REG        0x11U
#define TX_CONTROL_REG  0x14U
#define TX_ASK_REG      0x15U
#define CRC_RESULT_H    0x21U
#define CRC_RESULT_L    0x22U
#define T_MODE_REG      0x2AU
#define T_PRESCALER_REG 0x2BU
#define T_RELOAD_H_REG  0x2CU
#define T_RELOAD_L_REG  0x2DU
#define VERSION_REG     0x37U

#define CMD_IDLE       0x00U
#define CMD_CALC_CRC   0x03U
#define CMD_TRANSCEIVE 0x0CU
#define CMD_SOFT_RESET 0x0FU

#define PICC_REQA     0x26U
#define PICC_CL1      0x93U
#define PICC_HALT     0x50U

static void escribir(uint8_t registro, uint8_t valor)
{
    /* Baja SS, envia direccion de escritura y dato, y vuelve a subir SS. */
    PORTB &= (uint8_t)~(1 << RC522_SS);
    spi_transferir((uint8_t)((registro << 1) & 0x7EU));
    spi_transferir(valor);
    PORTB |= (1 << RC522_SS);
}

static uint8_t leer(uint8_t registro)
{
    /* Lee un registro interno del MFRC522 mediante SPI. */
    uint8_t valor;

    PORTB &= (uint8_t)~(1 << RC522_SS);
    spi_transferir((uint8_t)(((registro << 1) & 0x7EU) | 0x80U));
    valor = spi_transferir(0);
    PORTB |= (1 << RC522_SS);
    return valor;
}

static void activar_bits(uint8_t registro, uint8_t mascara)
{
    /* Activa solo los bits indicados sin modificar los restantes. */
    escribir(registro, (uint8_t)(leer(registro) | mascara));
}

static void desactivar_bits(uint8_t registro, uint8_t mascara)
{
    /* Desactiva solo los bits indicados sin modificar los restantes. */
    escribir(registro, (uint8_t)(leer(registro) & (uint8_t)~mascara));
}

static uint8_t calcular_crc(const uint8_t *datos, uint8_t cantidad,
                            uint8_t resultado[2])
{
    /* Usa el coprocesador CRC interno del RC522. Se consulta DivIrqReg
       mediante polling hasta obtener el resultado o agotar el limite. */
    uint16_t espera = 5000;
    uint8_t i;

    escribir(COMMAND_REG, CMD_IDLE);
    escribir(DIV_IRQ_REG, 0x04U);
    activar_bits(FIFO_LEVEL_REG, 0x80U);

    for (i = 0; i < cantidad; i++) {
        escribir(FIFO_DATA_REG, datos[i]);
    }

    escribir(COMMAND_REG, CMD_CALC_CRC);
    while (espera-- > 0) {
        if (leer(DIV_IRQ_REG) & 0x04U) {
            resultado[0] = leer(CRC_RESULT_L);
            resultado[1] = leer(CRC_RESULT_H);
            escribir(COMMAND_REG, CMD_IDLE);
            return 1;
        }
    }
    escribir(COMMAND_REG, CMD_IDLE);
    return 0;
}

static uint8_t transmitir(const uint8_t *envio, uint8_t largo_envio,
                          uint8_t *respuesta, uint8_t *largo_respuesta)
{
    uint16_t espera = 3000;
    /* Esta variable contiene ComIrqReg del RC522. Su nombre no significa
       que el ATmega atienda una interrupcion: el registro se consulta. */
    uint8_t interrupcion;
    uint8_t capacidad = largo_respuesta != 0 ? *largo_respuesta : 0;
    uint8_t cantidad;
    uint8_t i;

    escribir(COMMAND_REG, CMD_IDLE);
    escribir(COM_IRQ_REG, 0x7FU);
    activar_bits(FIFO_LEVEL_REG, 0x80U);
    for (i = 0; i < largo_envio; i++) {
        escribir(FIFO_DATA_REG, envio[i]);
    }

    escribir(COMMAND_REG, CMD_TRANSCEIVE);
    activar_bits(BIT_FRAMING_REG, 0x80U);
    do {
        interrupcion = leer(COM_IRQ_REG);
        if (interrupcion & 0x30U) {
            break;
        }
        if (interrupcion & 0x01U) {
            desactivar_bits(BIT_FRAMING_REG, 0x80U);
            return 0;
        }
    } while (--espera > 0);

    desactivar_bits(BIT_FRAMING_REG, 0x80U);
    if (espera == 0 || (leer(ERROR_REG) & 0x13U)) {
        return 0;
    }

    if (respuesta != 0 && largo_respuesta != 0) {
        cantidad = leer(FIFO_LEVEL_REG);
        if (cantidad > capacidad) {
            cantidad = capacidad;
        }
        for (i = 0; i < cantidad; i++) {
            respuesta[i] = leer(FIFO_DATA_REG);
        }
        *largo_respuesta = cantidad;
    }
    return 1;
}

static uint8_t solicitar(void)
{
    /* Envia REQA para detectar una tarjeta ISO/IEC 14443A cercana. */
    uint8_t comando = PICC_REQA;
    uint8_t respuesta[2];
    uint8_t largo = sizeof(respuesta);
    uint8_t correcto;

    /* REQA utiliza una trama corta de siete bits. */
    escribir(BIT_FRAMING_REG, 0x07U);
    correcto = transmitir(&comando, 1, respuesta, &largo);
    escribir(BIT_FRAMING_REG, 0);
    return correcto && largo == 2;
}

static uint8_t anticolision(uint8_t uid_bcc[5])
{
    /* Obtiene UID de cuatro bytes y verifica el byte de control BCC. */
    uint8_t comando[2] = {PICC_CL1, 0x20U};
    uint8_t largo = 5;

    escribir(BIT_FRAMING_REG, 0);
    desactivar_bits(COLL_REG, 0x80U);
    if (!transmitir(comando, 2, uid_bcc, &largo) || largo != 5) {
        return 0;
    }
    return (uint8_t)(uid_bcc[0] ^ uid_bcc[1] ^ uid_bcc[2] ^ uid_bcc[3])
           == uid_bcc[4];
}

static uint8_t seleccionar(const uint8_t uid_bcc[5])
{
    /* Selecciona la tarjeta detectada y comprueba la respuesta del lector. */
    uint8_t trama[9];
    uint8_t respuesta[3];
    uint8_t largo = sizeof(respuesta);
    uint8_t i;

    trama[0] = PICC_CL1;
    trama[1] = 0x70U;
    for (i = 0; i < 5; i++) {
        trama[2U + i] = uid_bcc[i];
    }
    if (!calcular_crc(trama, 7, &trama[7])) {
        return 0;
    }
    return transmitir(trama, sizeof(trama), respuesta, &largo) && largo == 3;
}

static void detener_tarjeta(void)
{
    /* Envia PICC_HALT para finalizar la comunicacion con la tarjeta actual. */
    uint8_t trama[4] = {PICC_HALT, 0, 0, 0};

    if (calcular_crc(trama, 2, &trama[2])) {
        transmitir(trama, sizeof(trama), 0, 0);
    }
}

uint8_t rc522_iniciar(void)
{
    /* Reinicia y configura el RC522. El pin IRQ del modulo no se utiliza;
       la presencia de tarjeta se comprueba periodicamente desde main. */
    uint16_t espera = 1000;

    spi_iniciar();
    DDRB |= (1 << RC522_RST);
    PORTB &= (uint8_t)~(1 << RC522_RST);
    _delay_ms(5);
    PORTB |= (1 << RC522_RST);
    _delay_ms(50);

    escribir(COMMAND_REG, CMD_SOFT_RESET);
    do {
        _delay_ms(1);
        if (!(leer(COMMAND_REG) & 0x10U)) {
            break;
        }
    } while (--espera > 0);

    /* Parámetros recomendados para tarjetas ISO/IEC 14443A a 106 kbps. */
    escribir(T_MODE_REG, 0x8DU);
    escribir(T_PRESCALER_REG, 0x3EU);
    escribir(T_RELOAD_L_REG, 30U);
    escribir(T_RELOAD_H_REG, 0);
    escribir(TX_ASK_REG, 0x40U);
    escribir(MODE_REG, 0x3DU);
    activar_bits(TX_CONTROL_REG, 0x03U);
    return leer(VERSION_REG);
}

uint8_t rc522_leer_uid(uint8_t uid[4])
{
    /* Secuencia completa: REQA, anticolision, seleccion y HALT. */
    uint8_t uid_bcc[5];
    uint8_t i;

    if (!solicitar() || !anticolision(uid_bcc) || !seleccionar(uid_bcc)) {
        return 0;
    }
    for (i = 0; i < 4; i++) {
        uid[i] = uid_bcc[i];
    }
    detener_tarjeta();
    return 1;
}

/* ==================== LCD I2C ==================== */

#define LCD_RS  0x01U
#define LCD_EN  0x04U
#define LCD_LUZ 0x08U

static uint8_t activa;
static uint8_t direccion_actual;

static uint8_t twi_esperar(void)
{
    /* Espera por polling a que TWINT indique fin de una operacion I2C.
       TWIE no esta habilitado, por lo tanto no se genera una ISR de TWI. */
    uint16_t limite = 60000;

    while (!(TWCR & (1 << TWINT))) {
        if (--limite == 0) {
            return 0;
        }
    }
    return 1;
}

static void twi_detener(void)
{
    /* Genera la condicion STOP del bus I2C. */
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

static uint8_t twi_comenzar(uint8_t direccion)
{
    /* Genera START y envia la direccion I2C en modo escritura. */
    uint8_t estado;

    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    if (!twi_esperar()) {
        return 0;
    }
    estado = TWSR & 0xF8U;
    if (estado != 0x08U && estado != 0x10U) {
        twi_detener();
        return 0;
    }

    TWDR = (uint8_t)(direccion << 1);
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!twi_esperar() || (TWSR & 0xF8U) != 0x18U) {
        twi_detener();
        return 0;
    }
    return 1;
}

static uint8_t twi_enviar(uint8_t dato)
{
    /* Envia un byte y verifica que el esclavo responda con ACK. */
    TWDR = dato;
    TWCR = (1 << TWINT) | (1 << TWEN);
    return twi_esperar() && (TWSR & 0xF8U) == 0x28U;
}

static uint8_t expansor(uint8_t dato)
{
    /* Envia un byte al expansor PCF8574 conectado a la LCD. */
    uint8_t correcto;

    if (!twi_comenzar(direccion_actual)) {
        return 0;
    }
    correcto = twi_enviar((uint8_t)(dato | LCD_LUZ));
    twi_detener();
    return correcto;
}

static void nibble(uint8_t dato, uint8_t control)
{
    /* Transfiere cuatro bits y genera el pulso Enable de la LCD. */
    dato = (uint8_t)((dato & 0xF0U) | control);
    expansor((uint8_t)(dato | LCD_EN));
    _delay_us(1);
    expansor((uint8_t)(dato & (uint8_t)~LCD_EN));
    _delay_us(50);
}

static void enviar(uint8_t control, uint8_t dato)
{
    /* Divide cada byte en nibble alto y nibble bajo. */
    if (!activa) {
        return;
    }
    nibble(dato, control);
    nibble((uint8_t)(dato << 4), control);
}

static void comando(uint8_t dato)
{
    /* Envia una instruccion al controlador HD44780. */
    enviar(0, dato);
    if (dato == 0x01U || dato == 0x02U) {
        _delay_ms(2);
    }
}

static void caracter(char dato)
{
    /* Envia un caracter visible a la LCD. */
    enviar(LCD_RS, (uint8_t)dato);
}

uint8_t lcd_iniciar(void)
{
    uint8_t direccion;

    /* TWI a 100 kHz, sin aritmetica de punto flotante.
       El bus se maneja por polling, no por interrupciones. */
    TWSR = 0;
    TWBR = (uint8_t)((F_CPU / 100000UL - 16UL) / 2UL);
    TWCR = (1 << TWEN);
    PORTC |= (1 << PC4) | (1 << PC5);

    /* Busca automaticamente cualquier dispositivo I2C valido. En este
       montaje el unico dispositivo del bus es el adaptador de la LCD. */
    activa = 0;
    for (direccion = 0x08U; direccion <= 0x77U; direccion++) {
        if (twi_comenzar(direccion)) {
            twi_detener();
            direccion_actual = direccion;
            activa = 1;
            break;
        }
    }
    if (!activa) {
        return 0;
    }

    /* Secuencia de arranque de la LCD HD44780 en modo de cuatro bits. */
    _delay_ms(50);
    nibble(0x30U, 0);
    _delay_ms(5);
    nibble(0x30U, 0);
    _delay_us(150);
    nibble(0x30U, 0);
    nibble(0x20U, 0);
    comando(0x28U);
    comando(0x0CU);
    comando(0x06U);
    comando(0x01U);
    return 1;
}

uint8_t lcd_direccion(void)
{
    /* Devuelve la direccion I2C encontrada durante el escaneo. */
    return direccion_actual;
}

void lcd_linea(uint8_t fila, const char *texto)
{
    /* Escribe exactamente 16 posiciones y completa con espacios. */
    uint8_t posicion;

    if (!activa) {
        return;
    }
    comando(fila == 0 ? 0x80U : 0xC0U);
    for (posicion = 0; posicion < 16; posicion++) {
        caracter(*texto != '\0' ? *texto++ : ' ');
    }
}

void lcd_linea_flash(uint8_t fila, PGM_P texto)
{
    /* Copia una linea desde Flash y luego la muestra en la LCD. */
    char linea[17];
    uint8_t posicion = 0;
    char dato;

    while (posicion < 16 &&
           (dato = (char)pgm_read_byte(texto++)) != '\0') {
        linea[posicion++] = dato;
    }
    linea[posicion] = '\0';
    lcd_linea(fila, linea);
}

void lcd_valor_flash(uint8_t fila, PGM_P titulo, uint32_t valor)
{
    /* Forma una linea con titulo y numero sin usar sprintf. */
    char linea[17];
    char numero[11];
    uint8_t posicion = 0;
    uint8_t inicio = sizeof(numero);
    char dato;

    while (posicion < 16 &&
           (dato = (char)pgm_read_byte(titulo++)) != '\0') {
        linea[posicion++] = dato;
    }
    do {
        numero[--inicio] = (char)('0' + valor % 10U);
        valor /= 10U;
    } while (valor != 0);
    while (posicion < 16 && inicio < sizeof(numero)) {
        linea[posicion++] = numero[inicio++];
    }
    linea[posicion] = '\0';
    lcd_linea(fila, linea);
}

/* ==================== TECLADO ==================== */

static uint8_t columna_presionada(void)
{
    /* Las columnas tienen pull-up: una columna en cero indica pulsacion. */
    if (!(PIND & (1 << PD6))) return 0;
    if (!(PIND & (1 << PD7))) return 1;
    if (!(PINB & (1 << PB0))) return 2;
    if (!(PINC & (1 << PC0))) return 3;
    return 0xFFU;
}

void teclado_iniciar(void)
{
    /* Filas D2-D5 como salidas y columnas D6, D7, D8 y A0 con pull-up. */
    DDRD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
    PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
    DDRD &= (uint8_t)~((1 << PD6) | (1 << PD7));
    PORTD |= (1 << PD6) | (1 << PD7);
    DDRB &= (uint8_t)~(1 << PB0);
    PORTB |= (1 << PB0);
    DDRC &= (uint8_t)~(1 << PC0);
    PORTC |= (1 << PC0);
}

char teclado_leer(void)
{
    /* Escaneo matricial por polling. No se usan interrupciones de pin.
       Se activa una fila por vez y se observan las cuatro columnas. */
    static const char teclas[4][4] = {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
    };
    uint8_t fila;
    uint8_t columna;

    for (fila = 0; fila < 4; fila++) {
        PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
        PORTD &= (uint8_t)~(1 << (PD2 + fila));
        _delay_us(10);
        columna = columna_presionada();
        if (columna != 0xFFU) {
            /* Retardo de antirrebote del contacto mecanico. */
            _delay_ms(25);
            if (columna_presionada() == columna) {
                /* Espera bloqueante hasta que el usuario suelte la tecla. */
                while (columna_presionada() != 0xFFU) {
                }
                PORTD |= (1 << PD2) | (1 << PD3) |
                         (1 << PD4) | (1 << PD5);
                /* El conector del teclado fisico esta cableado en orden
                   inverso: se intercambian filas/columnas y ambos indices. */
                return teclas[3U - columna][3U - fila];
            }
        }
    }
    PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
    return '\0';
}

/* ==================== LED Y BUZZER ==================== */

#define LED_VERDE PC1
#define LED_ROJO  PC2
#define BUZZER    PC3

static void tono(uint16_t ciclos, uint16_t demora_us)
{
    /* Genera una onda cuadrada cambiando el pin por software.
       No utiliza Timer, PWM ni interrupciones; mientras suena, bloquea. */
    while (ciclos-- > 0) {
        PORTC ^= (1 << BUZZER);
        if (demora_us == 250U) {
            _delay_us(250);
        } else {
            _delay_us(1000);
        }
    }
    PORTC &= (uint8_t)~(1 << BUZZER);
}

void salidas_iniciar(void)
{
    /* Configura LED verde, LED rojo y buzzer como salidas digitales. */
    DDRC |= (1 << LED_VERDE) | (1 << LED_ROJO) | (1 << BUZZER);
    PORTC &= (uint8_t)~((1 << LED_VERDE) | (1 << LED_ROJO) |
                       (1 << BUZZER));
}

void indicar_exito(void)
{
    /* Señal visual y sonora para una operacion correcta. */
    PORTC |= (1 << LED_VERDE);
    tono(400, 250);
    _delay_ms(250);
    PORTC &= (uint8_t)~(1 << LED_VERDE);
}

void indicar_error(void)
{
    /* Señal visual y sonora para una operacion incorrecta. */
    PORTC |= (1 << LED_ROJO);
    tono(300, 1000);
    PORTC &= (uint8_t)~(1 << LED_ROJO);
}

void alarma_intentos(void)
{
    /* Alarma de tres tonos luego de tres intentos de PIN fallidos. */
    uint8_t i;

    PORTC |= (1 << LED_ROJO);
    for (i = 0; i < 3; i++) {
        tono(300, 1000);
        _delay_ms(120);
    }
    PORTC &= (uint8_t)~(1 << LED_ROJO);
}

/* ==================== LOGICA DEL CAJERO ==================== */

#define CANTIDAD_CUENTAS 3U
#define VERSION_EEPROM   0x4A46U

typedef struct {
    /* Datos asociados a cada tarjeta registrada. */
    uint8_t uid[4];
    char pin[4];
    int32_t saldo;
} Cuenta;

typedef enum {
    /* La variable estado define como se interpreta cada entrada. */
    ESPERAR_TARJETA,
    INGRESAR_PIN,
    MOSTRAR_MENU,
    INGRESAR_RETIRO,
    INGRESAR_DEPOSITO,
    INGRESAR_NUEVO_PIN,
    CONFIRMAR_NUEVO_PIN
} Estado;

/* Datos iniciales de prueba. Los UID se reemplazan por los de las tarjetas reales. */
static const Cuenta cuentas_iniciales[CANTIDAD_CUENTAS] PROGMEM = {
    {{0xC3, 0x50, 0xB9, 0x14}, {'1', '2', '3', '4'}, 15000},
    {{0x0C, 0xAD, 0x38, 0x4A}, {'1', '2', '3', '4'}, 8000},
    {{0xDE, 0xAD, 0xBE, 0xEF}, {'1', '2', '3', '4'}, 25000}
};

static Cuenta EEMEM cuentas_eeprom[CANTIDAD_CUENTAS];
static uint16_t EEMEM version_eeprom;

static Cuenta cuentas[CANTIDAD_CUENTAS];
static Estado estado;
static int8_t cuenta_actual;
static uint8_t intentos_pin;
static char pin_ingresado[4];
static char nuevo_pin[4];
static uint8_t largo_pin;
static uint32_t monto;
static uint8_t digitos_monto;
static uint8_t lector_disponible;

static void imprimir_uid(const uint8_t uid[4])
{
    /* Presenta los cuatro bytes del UID por UART en hexadecimal. */
    uint8_t i;

    UART("UID detectado: ");
    for (i = 0; i < 4; i++) {
        uart_hex(uid[i]);
        uart_caracter(i < 3 ? ' ' : '\r');
    }
    uart_caracter('\n');
}

static void cuentas_iniciar(void)
{
    /* Si la EEPROM no tiene esta version, carga los valores iniciales.
       Luego copia todas las cuentas a SRAM para trabajar con ellas. */
    Cuenta temporal;
    uint8_t i, j;

    /* La versión evita reinicializar los saldos en cada encendido. */
    if (eeprom_read_word(&version_eeprom) != VERSION_EEPROM) {
        for (i = 0; i < CANTIDAD_CUENTAS; i++) {
            /* Copia manual desde Flash para admitir versiones antiguas
               de AVR-GCC que no incluyen la rutina de copia especial. */
            for (j = 0; j < sizeof(Cuenta); j++) {
                ((uint8_t *)&temporal)[j] = pgm_read_byte(
                    ((const uint8_t *)&cuentas_iniciales[i]) + j);
            }
            eeprom_update_block(&temporal, &cuentas_eeprom[i],
                                sizeof(Cuenta));
        }
        eeprom_update_word(&version_eeprom, VERSION_EEPROM);
    }
    eeprom_read_block(cuentas, cuentas_eeprom, sizeof(cuentas));
}

static int8_t buscar_cuenta(const uint8_t uid[4])
{
    /* Busca coincidencia exacta de los cuatro bytes del UID. */
    uint8_t i;

    for (i = 0; i < CANTIDAD_CUENTAS; i++) {
        if (memcmp(uid, cuentas[i].uid, sizeof(cuentas[i].uid)) == 0) {
            return (int8_t)i;
        }
    }
    return -1;
}

static void mostrar_espera(void)
{
    /* Estado inicial y final de cada sesion. */
    estado = ESPERAR_TARJETA;
    cuenta_actual = -1;
    LCD(0, "CAJERO ATM");
    LCD(1, "Acerque tarjeta");
    UART("\r\n=== CAJERO AUTOMATICO ===\r\n");
    UART("Acerque una tarjeta al lector RC522.\r\n");
}

static void mostrar_pin(void)
{
    /* Oculta cada digito ingresado mostrando un asterisco. */
    char linea[17] = "PIN:            ";
    uint8_t i;

    for (i = 0; i < largo_pin; i++) {
        linea[5U + i] = '*';
    }
    LCD(0, "Ingrese PIN");
    lcd_linea(1, linea);
}

static void mostrar_menu(void)
{
    /* Cambia al estado de menu y muestra las cuatro opciones. */
    estado = MOSTRAR_MENU;
    LCD(0, "A Saldo B Retiro");
    LCD(1, "C Dep D Sal *PIN");
    UART("\r\nMENU: A Saldo | B Retiro | C Deposito | D Salir | * Cambiar PIN\r\n");
}

static void mostrar_cambio_pin(void)
{
    char linea[17] = "PIN:            ";
    uint8_t i;

    for (i = 0; i < largo_pin; i++) {
        linea[5U + i] = '*';
    }
    if (estado == INGRESAR_NUEVO_PIN) {
        LCD(0, "Nuevo PIN");
    } else {
        LCD(0, "Repita PIN");
    }
    lcd_linea(1, linea);
}

static void mostrar_monto(void)
{
    /* Presenta el monto acumulado en retiro o deposito. */
    if (estado == INGRESAR_RETIRO) {
        LCD(0, "RETIRO");
    } else {
        LCD(0, "DEPOSITO");
    }
    LCD_VALOR(1, "Monto: $", monto);
}

static void verificar_pin(void)
{
    /* Compara siempre cuatro caracteres. Al tercer error activa la alarma
       y finaliza la sesion actual. */
    if (memcmp(pin_ingresado, cuentas[cuenta_actual].pin, 4) == 0) {
        UART("PIN correcto.\r\n");
        LCD(0, "PIN correcto");
        LCD(1, "Bienvenido");
        indicar_exito();
        mostrar_menu();
        return;
    }

    intentos_pin++;
    largo_pin = 0;
    UART("PIN incorrecto.\r\n");
    LCD(0, "PIN incorrecto");
    indicar_error();

    if (intentos_pin >= 3) {
        UART("Tres intentos fallidos. Acceso bloqueado.\r\n");
        LCD(0, "3 intentos");
        LCD(1, "Acceso bloqueado");
        alarma_intentos();
        _delay_ms(1000);
        mostrar_espera();
    } else {
        mostrar_pin();
    }
}

static void procesar_pin(char tecla)
{
    /* D cancela, * borra y los digitos completan el PIN. */
    if (tecla == 'D') {
        UART("Ingreso cancelado.\r\n");
        mostrar_espera();
    } else if (tecla == '*') {
        if (largo_pin > 0) {
            largo_pin--;
        }
        mostrar_pin();
    } else if (tecla >= '0' && tecla <= '9' && largo_pin < 4) {
        pin_ingresado[largo_pin++] = tecla;
        mostrar_pin();
        if (largo_pin == 4) {
            verificar_pin();
        }
    }
}

static void consultar_saldo(void)
{
    /* Muestra el saldo sin modificarlo y luego vuelve al menu. */
    uint32_t saldo = (uint32_t)cuentas[cuenta_actual].saldo;

    LCD(0, "Consulta saldo");
    LCD_VALOR(1, "Saldo: $", saldo);
    UART("Saldo actual: $");
    uart_numero(saldo);
    UART("\r\n");
    indicar_exito();
    _delay_ms(1200);
    mostrar_menu();
}

static void guardar_saldo(void)
{
    /* Persiste solamente el saldo actual. eeprom_update evita escribir
       celdas que ya contienen el mismo valor. */
    eeprom_update_dword((uint32_t *)&cuentas_eeprom[cuenta_actual].saldo,
                        (uint32_t)cuentas[cuenta_actual].saldo);
}

static void guardar_pin(void)
{
    /* Actualiza el PIN en SRAM y EEPROM. La EEPROM conserva el cambio
       aunque se reinicie o se desconecte la alimentacion del Arduino. */
    memcpy(cuentas[cuenta_actual].pin, nuevo_pin, sizeof(nuevo_pin));
    eeprom_update_block(nuevo_pin, &cuentas_eeprom[cuenta_actual].pin,
                        sizeof(nuevo_pin));
}

static void confirmar_retiro(void)
{
    /* Rechaza cero y montos superiores al saldo disponible. */
    if (monto == 0) {
        LCD(0, "Monto invalido");
        UART("Monto de retiro invalido.\r\n");
        indicar_error();
    } else if (monto > (uint32_t)cuentas[cuenta_actual].saldo) {
        LCD(0, "Saldo insufic.");
        UART("Saldo insuficiente.\r\n");
        indicar_error();
    } else {
        cuentas[cuenta_actual].saldo -= (int32_t)monto;
        guardar_saldo();
        LCD(0, "Retiro exitoso");
        LCD_VALOR(1, "Saldo: $", (uint32_t)cuentas[cuenta_actual].saldo);
        UART("Retiro exitoso. Nuevo saldo: $");
        uart_numero((uint32_t)cuentas[cuenta_actual].saldo);
        UART("\r\n");
        indicar_exito();
    }
    _delay_ms(1200);
    mostrar_menu();
}

static void confirmar_deposito(void)
{
    /* Rechaza cero y evita desbordar el rango positivo de int32_t. */
    if (monto == 0 ||
        monto > (uint32_t)(2147483647L - cuentas[cuenta_actual].saldo)) {
        LCD(0, "Monto invalido");
        UART("Monto de deposito invalido.\r\n");
        indicar_error();
    } else {
        cuentas[cuenta_actual].saldo += (int32_t)monto;
        guardar_saldo();
        LCD(0, "Deposito exitoso");
        LCD_VALOR(1, "Saldo: $", (uint32_t)cuentas[cuenta_actual].saldo);
        UART("Deposito exitoso. Nuevo saldo: $");
        uart_numero((uint32_t)cuentas[cuenta_actual].saldo);
        UART("\r\n");
        indicar_exito();
    }
    _delay_ms(1200);
    mostrar_menu();
}

static void iniciar_operacion(Estado operacion)
{
    /* Prepara una nueva carga de monto para retiro o deposito. */
    estado = operacion;
    monto = 0;
    digitos_monto = 0;
    mostrar_monto();
    UART("Ingrese el monto. # confirma, * borra y D cancela.\r\n");
}

static void iniciar_cambio_pin(void)
{
    /* El usuario ya fue autenticado, por eso se solicita directamente
       el PIN nuevo y luego una segunda escritura para confirmarlo. */
    estado = INGRESAR_NUEVO_PIN;
    largo_pin = 0;
    mostrar_cambio_pin();
    UART("Ingrese el nuevo PIN de 4 digitos. * borra y D cancela.\r\n");
}

static void procesar_cambio_pin(char tecla)
{
    if (tecla == 'D') {
        UART("Cambio de PIN cancelado.\r\n");
        mostrar_menu();
        return;
    }

    if (tecla == '*') {
        if (largo_pin > 0) {
            largo_pin--;
        }
        mostrar_cambio_pin();
        return;
    }

    if (tecla < '0' || tecla > '9' || largo_pin >= 4) {
        return;
    }

    if (estado == INGRESAR_NUEVO_PIN) {
        nuevo_pin[largo_pin++] = tecla;
        mostrar_cambio_pin();
        if (largo_pin == 4) {
            estado = CONFIRMAR_NUEVO_PIN;
            largo_pin = 0;
            mostrar_cambio_pin();
            UART("Repita el nuevo PIN.\r\n");
        }
        return;
    }

    pin_ingresado[largo_pin++] = tecla;
    mostrar_cambio_pin();
    if (largo_pin == 4) {
        if (memcmp(pin_ingresado, nuevo_pin, sizeof(nuevo_pin)) == 0) {
            guardar_pin();
            LCD(0, "PIN actualizado");
            LCD(1, "Cambio guardado");
            UART("PIN actualizado y guardado en EEPROM.\r\n");
            indicar_exito();
            _delay_ms(800);
            mostrar_menu();
        } else {
            LCD(0, "PIN no coincide");
            LCD(1, "Intente de nuevo");
            UART("Los PIN no coinciden. Ingrese nuevamente el PIN nuevo.\r\n");
            indicar_error();
            _delay_ms(800);
            estado = INGRESAR_NUEVO_PIN;
            largo_pin = 0;
            mostrar_cambio_pin();
        }
    }
}

static void procesar_menu(char tecla)
{
    /* A consulta, B retira, C deposita y D cierra la sesion. */
    switch (tecla) {
        case 'A':
            consultar_saldo();
            break;
        case 'B':
            iniciar_operacion(INGRESAR_RETIRO);
            break;
        case 'C':
            iniciar_operacion(INGRESAR_DEPOSITO);
            break;
        case 'D':
            UART("Sesion finalizada.\r\n");
            LCD(0, "Sesion cerrada");
            LCD(1, "Retire tarjeta");
            _delay_ms(1000);
            mostrar_espera();
            break;
        case '*':
            iniciar_cambio_pin();
            break;
        default:
            break;
    }
}

static void procesar_monto(char tecla)
{
    /* D cancela, * elimina el ultimo digito, # confirma y los numeros
       construyen el monto con monto = monto * 10 + nuevo_digito. */
    if (tecla == 'D') {
        UART("Operacion cancelada.\r\n");
        mostrar_menu();
    } else if (tecla == '*') {
        if (digitos_monto > 0) {
            monto /= 10U;
            digitos_monto--;
            mostrar_monto();
        }
    } else if (tecla == '#') {
        if (estado == INGRESAR_RETIRO) {
            confirmar_retiro();
        } else {
            confirmar_deposito();
        }
    } else if (tecla >= '0' && tecla <= '9' && digitos_monto < 9) {
        monto = monto * 10U + (uint8_t)(tecla - '0');
        digitos_monto++;
        mostrar_monto();
    }
}

static void procesar_tarjeta(void)
{
    /* Consulta el RC522. Si obtiene un UID registrado, inicia el ingreso
       de PIN; si no esta registrado, informa el UID y vuelve a esperar. */
    uint8_t uid[4];

    if (!rc522_leer_uid(uid)) {
        return;
    }

    imprimir_uid(uid);
    cuenta_actual = buscar_cuenta(uid);
    if (cuenta_actual < 0) {
        UART("Tarjeta no registrada. Anote el UID mostrado.\r\n");
        LCD(0, "Tarjeta invalida");
        LCD(1, "UID por terminal");
        indicar_error();
        _delay_ms(700);
        mostrar_espera();
        return;
    }

    UART("Tarjeta reconocida. Ingrese PIN.\r\n");
    estado = INGRESAR_PIN;
    intentos_pin = 0;
    largo_pin = 0;
    mostrar_pin();
}

void cajero_iniciar(void)
{
    /* Inicializa una vez todos los perifericos y datos del cajero. */
    uint8_t version_rc522;

    uart_iniciar();
    salidas_iniciar();
    teclado_iniciar();
    cuentas_iniciar();

    if (lcd_iniciar()) {
        UART("LCD I2C detectado en 0x");
        uart_hex(lcd_direccion());
        UART(".\r\n");
    } else {
        UART("Aviso: no se encontro ningun dispositivo I2C.\r\n");
    }

    version_rc522 = rc522_iniciar();
    UART("RC522 VersionReg: 0x");
    uart_hex(version_rc522);
    UART("\r\n");
    /* 0x91 y 0x92 corresponden al MFRC522 original.
       El lector utilizado en el montaje se identifica como 0xB2. */
    lector_disponible = version_rc522 == 0x91U ||
                        version_rc522 == 0x92U ||
                        version_rc522 == 0xB2U;

    if (!lector_disponible) {
        UART("ERROR: revise 3.3 V, GND y las conexiones SPI.\r\n");
        LCD(0, "Error RC522");
        LCD(1, "Revise conexion");
        indicar_error();
        _delay_ms(1200);
    }
    mostrar_espera();
}

void cajero_actualizar(void)
{
    char tecla;

    /* Cada llamada procesa una sola entrada de la maquina de estados.
       Esta funcion se invoca repetidamente desde while(1); no es una ISR. */
    if (estado == ESPERAR_TARJETA) {
        if (lector_disponible) {
            procesar_tarjeta();
        }
        _delay_ms(30);
        return;
    }

    tecla = teclado_leer();
    if (tecla == '\0') {
        return;
    }

    /* El switch implementa directamente la maquina de estados. */
    switch (estado) {
        case INGRESAR_PIN:
            procesar_pin(tecla);
            break;

        case MOSTRAR_MENU:
            procesar_menu(tecla);
            break;

        case INGRESAR_RETIRO:
        case INGRESAR_DEPOSITO:
            procesar_monto(tecla);
            break;

        case INGRESAR_NUEVO_PIN:
        case CONFIRMAR_NUEVO_PIN:
            procesar_cambio_pin(tecla);
            break;

        case ESPERAR_TARJETA:
        default:
            break;
    }
}

/* ==================== PROGRAMA PRINCIPAL ==================== */

int main(void)
{
    /* Superloop: inicializacion unica y actualizacion continua por polling. */
    cajero_iniciar();

    while (1) {
        cajero_actualizar();
    }
}
