#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define BAUDIOS 9600UL
#define UBRR_VALOR ((F_CPU / (16UL * BAUDIOS)) - 1UL)

#define RC522_RST PB1
#define RC522_SS  PB2
#define SPI_MOSI  PB3
#define SPI_MISO  PB4
#define SPI_SCK   PB5

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

#define PICC_REQA         0x26U
#define PICC_ANTICOLL_CL1 0x93U
#define PICC_HALT         0x50U

static void uart_iniciar(void)
{
    UBRR0H = (uint8_t)(UBRR_VALOR >> 8);
    UBRR0L = (uint8_t)UBRR_VALOR;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_caracter(char dato)
{
    while (!(UCSR0A & (1 << UDRE0))) {
    }
    UDR0 = dato;
}

static void uart_texto(const char *texto)
{
    while (*texto != '\0') {
        uart_caracter(*texto++);
    }
}

static void uart_hex(uint8_t valor)
{
    static const char digitos[] = "0123456789ABCDEF";

    uart_caracter(digitos[valor >> 4]);
    uart_caracter(digitos[valor & 0x0FU]);
}

static void uart_uid(const uint8_t uid[4])
{
    uint8_t i;

    uart_texto("UID detectado: ");
    for (i = 0; i < 4; i++) {
        uart_hex(uid[i]);
        if (i < 3) {
            uart_caracter(' ');
        }
    }
    uart_texto("\r\n");
}

static uint8_t spi_transferir(uint8_t dato)
{
    SPDR = dato;
    while (!(SPSR & (1 << SPIF))) {
    }
    return SPDR;
}

static void spi_iniciar(void)
{
    DDRB |= (1 << RC522_RST) | (1 << RC522_SS) | (1 << SPI_MOSI) | (1 << SPI_SCK);
    DDRB &= (uint8_t)~(1 << SPI_MISO);
    PORTB |= (1 << RC522_SS) | (1 << RC522_RST);
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
    SPSR = 0;
}

static void rc522_escribir(uint8_t registro, uint8_t valor)
{
    PORTB &= (uint8_t)~(1 << RC522_SS);
    spi_transferir((uint8_t)((registro << 1) & 0x7EU));
    spi_transferir(valor);
    PORTB |= (1 << RC522_SS);
}

static uint8_t rc522_leer(uint8_t registro)
{
    uint8_t valor;

    PORTB &= (uint8_t)~(1 << RC522_SS);
    spi_transferir((uint8_t)(((registro << 1) & 0x7EU) | 0x80U));
    valor = spi_transferir(0);
    PORTB |= (1 << RC522_SS);
    return valor;
}

static void rc522_activar_bits(uint8_t registro, uint8_t mascara)
{
    rc522_escribir(registro, (uint8_t)(rc522_leer(registro) | mascara));
}

static void rc522_desactivar_bits(uint8_t registro, uint8_t mascara)
{
    rc522_escribir(registro, (uint8_t)(rc522_leer(registro) & (uint8_t)~mascara));
}

static uint8_t rc522_calcular_crc(const uint8_t *datos, uint8_t cantidad,
                                  uint8_t resultado[2])
{
    uint16_t espera = 5000;
    uint8_t i;

    rc522_escribir(COMMAND_REG, CMD_IDLE);
    rc522_escribir(DIV_IRQ_REG, 0x04U);
    rc522_activar_bits(FIFO_LEVEL_REG, 0x80U);

    for (i = 0; i < cantidad; i++) {
        rc522_escribir(FIFO_DATA_REG, datos[i]);
    }

    rc522_escribir(COMMAND_REG, CMD_CALC_CRC);
    while (espera-- > 0) {
        if (rc522_leer(DIV_IRQ_REG) & 0x04U) {
            resultado[0] = rc522_leer(CRC_RESULT_L);
            resultado[1] = rc522_leer(CRC_RESULT_H);
            rc522_escribir(COMMAND_REG, CMD_IDLE);
            return 1;
        }
    }

    rc522_escribir(COMMAND_REG, CMD_IDLE);
    return 0;
}

static uint8_t rc522_transmitir(const uint8_t *envio, uint8_t largo_envio,
                                uint8_t *respuesta, uint8_t *largo_respuesta)
{
    uint16_t espera = 3000;
    uint8_t interrupcion;
    uint8_t cantidad;
    uint8_t capacidad = largo_respuesta != 0 ? *largo_respuesta : 0;
    uint8_t i;

    rc522_escribir(COMMAND_REG, CMD_IDLE);
    rc522_escribir(COM_IRQ_REG, 0x7FU);
    rc522_activar_bits(FIFO_LEVEL_REG, 0x80U);

    for (i = 0; i < largo_envio; i++) {
        rc522_escribir(FIFO_DATA_REG, envio[i]);
    }

    rc522_escribir(COMMAND_REG, CMD_TRANSCEIVE);
    rc522_activar_bits(BIT_FRAMING_REG, 0x80U);

    do {
        interrupcion = rc522_leer(COM_IRQ_REG);
        if (interrupcion & 0x30U) {
            break;
        }
        if (interrupcion & 0x01U) {
            rc522_desactivar_bits(BIT_FRAMING_REG, 0x80U);
            return 0;
        }
    } while (--espera > 0);

    rc522_desactivar_bits(BIT_FRAMING_REG, 0x80U);
    if (espera == 0 || (rc522_leer(ERROR_REG) & 0x13U)) {
        return 0;
    }

    if (respuesta != 0 && largo_respuesta != 0) {
        cantidad = rc522_leer(FIFO_LEVEL_REG);
        if (cantidad > capacidad) {
            cantidad = capacidad;
        }
        for (i = 0; i < cantidad; i++) {
            respuesta[i] = rc522_leer(FIFO_DATA_REG);
        }
        *largo_respuesta = cantidad;
    }
    return 1;
}

static uint8_t rc522_solicitar(void)
{
    uint8_t comando = PICC_REQA;
    uint8_t respuesta[2];
    uint8_t largo = sizeof(respuesta);
    uint8_t correcto;

    rc522_escribir(BIT_FRAMING_REG, 0x07U);
    correcto = rc522_transmitir(&comando, 1, respuesta, &largo);
    rc522_escribir(BIT_FRAMING_REG, 0);
    return correcto && largo == 2;
}

static uint8_t rc522_anticolision(uint8_t uid_bcc[5])
{
    uint8_t comando[2] = {PICC_ANTICOLL_CL1, 0x20U};
    uint8_t largo = 5;

    rc522_escribir(BIT_FRAMING_REG, 0);
    rc522_desactivar_bits(COLL_REG, 0x80U);
    if (!rc522_transmitir(comando, 2, uid_bcc, &largo) || largo != 5) {
        return 0;
    }
    return (uint8_t)(uid_bcc[0] ^ uid_bcc[1] ^ uid_bcc[2] ^ uid_bcc[3]) == uid_bcc[4];
}

static uint8_t rc522_seleccionar(const uint8_t uid_bcc[5])
{
    uint8_t trama[9];
    uint8_t respuesta[3];
    uint8_t largo = sizeof(respuesta);
    uint8_t i;

    trama[0] = PICC_ANTICOLL_CL1;
    trama[1] = 0x70U;
    for (i = 0; i < 5; i++) {
        trama[2U + i] = uid_bcc[i];
    }
    if (!rc522_calcular_crc(trama, 7, &trama[7])) {
        return 0;
    }
    return rc522_transmitir(trama, sizeof(trama), respuesta, &largo) && largo == 3;
}

static void rc522_detener_tarjeta(void)
{
    uint8_t trama[4] = {PICC_HALT, 0, 0, 0};

    if (rc522_calcular_crc(trama, 2, &trama[2])) {
        rc522_transmitir(trama, sizeof(trama), 0, 0);
    }
}

static uint8_t rc522_leer_uid(uint8_t uid[4])
{
    uint8_t uid_bcc[5];
    uint8_t i;

    if (!rc522_solicitar() || !rc522_anticolision(uid_bcc)) {
        return 0;
    }
    if (!rc522_seleccionar(uid_bcc)) {
        return 0;
    }

    for (i = 0; i < 4; i++) {
        uid[i] = uid_bcc[i];
    }
    rc522_detener_tarjeta();
    return 1;
}

static uint8_t rc522_iniciar(void)
{
    uint16_t espera = 1000;
    uint8_t version;

    spi_iniciar();
    PORTB &= (uint8_t)~(1 << RC522_RST);
    _delay_ms(5);
    PORTB |= (1 << RC522_RST);
    _delay_ms(50);

    rc522_escribir(COMMAND_REG, CMD_SOFT_RESET);
    do {
        _delay_ms(1);
        if (!(rc522_leer(COMMAND_REG) & 0x10U)) {
            break;
        }
    } while (--espera > 0);

    rc522_escribir(T_MODE_REG, 0x8DU);
    rc522_escribir(T_PRESCALER_REG, 0x3EU);
    rc522_escribir(T_RELOAD_L_REG, 30U);
    rc522_escribir(T_RELOAD_H_REG, 0);
    rc522_escribir(TX_ASK_REG, 0x40U);
    rc522_escribir(MODE_REG, 0x3DU);
    rc522_activar_bits(TX_CONTROL_REG, 0x03U);

    version = rc522_leer(VERSION_REG);
    uart_texto("RC522 VersionReg: 0x");
    uart_hex(version);
    uart_texto("\r\n");
    return version != 0x00U && version != 0xFFU;
}

int main(void)
{
    uint8_t uid[4];

    uart_iniciar();
    uart_texto("\r\nPrueba fisica del lector RC522\r\n");

    if (!rc522_iniciar()) {
        uart_texto("ERROR: revise 3.3 V, GND y las conexiones SPI.\r\n");
        while (1) {
        }
    }

    uart_texto("Lector listo. Acerque una tarjeta.\r\n");
    while (1) {
        if (rc522_leer_uid(uid)) {
            uart_uid(uid);
            uart_texto("Retire la tarjeta antes de leer otra.\r\n");
            _delay_ms(500);
        }
        _delay_ms(30);
    }
}
