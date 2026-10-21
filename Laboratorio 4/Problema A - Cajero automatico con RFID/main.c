#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>

#define BAUDIOS 9600UL
#define UBRR_VALOR ((F_CPU / (16UL * BAUDIOS)) - 1UL)
#define CANTIDAD_CUENTAS 3U
#define VERSION_DATOS_EEPROM 0x4A45U

#define LCD_RS        0x01U
#define LCD_EN        0x04U
#define LCD_LUZ       0x08U

#define LED_VERDE PC1
#define LED_ROJO  PC2
#define BUZZER    PC3

#define RC522_RST PB1
#define RC522_SS  PB2
#define SPI_MOSI  PB3
#define SPI_MISO  PB4
#define SPI_SCK   PB5

#define RC522_COMMAND_REG     0x01U
#define RC522_COM_IRQ_REG     0x04U
#define RC522_DIV_IRQ_REG     0x05U
#define RC522_ERROR_REG       0x06U
#define RC522_FIFO_DATA_REG   0x09U
#define RC522_FIFO_LEVEL_REG  0x0AU
#define RC522_CONTROL_REG     0x0CU
#define RC522_BIT_FRAMING_REG 0x0DU
#define RC522_COLL_REG        0x0EU
#define RC522_MODE_REG        0x11U
#define RC522_TX_CONTROL_REG  0x14U
#define RC522_TX_ASK_REG      0x15U
#define RC522_CRC_RESULT_H    0x21U
#define RC522_CRC_RESULT_L    0x22U
#define RC522_T_MODE_REG      0x2AU
#define RC522_T_PRESCALER_REG 0x2BU
#define RC522_T_RELOAD_H_REG  0x2CU
#define RC522_T_RELOAD_L_REG  0x2DU
#define RC522_VERSION_REG     0x37U

#define RC522_CMD_IDLE        0x00U
#define RC522_CMD_CALC_CRC    0x03U
#define RC522_CMD_TRANSCEIVE  0x0CU
#define RC522_CMD_SOFT_RESET  0x0FU

#define PICC_REQA             0x26U
#define PICC_ANTICOLL_CL1     0x93U
#define PICC_HALT             0x50U

typedef struct {
    uint8_t uid[4];
    char pin[4];
    int32_t saldo;
} Cuenta;

typedef enum {
    ESPERAR_TARJETA,
    INGRESAR_PIN,
    MOSTRAR_MENU,
    INGRESAR_RETIRO,
    INGRESAR_DEPOSITO
} Estado;

static Cuenta EEMEM cuentas_eeprom[CANTIDAD_CUENTAS];
static uint16_t EEMEM version_eeprom;

static Cuenta cuentas[CANTIDAD_CUENTAS];
static Estado estado;
static int8_t cuenta_actual;
static uint8_t intentos_pin;
static char pin_ingresado[4];
static uint8_t largo_pin;
static uint32_t monto_ingresado;
static uint8_t digitos_monto;
static uint8_t lcd_activo;
static uint8_t lcd_direccion;

static void uart_iniciar(void)
{
    UBRR0H = (uint8_t)(UBRR_VALOR >> 8);
    UBRR0L = (uint8_t)UBRR_VALOR;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_caracter(char dato)
{
    while (!(UCSR0A & (1 << UDRE0))) {
    }
    UDR0 = (uint8_t)dato;
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

static void numero_texto(uint32_t numero, char *texto)
{
    char invertido[11];
    uint8_t cantidad = 0;
    uint8_t i;

    if (numero == 0) {
        texto[0] = '0';
        texto[1] = '\0';
        return;
    }

    while (numero > 0 && cantidad < sizeof(invertido)) {
        invertido[cantidad++] = (char)('0' + (numero % 10U));
        numero /= 10U;
    }

    for (i = 0; i < cantidad; i++) {
        texto[i] = invertido[cantidad - 1U - i];
    }
    texto[cantidad] = '\0';
}

static void uart_numero(uint32_t numero)
{
    char texto[12];
    numero_texto(numero, texto);
    uart_texto(texto);
}

static void twi_iniciar(void)
{
    TWSR = 0;
    TWBR = 152;
    TWCR = (1 << TWEN);
    PORTC |= (1 << PC4) | (1 << PC5);
}

static uint8_t twi_esperar(void)
{
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
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

static uint8_t twi_comenzar(uint8_t direccion)
{
    uint8_t resultado;

    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    if (!twi_esperar()) {
        return 0;
    }

    resultado = TWSR & 0xF8U;
    if (resultado != 0x08U && resultado != 0x10U) {
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
    TWDR = dato;
    TWCR = (1 << TWINT) | (1 << TWEN);
    return twi_esperar() && (TWSR & 0xF8U) == 0x28U;
}

static uint8_t lcd_expansor(uint8_t dato)
{
    uint8_t correcto;

    if (!twi_comenzar(lcd_direccion)) {
        return 0;
    }

    correcto = twi_enviar((uint8_t)(dato | LCD_LUZ));
    twi_detener();
    return correcto;
}

static void lcd_nibble(uint8_t nibble, uint8_t control)
{
    uint8_t dato = (uint8_t)((nibble & 0xF0U) | control);

    lcd_expansor((uint8_t)(dato | LCD_EN));
    _delay_us(1);
    lcd_expansor((uint8_t)(dato & (uint8_t)~LCD_EN));
    _delay_us(50);
}

static void lcd_enviar(uint8_t control, uint8_t dato)
{
    if (!lcd_activo) {
        return;
    }

    lcd_nibble(dato, control);
    lcd_nibble((uint8_t)(dato << 4), control);
}

static void lcd_comando(uint8_t comando)
{
    lcd_enviar(0, comando);
    if (comando == 0x01U || comando == 0x02U) {
        _delay_ms(2);
    } else {
        _delay_us(50);
    }
}

static void lcd_dato(char dato)
{
    lcd_enviar(LCD_RS, (uint8_t)dato);
    _delay_us(50);
}

static void lcd_iniciar(void)
{
    static const uint8_t direcciones[] = {0x27U, 0x3FU};
    uint8_t i;

    lcd_activo = 0;
    for (i = 0; i < sizeof(direcciones); i++) {
        if (twi_comenzar(direcciones[i])) {
            twi_detener();
            lcd_direccion = direcciones[i];
            lcd_activo = 1;
            break;
        }
    }
    if (!lcd_activo) {
        return;
    }

    _delay_ms(50);
    lcd_nibble(0x30U, 0);
    _delay_ms(5);
    lcd_nibble(0x30U, 0);
    _delay_us(150);
    lcd_nibble(0x30U, 0);
    lcd_nibble(0x20U, 0);
    lcd_comando(0x28U);
    lcd_comando(0x0CU);
    lcd_comando(0x06U);
    lcd_comando(0x01U);
}

static void lcd_linea(uint8_t fila, const char *texto)
{
    uint8_t posicion;

    if (!lcd_activo) {
        return;
    }

    lcd_comando(fila == 0 ? 0x80U : 0xC0U);
    for (posicion = 0; posicion < 16; posicion++) {
        if (*texto != '\0') {
            lcd_dato(*texto++);
        } else {
            lcd_dato(' ');
        }
    }
}

static void lcd_numero(const char *titulo, uint32_t numero)
{
    char linea[17];
    char valor[12];
    uint8_t posicion = 0;
    uint8_t i = 0;

    while (posicion < 16 && titulo[i] != '\0') {
        linea[posicion++] = titulo[i++];
    }
    numero_texto(numero, valor);
    i = 0;
    while (posicion < 16 && valor[i] != '\0') {
        linea[posicion++] = valor[i++];
    }
    while (posicion < 16) {
        linea[posicion++] = ' ';
    }
    linea[16] = '\0';
    lcd_linea(1, linea);
}

static void salidas_iniciar(void)
{
    DDRC |= (1 << LED_VERDE) | (1 << LED_ROJO) | (1 << BUZZER);
    PORTC &= (uint8_t)~((1 << LED_VERDE) | (1 << LED_ROJO) | (1 << BUZZER));
}

static void tono_correcto(void)
{
    uint16_t i;

    for (i = 0; i < 400; i++) {
        PORTC ^= (1 << BUZZER);
        _delay_us(250);
    }
    PORTC &= (uint8_t)~(1 << BUZZER);
}

static void tono_error(void)
{
    uint16_t i;

    for (i = 0; i < 300; i++) {
        PORTC ^= (1 << BUZZER);
        _delay_us(1000);
    }
    PORTC &= (uint8_t)~(1 << BUZZER);
}

static void indicar_exito(void)
{
    PORTC |= (1 << LED_VERDE);
    tono_correcto();
    _delay_ms(250);
    PORTC &= (uint8_t)~(1 << LED_VERDE);
}

static void indicar_error(void)
{
    PORTC |= (1 << LED_ROJO);
    tono_error();
    PORTC &= (uint8_t)~(1 << LED_ROJO);
}

static void alarma_intentos(void)
{
    uint8_t i;

    PORTC |= (1 << LED_ROJO);
    for (i = 0; i < 3; i++) {
        tono_error();
        _delay_ms(120);
    }
    PORTC &= (uint8_t)~(1 << LED_ROJO);
}

static void teclado_iniciar(void)
{
    DDRD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
    PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);

    DDRD &= (uint8_t)~((1 << PD6) | (1 << PD7));
    PORTD |= (1 << PD6) | (1 << PD7);
    DDRB &= (uint8_t)~(1 << PB0);
    PORTB |= (1 << PB0);
    DDRC &= (uint8_t)~(1 << PC0);
    PORTC |= (1 << PC0);
}

static uint8_t columna_presionada(void)
{
    if (!(PIND & (1 << PD6))) return 0;
    if (!(PIND & (1 << PD7))) return 1;
    if (!(PINB & (1 << PB0))) return 2;
    if (!(PINC & (1 << PC0))) return 3;
    return 0xFFU;
}

static char teclado_leer(void)
{
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
            _delay_ms(25);
            if (columna_presionada() == columna) {
                while (columna_presionada() != 0xFFU) {
                }
                PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
                return teclas[fila][columna];
            }
        }
    }

    PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
    return '\0';
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

static uint8_t rc522_calcular_crc(const uint8_t *datos, uint8_t cantidad, uint8_t resultado[2])
{
    uint16_t espera = 5000;
    uint8_t i;

    rc522_escribir(RC522_COMMAND_REG, RC522_CMD_IDLE);
    rc522_escribir(RC522_DIV_IRQ_REG, 0x04U);
    rc522_activar_bits(RC522_FIFO_LEVEL_REG, 0x80U);

    for (i = 0; i < cantidad; i++) {
        rc522_escribir(RC522_FIFO_DATA_REG, datos[i]);
    }

    rc522_escribir(RC522_COMMAND_REG, RC522_CMD_CALC_CRC);
    while (espera-- > 0) {
        if (rc522_leer(RC522_DIV_IRQ_REG) & 0x04U) {
            resultado[0] = rc522_leer(RC522_CRC_RESULT_L);
            resultado[1] = rc522_leer(RC522_CRC_RESULT_H);
            rc522_escribir(RC522_COMMAND_REG, RC522_CMD_IDLE);
            return 1;
        }
    }
    rc522_escribir(RC522_COMMAND_REG, RC522_CMD_IDLE);
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

    rc522_escribir(RC522_COMMAND_REG, RC522_CMD_IDLE);
    rc522_escribir(RC522_COM_IRQ_REG, 0x7FU);
    rc522_activar_bits(RC522_FIFO_LEVEL_REG, 0x80U);

    for (i = 0; i < largo_envio; i++) {
        rc522_escribir(RC522_FIFO_DATA_REG, envio[i]);
    }

    rc522_escribir(RC522_COMMAND_REG, RC522_CMD_TRANSCEIVE);
    rc522_activar_bits(RC522_BIT_FRAMING_REG, 0x80U);

    do {
        interrupcion = rc522_leer(RC522_COM_IRQ_REG);
        if (interrupcion & 0x30U) {
            break;
        }
        if (interrupcion & 0x01U) {
            rc522_desactivar_bits(RC522_BIT_FRAMING_REG, 0x80U);
            return 0;
        }
    } while (--espera > 0);

    rc522_desactivar_bits(RC522_BIT_FRAMING_REG, 0x80U);
    if (espera == 0 || (rc522_leer(RC522_ERROR_REG) & 0x13U)) {
        return 0;
    }

    if (respuesta != 0 && largo_respuesta != 0) {
        cantidad = rc522_leer(RC522_FIFO_LEVEL_REG);
        if (cantidad > capacidad) {
            cantidad = capacidad;
        }
        for (i = 0; i < cantidad; i++) {
            respuesta[i] = rc522_leer(RC522_FIFO_DATA_REG);
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

    rc522_escribir(RC522_BIT_FRAMING_REG, 0x07U);
    correcto = rc522_transmitir(&comando, 1, respuesta, &largo);
    rc522_escribir(RC522_BIT_FRAMING_REG, 0x00U);
    return correcto && largo == 2;
}

static uint8_t rc522_anticolision(uint8_t uid_bcc[5])
{
    uint8_t comando[2] = {PICC_ANTICOLL_CL1, 0x20U};
    uint8_t largo = 5;

    rc522_escribir(RC522_BIT_FRAMING_REG, 0x00U);
    rc522_desactivar_bits(RC522_COLL_REG, 0x80U);
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
    uint8_t trama[4] = {PICC_HALT, 0x00U, 0, 0};

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

    rc522_escribir(RC522_COMMAND_REG, RC522_CMD_SOFT_RESET);
    do {
        _delay_ms(1);
        if (!(rc522_leer(RC522_COMMAND_REG) & 0x10U)) {
            break;
        }
    } while (--espera > 0);

    rc522_escribir(RC522_T_MODE_REG, 0x8DU);
    rc522_escribir(RC522_T_PRESCALER_REG, 0x3EU);
    rc522_escribir(RC522_T_RELOAD_L_REG, 30U);
    rc522_escribir(RC522_T_RELOAD_H_REG, 0U);
    rc522_escribir(RC522_TX_ASK_REG, 0x40U);
    rc522_escribir(RC522_MODE_REG, 0x3DU);
    rc522_activar_bits(RC522_TX_CONTROL_REG, 0x03U);

    version = rc522_leer(RC522_VERSION_REG);
    uart_texto("RC522 VersionReg: 0x");
    uart_hex(version);
    uart_texto("\r\n");
    return version != 0x00U && version != 0xFFU;
}

static void cuentas_iniciar(void)
{
    static const Cuenta iniciales[CANTIDAD_CUENTAS] = {
        {{0xA1, 0xB2, 0xC3, 0xD4}, {'1', '2', '3', '4'}, 15000},
        {{0x11, 0x22, 0x33, 0x44}, {'1', '2', '3', '4'}, 8000},
        {{0xDE, 0xAD, 0xBE, 0xEF}, {'1', '2', '3', '4'}, 25000}
    };

    if (eeprom_read_word(&version_eeprom) != VERSION_DATOS_EEPROM) {
        eeprom_update_block(iniciales, cuentas_eeprom, sizeof(iniciales));
        eeprom_update_word(&version_eeprom, VERSION_DATOS_EEPROM);
    }
    eeprom_read_block(cuentas, cuentas_eeprom, sizeof(cuentas));
}

static int8_t buscar_cuenta(const uint8_t uid[4])
{
    uint8_t cuenta;

    for (cuenta = 0; cuenta < CANTIDAD_CUENTAS; cuenta++) {
        if (memcmp(uid, cuentas[cuenta].uid, 4) == 0) {
            return (int8_t)cuenta;
        }
    }
    return -1;
}

static void mostrar_espera(void)
{
    estado = ESPERAR_TARJETA;
    cuenta_actual = -1;
    lcd_linea(0, "CAJERO ATM");
    lcd_linea(1, "Acerque tarjeta");
    uart_texto("\r\n=== CAJERO AUTOMATICO ===\r\n");
    uart_texto("Acerque una tarjeta al lector RC522.\r\n");
}

static void mostrar_pin(void)
{
    char linea[17] = "PIN:            ";
    uint8_t i;

    for (i = 0; i < largo_pin; i++) {
        linea[5U + i] = '*';
    }
    lcd_linea(0, "Ingrese PIN");
    lcd_linea(1, linea);
}

static void mostrar_menu(void)
{
    estado = MOSTRAR_MENU;
    lcd_linea(0, "A Saldo B Retiro");
    lcd_linea(1, "C Depos D Salir");
    uart_texto("\r\nMENU: A Saldo | B Retiro | C Deposito | D Salir\r\n");
}

static void mostrar_monto(const char *operacion)
{
    lcd_linea(0, operacion);
    lcd_numero("Monto: $", monto_ingresado);
}

static void verificar_pin(void)
{
    if (memcmp(pin_ingresado, cuentas[cuenta_actual].pin, 4) == 0) {
        uart_texto("PIN correcto.\r\n");
        lcd_linea(0, "PIN correcto");
        lcd_linea(1, "Bienvenido");
        indicar_exito();
        mostrar_menu();
        return;
    }

    intentos_pin++;
    largo_pin = 0;
    uart_texto("PIN incorrecto.\r\n");
    lcd_linea(0, "PIN incorrecto");
    indicar_error();

    if (intentos_pin >= 3) {
        uart_texto("Tres intentos fallidos. Acceso bloqueado.\r\n");
        lcd_linea(0, "3 intentos");
        lcd_linea(1, "Acceso bloqueado");
        alarma_intentos();
        _delay_ms(1000);
        mostrar_espera();
    } else {
        mostrar_pin();
    }
}

static void procesar_pin(char tecla)
{
    if (tecla == 'D') {
        uart_texto("Ingreso cancelado.\r\n");
        mostrar_espera();
        return;
    }

    if (tecla == '*') {
        if (largo_pin > 0) {
            largo_pin--;
        }
        mostrar_pin();
        return;
    }

    if (tecla < '0' || tecla > '9' || largo_pin >= 4) {
        return;
    }

    pin_ingresado[largo_pin++] = tecla;
    mostrar_pin();
    if (largo_pin == 4) {
        verificar_pin();
    }
}

static void consultar_saldo(void)
{
    uint32_t saldo = (uint32_t)cuentas[cuenta_actual].saldo;

    lcd_linea(0, "Consulta saldo");
    lcd_numero("Saldo: $", saldo);
    uart_texto("Saldo actual: $");
    uart_numero(saldo);
    uart_texto("\r\n");
    indicar_exito();
    _delay_ms(1200);
    mostrar_menu();
}

static void guardar_saldo(void)
{
    eeprom_update_dword((uint32_t *)&cuentas_eeprom[cuenta_actual].saldo,
                        (uint32_t)cuentas[cuenta_actual].saldo);
}

static void confirmar_retiro(void)
{
    if (monto_ingresado == 0) {
        lcd_linea(0, "Monto invalido");
        uart_texto("Monto de retiro invalido.\r\n");
        indicar_error();
    } else if (monto_ingresado > (uint32_t)cuentas[cuenta_actual].saldo) {
        lcd_linea(0, "Saldo insufic.");
        uart_texto("Saldo insuficiente.\r\n");
        indicar_error();
    } else {
        cuentas[cuenta_actual].saldo -= (int32_t)monto_ingresado;
        guardar_saldo();
        lcd_linea(0, "Retiro exitoso");
        lcd_numero("Saldo: $", (uint32_t)cuentas[cuenta_actual].saldo);
        uart_texto("Retiro exitoso. Nuevo saldo: $");
        uart_numero((uint32_t)cuentas[cuenta_actual].saldo);
        uart_texto("\r\n");
        indicar_exito();
    }
    _delay_ms(1200);
    mostrar_menu();
}

static void confirmar_deposito(void)
{
    if (monto_ingresado == 0 ||
        monto_ingresado > (uint32_t)(2147483647L - cuentas[cuenta_actual].saldo)) {
        lcd_linea(0, "Monto invalido");
        uart_texto("Monto de deposito invalido.\r\n");
        indicar_error();
    } else {
        cuentas[cuenta_actual].saldo += (int32_t)monto_ingresado;
        guardar_saldo();
        lcd_linea(0, "Deposito exitoso");
        lcd_numero("Saldo: $", (uint32_t)cuentas[cuenta_actual].saldo);
        uart_texto("Deposito exitoso. Nuevo saldo: $");
        uart_numero((uint32_t)cuentas[cuenta_actual].saldo);
        uart_texto("\r\n");
        indicar_exito();
    }
    _delay_ms(1200);
    mostrar_menu();
}

static void iniciar_operacion(Estado nuevo_estado, const char *titulo)
{
    estado = nuevo_estado;
    monto_ingresado = 0;
    digitos_monto = 0;
    mostrar_monto(titulo);
    uart_texto("Ingrese el monto. # confirma, * borra y D cancela.\r\n");
}

static void procesar_menu(char tecla)
{
    switch (tecla) {
        case 'A':
            consultar_saldo();
            break;
        case 'B':
            iniciar_operacion(INGRESAR_RETIRO, "RETIRO");
            break;
        case 'C':
            iniciar_operacion(INGRESAR_DEPOSITO, "DEPOSITO");
            break;
        case 'D':
            uart_texto("Sesion finalizada.\r\n");
            lcd_linea(0, "Sesion cerrada");
            lcd_linea(1, "Retire tarjeta");
            _delay_ms(1000);
            mostrar_espera();
            break;
        default:
            break;
    }
}

static void procesar_monto(char tecla)
{
    if (tecla == 'D') {
        uart_texto("Operacion cancelada.\r\n");
        mostrar_menu();
        return;
    }

    if (tecla == '*') {
        if (digitos_monto > 0) {
            monto_ingresado /= 10U;
            digitos_monto--;
            mostrar_monto(estado == INGRESAR_RETIRO ? "RETIRO" : "DEPOSITO");
        }
        return;
    }

    if (tecla == '#') {
        if (estado == INGRESAR_RETIRO) {
            confirmar_retiro();
        } else {
            confirmar_deposito();
        }
        return;
    }

    if (tecla >= '0' && tecla <= '9' && digitos_monto < 9) {
        monto_ingresado = monto_ingresado * 10U + (uint8_t)(tecla - '0');
        digitos_monto++;
        mostrar_monto(estado == INGRESAR_RETIRO ? "RETIRO" : "DEPOSITO");
    }
}

static void procesar_tarjeta(void)
{
    uint8_t uid[4];

    if (!rc522_leer_uid(uid)) {
        return;
    }

    uart_uid(uid);
    cuenta_actual = buscar_cuenta(uid);
    if (cuenta_actual < 0) {
        uart_texto("Tarjeta no registrada. Anote el UID mostrado.\r\n");
        lcd_linea(0, "Tarjeta invalida");
        lcd_linea(1, "UID por terminal");
        indicar_error();
        _delay_ms(700);
        mostrar_espera();
        return;
    }

    uart_texto("Tarjeta reconocida. Ingrese PIN.\r\n");
    estado = INGRESAR_PIN;
    intentos_pin = 0;
    largo_pin = 0;
    mostrar_pin();
}

int main(void)
{
    char tecla;
    uint8_t lector_disponible;

    uart_iniciar();
    salidas_iniciar();
    teclado_iniciar();
    twi_iniciar();
    lcd_iniciar();
    cuentas_iniciar();

    uart_texto("\r\nIniciando lector RFID...\r\n");
    lector_disponible = rc522_iniciar();
    if (!lector_disponible) {
        uart_texto("ERROR: el RC522 no responde. Revise 3.3 V, GND y SPI.\r\n");
        lcd_linea(0, "Error RC522");
        lcd_linea(1, "Revise conexion");
        indicar_error();
        _delay_ms(1200);
    } else {
        uart_texto("RC522 listo.\r\n");
    }

    if (!lcd_activo) {
        uart_texto("Aviso: LCD I2C no encontrado en 0x27 ni 0x3F.\r\n");
    } else {
        uart_texto("LCD I2C detectado en 0x");
        uart_hex(lcd_direccion);
        uart_texto(".\r\n");
    }
    mostrar_espera();

    while (1) {
        if (estado == ESPERAR_TARJETA) {
            if (lector_disponible) {
                procesar_tarjeta();
            }
            _delay_ms(30);
            continue;
        }

        tecla = teclado_leer();
        if (tecla == '\0') {
            continue;
        }

        if (estado == INGRESAR_PIN) {
            procesar_pin(tecla);
        } else if (estado == MOSTRAR_MENU) {
            procesar_menu(tecla);
        } else if (estado == INGRESAR_RETIRO || estado == INGRESAR_DEPOSITO) {
            procesar_monto(tecla);
        }
    }
}
