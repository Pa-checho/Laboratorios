#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>

#define BAUDIOS 9600UL
#define UBRR_VALOR ((F_CPU / (16UL * BAUDIOS)) - 1UL)
#define LCD_DIRECCION 0x3EU
#define CANTIDAD_CUENTAS 3U
#define MARCA_EEPROM 0x4A43U

#define LED_VERDE PC1
#define LED_ROJO  PC2
#define BUZZER    PC3

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
static uint16_t EEMEM marca_eeprom;

static Cuenta cuentas[CANTIDAD_CUENTAS];
static Estado estado;
static int8_t cuenta_actual;
static uint8_t intentos_pin;
static char pin_ingresado[4];
static uint8_t largo_pin;
static uint32_t monto_ingresado;
static uint8_t digitos_monto;
static char uid_texto[8];
static uint8_t largo_uid;
static uint8_t lcd_activo;

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

static uint8_t uart_hay_dato(void)
{
    return (UCSR0A & (1 << RXC0)) != 0;
}

static char uart_recibir(void)
{
    return (char)UDR0;
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

    resultado = TWSR & 0xF8;
    if (resultado != 0x08 && resultado != 0x10) {
        twi_detener();
        return 0;
    }

    TWDR = (uint8_t)(direccion << 1);
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!twi_esperar() || (TWSR & 0xF8) != 0x18) {
        twi_detener();
        return 0;
    }
    return 1;
}

static uint8_t twi_enviar(uint8_t dato)
{
    TWDR = dato;
    TWCR = (1 << TWINT) | (1 << TWEN);
    return twi_esperar() && (TWSR & 0xF8) == 0x28;
}

static uint8_t lcd_enviar(uint8_t control, uint8_t dato)
{
    uint8_t correcto;

    if (!lcd_activo || !twi_comenzar(LCD_DIRECCION)) {
        return 0;
    }

    correcto = twi_enviar(control);
    correcto &= twi_enviar(dato);
    twi_detener();
    return correcto;
}

static void lcd_comando(uint8_t comando)
{
    lcd_enviar(0x80, comando);
    if (comando == 0x01 || comando == 0x02) {
        _delay_ms(2);
    } else {
        _delay_us(50);
    }
}

static void lcd_dato(char dato)
{
    lcd_enviar(0x40, (uint8_t)dato);
    _delay_us(50);
}

static void lcd_iniciar(void)
{
    lcd_activo = twi_comenzar(LCD_DIRECCION);
    if (!lcd_activo) {
        return;
    }
    twi_detener();

    _delay_ms(50);
    lcd_comando(0x02);
    _delay_ms(100);
    lcd_comando(0x0C);
    lcd_comando(0x28);
    lcd_comando(0x01);
    lcd_comando(0x06);
}

static void lcd_linea(uint8_t fila, const char *texto)
{
    uint8_t posicion;

    if (!lcd_activo) {
        return;
    }

    lcd_comando(fila == 0 ? 0x80 : 0xC0);
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
    return 0xFF;
}

static char teclado_leer(void)
{
    static const char teclas[4][4] = {
        {'7', '8', '9', '/'},
        {'4', '5', '6', '*'},
        {'1', '2', '3', '-'},
        {'C', '0', '=', '+'}
    };
    uint8_t fila;
    uint8_t columna;

    for (fila = 0; fila < 4; fila++) {
        PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
        PORTD &= (uint8_t)~(1 << (PD2 + fila));
        _delay_us(10);

        columna = columna_presionada();
        if (columna != 0xFF) {
            _delay_ms(25);
            if (columna_presionada() == columna) {
                while (columna_presionada() != 0xFF) {
                }
                PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
                return teclas[fila][columna];
            }
        }
    }

    PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
    return '\0';
}

static void cuentas_iniciar(void)
{
    static const Cuenta iniciales[CANTIDAD_CUENTAS] = {
        {{0xA1, 0xB2, 0xC3, 0xD4}, {'1', '2', '3', '4'}, 15000},
        {{0x11, 0x22, 0x33, 0x44}, {'1', '2', '3', '4'}, 8000},
        {{0xDE, 0xAD, 0xBE, 0xEF}, {'1', '2', '3', '4'}, 25000}
    };

    if (eeprom_read_word(&marca_eeprom) != MARCA_EEPROM) {
        eeprom_update_block(iniciales, cuentas_eeprom, sizeof(iniciales));
        eeprom_update_word(&marca_eeprom, MARCA_EEPROM);
    }
    eeprom_read_block(cuentas, cuentas_eeprom, sizeof(cuentas));
}

static int8_t valor_hexadecimal(char dato)
{
    if (dato >= '0' && dato <= '9') return (int8_t)(dato - '0');
    if (dato >= 'A' && dato <= 'F') return (int8_t)(dato - 'A' + 10);
    if (dato >= 'a' && dato <= 'f') return (int8_t)(dato - 'a' + 10);
    return -1;
}

static int8_t buscar_cuenta(void)
{
    uint8_t uid[4];
    uint8_t i;
    uint8_t cuenta;

    for (i = 0; i < 4; i++) {
        uid[i] = (uint8_t)((valor_hexadecimal(uid_texto[i * 2U]) << 4) |
                           valor_hexadecimal(uid_texto[i * 2U + 1U]));
    }

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
    largo_uid = 0;
    lcd_linea(0, "CAJERO ATM");
    lcd_linea(1, "Acerque tarjeta");
    uart_texto("\r\n=== CAJERO AUTOMATICO ===\r\n");
    uart_texto("Ingrese UID de 8 caracteres:\r\n");
    uart_texto("Cuenta 1: A1B2C3D4\r\n");
    uart_texto("Cuenta 2: 11223344\r\n");
    uart_texto("Cuenta 3: DEADBEEF\r\n");
    uart_texto("PIN para las tres cuentas: 1234\r\n> ");
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
    lcd_linea(0, "1Saldo 2Retiro");
    lcd_linea(1, "3Depos CSalir");
    uart_texto("\r\nMENU: 1 Saldo | 2 Retiro | 3 Deposito | C Salir\r\n");
}

static void mostrar_monto(const char *operacion)
{
    lcd_linea(0, operacion);
    lcd_numero("Monto: $", monto_ingresado);
}

static void procesar_uid(char dato)
{
    int8_t valor;

    if (dato == '\r' || dato == '\n') {
        if (largo_uid != 0 && largo_uid != 8) {
            uart_texto("\r\nUID incompleto. Intente nuevamente.\r\n> ");
            largo_uid = 0;
        }
        return;
    }

    valor = valor_hexadecimal(dato);
    if (valor < 0) {
        return;
    }

    uart_caracter(dato);
    uid_texto[largo_uid++] = dato;
    if (largo_uid < 8) {
        return;
    }

    cuenta_actual = buscar_cuenta();
    largo_uid = 0;
    if (cuenta_actual < 0) {
        uart_texto("\r\nTarjeta no registrada.\r\n> ");
        lcd_linea(0, "Tarjeta invalida");
        lcd_linea(1, "Intente de nuevo");
        indicar_error();
        lcd_linea(0, "CAJERO ATM");
        lcd_linea(1, "Acerque tarjeta");
        return;
    }

    uart_texto("\r\nTarjeta reconocida. Ingrese PIN.\r\n");
    estado = INGRESAR_PIN;
    intentos_pin = 0;
    largo_pin = 0;
    mostrar_pin();
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
        uart_texto("Tres intentos fallidos. Tarjeta bloqueada.\r\n");
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
    if (tecla == 'C') {
        largo_pin = 0;
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

static void iniciar_operacion(Estado nueva, const char *titulo)
{
    estado = nueva;
    monto_ingresado = 0;
    digitos_monto = 0;
    mostrar_monto(titulo);
    uart_texto("Ingrese el monto y confirme con =. C cancela.\r\n");
}

static void procesar_menu(char tecla)
{
    switch (tecla) {
        case '1':
            consultar_saldo();
            break;
        case '2':
            iniciar_operacion(INGRESAR_RETIRO, "RETIRO");
            break;
        case '3':
            iniciar_operacion(INGRESAR_DEPOSITO, "DEPOSITO");
            break;
        case 'C':
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
    if (tecla == 'C') {
        uart_texto("Operacion cancelada.\r\n");
        mostrar_menu();
        return;
    }

    if (tecla == '=') {
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

int main(void)
{
    char tecla;

    uart_iniciar();
    salidas_iniciar();
    teclado_iniciar();
    twi_iniciar();
    lcd_iniciar();
    cuentas_iniciar();

    _delay_ms(250);
    if (!lcd_activo) {
        uart_texto("Aviso: LCD I2C 0x3E no responde.\r\n");
    }
    mostrar_espera();

    while (1) {
        if (estado == ESPERAR_TARJETA && uart_hay_dato()) {
            procesar_uid(uart_recibir());
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
