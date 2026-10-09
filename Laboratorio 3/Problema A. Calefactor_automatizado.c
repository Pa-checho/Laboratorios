/* Etapa 6: version completa, con ingreso numerico y validacion. */
/*
 * Control de temperatura - ATmega328P - Microchip Studio / AVR-GCC
 * Reloj supuesto: 16 MHz. Compilar con optimizacion -Os.
 * F_CPU debe coincidir con el reloj REAL; no configura los fusibles.
 *
 * DHT11 DATA: PD2, con resistencia de 4,7 kOhm a VCC.
 * Calefactor: LED en PB0. Ventilador: LED 1/2/3 en PD4/PD5/PD6.
 * Cada LED lleva una resistencia de 330 ohm y su catodo a GND.
 * LCD 16x2 HD44780: RS=PB1, E=PB2, D4..D7=PC0..PC3, RW=GND.
 * Serie TX: PD1 hacia RX del terminal. RX: PD0 desde TX del terminal.
 * Serie: 9600 baudios, 8 bits, sin paridad, 1 bit de parada.
 * No conectar motores ni calefactores directamente a los pines.
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#define DHT_PIN PD2
#define CALEFACTOR_PIN PB0
#define LED_1 PD4
#define LED_2 PD5
#define LED_3 PD6
#define LCD_RS PB1
#define LCD_E PB2

/* El rango medio es punto_medio - 6 hasta punto_medio + 6.
 * Inicialmente: 22 - 6 = 16 C y 22 + 6 = 28 C.
 * Limites de ingreso -10..60 C; los umbrales se desplazan con el punto medio.
 */
#define PUNTO_MINIMO -10
#define PUNTO_MAXIMO 60
/* El rango de reposo termina 6 C por encima del punto medio.
 * No permitir que ese limite supere el maximo medible del DHT11.
 */
#define DHT11_MAXIMO 50
#define MARGEN_SUPERIOR 6
int16_t punto_medio = 22;
int16_t temperatura = -2; /* -2: sin primera lectura. -1: error. */

/* volatile indica que la variable puede cambiar en una interrupcion. */
volatile uint8_t medir = 0;

/* Timer1 genera una interrupcion por segundo. */
ISR(TIMER1_COMPA_vect)
{
    static uint8_t segundos = 0;
    segundos++;
    if (segundos == 5) {
        segundos = 0;
        medir = 1;
    }
}

void serie_iniciar(void)
{
    uint16_t divisor = F_CPU / (16UL * 9600UL) - 1;
    UBRR0H = divisor >> 8;
    UBRR0L = divisor;
    UCSR0A = 0;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0); /* Envio y recepcion. */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); /* Formato 8N1. */
}

void serie_texto(const char *texto)
{
    while (*texto != '\0') {
        while (!(UCSR0A & (1 << UDRE0))) {
            /* Espera hasta poder enviar otro caracter. */
        }
        UDR0 = *texto;
        texto++;
    }
}

/* Convierte un entero a texto decimal sin depender de itoa.
 * El arreglo de salida debe tener espacio para 7 caracteres.
 */
void numero_a_texto(int16_t numero, char *texto)
{
    char digitos[5];
    uint8_t cantidad = 0;
    uint8_t posicion = 0;
    uint16_t valor;

    if (numero < 0) {
        texto[posicion++] = '-';
        /* Se usa 32 bits para poder convertir tambien -32768. */
        valor = (uint16_t)(-(int32_t)numero);
    } else {
        valor = (uint16_t)numero;
    }

    /* Extrae los digitos desde las unidades. Incluye el caso cero. */
    do {
        digitos[cantidad++] = '0' + valor % 10;
        valor /= 10;
    } while (valor > 0);

    /* Copia los digitos en el orden correcto. */
    while (cantidad > 0) texto[posicion++] = digitos[--cantidad];
    texto[posicion] = '\0'; /* Marca el final del texto. */
}

void serie_numero(int16_t numero)
{
    char texto[7];
    numero_a_texto(numero, texto);
    serie_texto(texto);
}

/* ---------------- PANTALLA LCD EN MODO DE 4 BITS ---------------- */

void lcd_4bits(uint8_t dato)
{
    /* PC0..PC3 van a D4..D7. Conserva los otros bits del puerto C. */
    PORTC = (PORTC & 0xF0) | (dato & 0x0F);
    _delay_us(1);
    PORTB |= (1 << LCD_E);
    _delay_us(1);
    PORTB &= ~(1 << LCD_E);
    _delay_us(1);
}

void lcd_enviar(uint8_t dato, uint8_t es_caracter)
{
    if (es_caracter) PORTB |= (1 << LCD_RS);
    else PORTB &= ~(1 << LCD_RS);

    lcd_4bits(dato >> 4); /* Primero los cuatro bits superiores. */
    lcd_4bits(dato);      /* Luego los cuatro inferiores. */
    _delay_us(50);
}

void lcd_comando(uint8_t comando)
{
    lcd_enviar(comando, 0);
    if (comando == 0x01 || comando == 0x02) _delay_ms(2);
}

void lcd_texto(const char *texto)
{
    while (*texto != '\0') {
        lcd_enviar(*texto, 1);
        texto++;
    }
}

void lcd_numero(int16_t numero)
{
    char texto[7];
    numero_a_texto(numero, texto);
    lcd_texto(texto);
}

void lcd_iniciar(void)
{
    PORTB &= ~((1 << LCD_RS) | (1 << LCD_E));
    DDRB |= (1 << LCD_RS) | (1 << LCD_E);
    DDRC |= 0x0F;
    PORTC &= 0xF0;

    /* Secuencia de inicio indicada por el controlador HD44780. */
    _delay_ms(40);
    lcd_4bits(3);
    _delay_ms(5);
    lcd_4bits(3);
    _delay_us(150);
    lcd_4bits(3);
    _delay_us(150);
    lcd_4bits(2);
    _delay_us(50);
    lcd_comando(0x28); /* 4 bits, 2 lineas, caracteres de 5x8. */
    lcd_comando(0x08); /* Pantalla apagada durante el inicio. */
    lcd_comando(0x01); /* Borra la pantalla. */
    lcd_comando(0x06); /* Avanza el cursor al escribir. */
    lcd_comando(0x0C); /* Pantalla encendida, cursor oculto. */
}

void lcd_actualizar(void)
{
    lcd_comando(0x80); /* Principio de la primera linea. */
    lcd_texto("                ");
    lcd_comando(0x80);
    if (temperatura == -2) lcd_texto("T:--");
    else if (temperatura < 0) lcd_texto("T:ERR");
    else {
        lcd_texto("T:");
        lcd_numero(temperatura);
        lcd_texto("C");
    }
    lcd_texto(" PM:");
    lcd_numero(punto_medio);
    lcd_texto("C");

    lcd_comando(0xC0); /* Principio de la segunda linea. */
    lcd_texto("                ");
    lcd_comando(0xC0);
    lcd_texto("Medio:");
    lcd_numero(punto_medio - 6);
    lcd_texto("-");
    lcd_numero(punto_medio + 6);
    lcd_texto("C");
}

void temporizador_iniciar(void)
{
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = F_CPU / 256UL - 1; /* Un segundo a 8 o 16 MHz. */
    TIFR1 = (1 << OCF1A);
    TIMSK1 = (1 << OCIE1A);
    /* Modo CTC: cuenta hasta OCR1A y vuelve a cero. Divisor: 256. */
    TCCR1B = (1 << WGM12) | (1 << CS12);
}

void leds_iniciar(void)
{
    PORTD &= ~((1 << LED_1) | (1 << LED_2) | (1 << LED_3));
    DDRD |= (1 << LED_1) | (1 << LED_2) | (1 << LED_3);
}

void ventilador(uint8_t nivel)
{
    /* Nivel 0: ninguno; 1: un LED; 2: dos; 3: los tres. */
    PORTD &= ~((1 << LED_1) | (1 << LED_2) | (1 << LED_3));
    if (nivel >= 1) PORTD |= (1 << LED_1);
    if (nivel >= 2) PORTD |= (1 << LED_2);
    if (nivel >= 3) PORTD |= (1 << LED_3);
}

/* Espera un nivel del DHT11. Devuelve 0 si el sensor no responde.
 * El limite evita que el programa se quede bloqueado indefinidamente.
 */
uint8_t dht_esperar(uint8_t nivel)
{
    uint8_t intentos = 0;
    while (((PIND & (1 << DHT_PIN)) != 0) != nivel) {
        if (++intentos == 100) {
            return 0;
        }
        _delay_us(1);
    }
    return 1;
}

/* Devuelve la temperatura entera, o -1 si falla la comunicacion.
 * Se llama con interrupciones deshabilitadas para respetar los pulsos.
 * El DHT11 entrega 5 bytes: humedad, decimal, temperatura, decimal,
 * y suma de comprobacion. Aqui usamos solo la temperatura entera.
 */
int16_t dht11_leer(void)
{
    uint8_t datos[5] = {0, 0, 0, 0, 0};
    uint8_t i;

    /* Solicita una lectura manteniendo DATA bajo por 20 ms. */
    PORTD &= ~(1 << DHT_PIN);
    DDRD |= (1 << DHT_PIN);
    _delay_ms(20);

    /* Libera DATA; la resistencia externa eleva la linea. */
    DDRD &= ~(1 << DHT_PIN);
    _delay_us(10);

    /* Respuesta inicial: nivel bajo, alto y comienzo de datos. */
    if (!dht_esperar(0)) return -1;
    if (!dht_esperar(1)) return -1;
    if (!dht_esperar(0)) return -1;

    for (i = 0; i < 40; i++) {
        if (!dht_esperar(1)) return -1;
        /* Un 0 tiene un pulso alto corto; un 1, uno mas largo. */
        _delay_us(40);
        datos[i / 8] <<= 1;
        if (PIND & (1 << DHT_PIN)) {
            datos[i / 8] |= 1;
        }
        if (!dht_esperar(0)) return -1;
    }

    /* Comprueba que los datos recibidos no esten corruptos. */
    if ((uint8_t)(datos[0] + datos[1] + datos[2] + datos[3]) != datos[4]) {
        return -1;
    }
    return datos[2];
}

void controlar_temperatura(void)
{
    /* Siempre apaga primero el calefactor antes de decidir. */
    PORTB &= ~(1 << CALEFACTOR_PIN);

    if (temperatura < 0) {
        ventilador(0);
        if (temperatura == -2) serie_texto("Esperando primera lectura.\r\n");
        else serie_texto("ERROR DHT11 | Todas las salidas apagadas.\r\n");
        return;
    }

    serie_texto("Temperatura: ");
    serie_numero(temperatura);
    serie_texto(" C | ");

    if (temperatura < punto_medio - 6) {
        ventilador(0);
        PORTB |= (1 << CALEFACTOR_PIN);
        serie_texto("Calefactor encendido | Ventilador apagado\r\n");
    } else if (temperatura <= punto_medio + 6) {
        ventilador(0);
        serie_texto("Calefactor apagado | Ventilador apagado\r\n");
    } else if (temperatura <= punto_medio + 17) {
        ventilador(1);
        serie_texto("Calefactor apagado | Ventilador BAJO: 1 LED\r\n");
    } else if (temperatura <= punto_medio + 28) {
        ventilador(2);
        serie_texto("Calefactor apagado | Ventilador MEDIO: 2 LED\r\n");
    } else {
        /* Con punto medio 22, el nivel alto empieza en 51 C. */
        ventilador(3);
        serie_texto("Calefactor apagado | Ventilador ALTO: 3 LED\r\n");
    }
}

/* ---------------- MENU POR EL PUERTO SERIE ---------------- */

void mostrar_rangos(void)
{
    serie_texto("\r\nPunto medio: ");
    serie_numero(punto_medio);
    serie_texto(" C\r\nCalefactor: T < ");
    serie_numero(punto_medio - 6);
    serie_texto(" C\r\nRango medio: ");
    serie_numero(punto_medio - 6);
    serie_texto(" a ");
    serie_numero(punto_medio + 6);
    serie_texto(" C\r\nBajo (1 LED): ");
    serie_numero(punto_medio + 7);
    serie_texto(" a ");
    serie_numero(punto_medio + 17);
    serie_texto(" C\r\nMedio (2 LED): ");
    serie_numero(punto_medio + 18);
    serie_texto(" a ");
    serie_numero(punto_medio + 28);
    serie_texto(" C\r\nAlto (3 LED): desde ");
    serie_numero(punto_medio + 29);
    serie_texto(" C\r\n");
}

void mostrar_menu(void)
{
    serie_texto("\r\n--- MENU DE TEMPERATURA ---\r\n");
    serie_texto("Escribir punto medio (-10 a 44) y pulsar Enter.\r\n");
    serie_texto("Proteccion DHT11: se rechazan valores mayores de 44 C.\r\n");
    serie_texto("r: restaurar 22 C | m: ver menu y rangos\r\n");
    serie_texto("Retroceso: borrar digito | Escape: cancelar entrada\r\n");
    mostrar_rangos();
}

void atender_menu(void)
{
    /* static conserva lo escrito entre una llamada y la siguiente.
     * Guarda hasta dos digitos y un signo menos opcional al principio.
     */
    static char entrada[3];
    static uint8_t cantidad = 0;
    static uint8_t entrada_invalida = 0;
    char opcion;
    char eco[2];
    uint8_t estado;
    uint8_t i;
    uint8_t inicio = 0;
    int16_t nuevo_punto = 0;
    uint8_t cambio = 0;

    /* No bloquea esperando Enter; las mediciones siguen funcionando. */
    if (!(UCSR0A & (1 << RXC0))) return;
    estado = UCSR0A;
    opcion = UDR0;
    if (estado & ((1 << FE0) | (1 << DOR0) | (1 << UPE0))) {
        /* Si se pierde un caracter, nunca aplicar un numero incompleto. */
        entrada_invalida = 1;
        serie_texto("\r\nError de recepcion: Escape y escribir otra vez.\r\n");
        return;
    }

    if (opcion == 27) { /* Escape cancela, sin cambiar el punto medio. */
        cantidad = 0;
        entrada_invalida = 0;
        serie_texto("\r\nEntrada cancelada.\r\n");
        return;
    }

    if (opcion == '\r' || opcion == '\n') {
        /* Ignora lineas vacias y el segundo caracter de un Enter CR+LF. */
        if (cantidad == 0 && !entrada_invalida) return;
        serie_texto("\r\n");
        if (cantidad > 0 && entrada[0] == '-') inicio = 1;
        if (inicio == 1 && cantidad == 1) entrada_invalida = 1;
        for (i = inicio; i < cantidad; i++) {
            nuevo_punto = nuevo_punto * 10 + (entrada[i] - '0');
        }
        if (inicio == 1) nuevo_punto = -nuevo_punto;
        if (entrada_invalida || nuevo_punto < PUNTO_MINIMO ||
            nuevo_punto > PUNTO_MAXIMO) {
            serie_texto("Valor invalido. Escribir un entero de -10 a 44.\r\n");
        } else if (nuevo_punto + MARGEN_SUPERIOR > DHT11_MAXIMO) {
            /* Rechaza antes de modificar el punto medio o las salidas. */
            serie_texto("Cambio rechazado: punto medio demasiado cercano ");
            serie_texto("o superior al maximo del DHT11 (50 C).\r\n");
            serie_texto("Se requiere un margen de 6 C. Maximo permitido: 44 C.\r\n");
            serie_texto("Se mantiene el punto medio en ");
            serie_numero(punto_medio);
            serie_texto(" C.\r\n");
        } else {
            punto_medio = nuevo_punto;
            cambio = 1;
        }
        cantidad = 0;
        entrada_invalida = 0;
    } else if (opcion == '\b' || opcion == 127) {
        if (cantidad > 0 && !entrada_invalida) {
            cantidad--;
            serie_texto("\b \b");
        }
    } else if (opcion == '-' && cantidad == 0 && !entrada_invalida) {
        entrada[cantidad++] = '-'; /* Solo permite el signo al comienzo. */
        serie_texto("-");
    } else if (opcion >= '0' && opcion <= '9') {
        if (cantidad > 0 && entrada[0] == '-') inicio = 1;
        if (cantidad < 2 + inicio && !entrada_invalida) {
            entrada[cantidad++] = opcion;
            eco[0] = opcion;
            eco[1] = '\0';
            serie_texto(eco); /* Permite ver los digitos escritos. */
        } else {
            entrada_invalida = 1; /* Rechaza numeros demasiado largos. */
        }
    } else if ((opcion == 'r' || opcion == 'R') &&
               cantidad == 0 && !entrada_invalida) {
        punto_medio = 22;
        cambio = 1;
    } else if ((opcion == 'm' || opcion == 'M') &&
               cantidad == 0 && !entrada_invalida) {
        mostrar_menu();
    } else {
        /* Rechaza letras, signos fuera de lugar y decimales. */
        entrada_invalida = 1;
    }

    if (cambio) {
        serie_texto("Punto medio actualizado: ");
        serie_numero(punto_medio);
        serie_texto(" C\r\n");
        /* Aplica la ultima lectura sin reiniciar el temporizador. */
        controlar_temperatura();
        lcd_actualizar();
        mostrar_rangos();
    }
}

int main(void)
{

    PORTB &= ~(1 << CALEFACTOR_PIN);
    DDRB |= (1 << CALEFACTOR_PIN);
    DDRD &= ~(1 << DHT_PIN);

    serie_iniciar();
    leds_iniciar();
    lcd_iniciar();
    lcd_actualizar();
    mostrar_menu();
    serie_texto("Primera medicion en 5 segundos.\r\n");
    temporizador_iniciar();
    sei(); /* Habilita las interrupciones. */

    while (1) {
        atender_menu();
        if (medir) {
            medir = 0;
            cli(); /* Protege los tiempos cortos del protocolo DHT11. */
            temperatura = dht11_leer();
            sei();

            controlar_temperatura();
            lcd_actualizar();
        }
    }
}