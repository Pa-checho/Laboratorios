/* Calefactor automatizado con ajuste del punto medio por puerto serie. */
/*
 * Control de temperatura - ATmega328P - Microchip Studio / AVR-GCC
 * Reloj supuesto: 16 MHz. Compilar con optimizacion -Os.
 * F_CPU debe coincidir con el reloj REAL; no configura los fusibles.
 *
 * DHT11 DATA: PD2, con resistencia de 4,7 kOhm a VCC.
 * Calefactor: LED en PB0. Ventilador: LED 1/2/3 en PD4/PD5/PD6.
 * PWM ventilador: PD3/OC2B (D3), requiere etapa de potencia externa.
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
#include <util/atomic.h>

#define DHT_PIN PD2
#define CALEFACTOR_PIN PB0
#define LED_1 PD4
#define LED_2 PD5
#define LED_3 PD6
#define LCD_RS PB1
#define LCD_E PB2

/* PM=22: calefactor 0..15, reposo 16..25, bajo 26..35,
 * medio 36..45, alto desde 46. Se cubre 46, omitido en la consigna.
 * Histeresis de 1 C: al subir se cruza umbral+1;
 * al bajar se cruza estrictamente por debajo de umbral-1.
 * PM 8..25 mantiene los cinco estados alcanzables entre 0 y 50 C.
 */
#define PUNTO_MINIMO 8
#define PUNTO_MAXIMO 25
#define DHT11_MAXIMO 50
#define HISTERESIS 1
#define VENTILADOR_PWM PD3 /* D3 = OC2B, hacia etapa de potencia. */
enum { CALOR, REPOSO, BAJO, MEDIO, ALTO, SIN_LECTURA };
static uint8_t estado_control = SIN_LECTURA;
static uint8_t nivel_ventilador;
static uint8_t pwm_ventilador;
static volatile uint32_t reloj_segundos;
int16_t punto_medio = 22;
int16_t temperatura = -2; /* -2: sin primera lectura. -1: error. */

/* volatile indica que la variable puede cambiar en una interrupcion. */
volatile uint8_t medir = 0;

/* Timer1 genera una interrupcion por segundo. */
ISR(TIMER1_COMPA_vect)
{
    static uint8_t segundos = 0;
    reloj_segundos++;
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
    else if (temperatura < 0 || temperatura > DHT11_MAXIMO) lcd_texto("T:ERR");
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
    lcd_numero(punto_medio + 3);
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
    /* Los LED conservan la indicacion original; D3 entrega el PWM real.
     * La salida PWM requiere una etapa de potencia para controlar un ventilador.
     */
    nivel_ventilador = nivel;
    pwm_ventilador = nivel * 85u; /* 0, 85, 170, 255. */
    PORTD &= ~((1 << LED_1) | (1 << LED_2) | (1 << LED_3));
    if (nivel >= 1) PORTD |= (1 << LED_1);
    if (nivel >= 2) PORTD |= (1 << LED_2);
    if (nivel >= 3) PORTD |= (1 << LED_3);
    OCR2B = pwm_ventilador;
    if (nivel) TCCR2A |= (1 << COM2B1);
    else {
        TCCR2A &= ~(1 << COM2B1);
        PORTD &= ~(1 << VENTILADOR_PWM); /* Apagado continuo, sin pulsito. */
    }
}

void ventilador_pwm_iniciar(void)
{
    PORTD &= ~(1 << VENTILADOR_PWM);
    DDRD |= (1 << VENTILADOR_PWM);
    TCCR2A = (1 << WGM21) | (1 << WGM20); /* Fast PWM de 8 bits. */
    TCCR2B = (1 << CS22); /* Divisor 64: 976,56 Hz. */
    OCR2B = 0;
}

int16_t umbral(uint8_t frontera)
{
    const int8_t desplazamiento[4] = {-6, 4, 14, 24};
    return punto_medio + desplazamiento[frontera];
}

void enviar_medicion(void)
{
    uint32_t segundos;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { segundos = reloj_segundos; }
    /* A1: cada linea incluye todos los datos necesarios para graficar.
     * MS es tiempo del micro; OK=0 indica que la lectura no es valida.
     */
    serie_texto("A1,MS=");
    char digitos[10];
    uint8_t cantidad = 0;
    uint32_t ms = segundos * 1000UL;
    do { digitos[cantidad++] = '0' + ms % 10; ms /= 10; } while (ms);
    while (cantidad) { char letra[2] = {digitos[--cantidad], 0}; serie_texto(letra); }
    serie_texto(",T="); serie_numero(temperatura);
    serie_texto(",PM="); serie_numero(punto_medio);
    serie_texto(",LI="); serie_numero(punto_medio - 6);
    serie_texto(",LS="); serie_numero(punto_medio + 3);
    serie_texto(",CAL="); serie_numero(estado_control == CALOR);
    serie_texto(",VENT="); serie_numero(nivel_ventilador);
    serie_texto(",PWM="); serie_numero(pwm_ventilador);
    serie_texto(",OK="); serie_numero(temperatura >= 0 && temperatura <= DHT11_MAXIMO);
    serie_texto("\r\n");
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
    PORTB &= ~(1 << CALEFACTOR_PIN);
    if (temperatura < 0 || temperatura > DHT11_MAXIMO) {
        estado_control = SIN_LECTURA;
        ventilador(0);
        enviar_medicion();
        return;
    }
    if (estado_control == SIN_LECTURA) {
        /* Primera lectura o cambio de PM: aplicar intervalos nominales. */
        estado_control = CALOR;
        while (estado_control < ALTO && temperatura >= umbral(estado_control))
            estado_control++;
    } else {
        /* Conservar estado dentro de la banda de histeresis.
         * Los bucles permiten saltos grandes sin pasar 5 s por cada nivel.
         */
        while (estado_control < ALTO &&
               temperatura >= umbral(estado_control) + HISTERESIS)
            estado_control++;
        while (estado_control > CALOR &&
               temperatura < umbral(estado_control - 1) - HISTERESIS)
            estado_control--;
    }
    if (estado_control == CALOR) PORTB |= (1 << CALEFACTOR_PIN);
    ventilador(estado_control >= BAJO ? estado_control - 1 : 0);
    enviar_medicion();
}

/* ---------------- MENU POR EL PUERTO SERIE ---------------- */

void mostrar_rangos(void)
{
    serie_texto("\r\nPunto medio: "); serie_numero(punto_medio);
    serie_texto(" C\r\nRango ideal nominal: "); serie_numero(punto_medio - 6);
    serie_texto(" a "); serie_numero(punto_medio + 3);
    serie_texto(" C\r\nVentilador bajo desde "); serie_numero(punto_medio + 4);
    serie_texto(" C; medio desde "); serie_numero(punto_medio + 14);
    serie_texto(" C; alto desde "); serie_numero(punto_medio + 24);
    serie_texto(" C.\r\nHisteresis: 1 C a cada lado del umbral.\r\n");
}

void mostrar_menu(void)
{
    serie_texto("\r\n--- MENU DE TEMPERATURA ---\r\n");
    serie_texto("Escribir punto medio (8 a 25) y pulsar Enter.\r\n");
    serie_texto("Proteccion DHT11: PM 8..25 mantiene todos los estados medibles.\r\n");
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
            serie_texto("Valor invalido. Escribir un entero de 8 a 25.\r\n");
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
        estado_control = SIN_LECTURA;
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
    ventilador_pwm_iniciar();
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
