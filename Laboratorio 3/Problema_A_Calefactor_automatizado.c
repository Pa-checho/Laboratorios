/* Etapa 4: Medir con DHT11 y controlar los LED.
 * C para ATmega328P / Arduino Uno R3 a 16 MHz.
 * Cada archivo es un programa completo: compilar SOLO una etapa.
 * LED: D4/PD4, D5/PD5, D6/PD6 y calefactor D8/PB0.
 * Cada LED: pin -> resistencia 330 ohm -> anodo; catodo -> GND.
 */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#define CALEFACTOR_PIN PB0
#define LED_1 PD4
#define LED_2 PD5
#define LED_3 PD6

/* DATA del DHT11: D2, resistencia de 4,7 kohm a 5 V. */
#define DHT_PIN PD2
int16_t punto_medio = 22; /* Todavia fijo; el menu se agrega en la etapa 6. */
int16_t temperatura = -2; /* -2: esperando; -1: fallo del sensor. */

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

/* Poner a cero antes de configurar evita encendidos al iniciar. */
void salidas_iniciar(void)
{
    PORTB &= ~(1 << CALEFACTOR_PIN);
    DDRB |= (1 << CALEFACTOR_PIN);
    leds_iniciar();
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

/* La interrupcion avisa; el trabajo se hace en main. */
volatile uint8_t medir = 0;

ISR(TIMER1_COMPA_vect)
{
    static uint8_t segundos = 0;
    segundos++;
    if (segundos == 5) {
        segundos = 0;
        medir = 1;
    }
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

/* Esperas limitadas: un sensor ausente no bloquea el programa. */
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

/* Recibe 40 bits y verifica la suma. Usa la temperatura entera. */
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

int main(void)
{
    salidas_iniciar();
    DDRD &= ~(1 << DHT_PIN);
    serie_iniciar();
    serie_texto("Punto medio fijo: 22 C. Primera lectura en 5 s.\r\n");
    temporizador_iniciar();
    sei();
    while (1) {
        if (medir) {
            medir = 0;
            cli(); /* Evita que una interrupcion altere los pulsos del DHT11. */
            temperatura = dht11_leer();
            sei();
            controlar_temperatura();
        }
    }
}
