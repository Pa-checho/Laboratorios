/*
 * Problema B: control de iluminacion con persiana motorizada.
 * Bloque 3: control del motor por PWM con limites de recorrido.
 * ATmega328P, 16 MHz. Compilar como C con optimizacion -Os.
 * LDR en A0, potenciometro de posicion en A1.
 * L298: ENA=D9, IN1=D4, IN2=D5.
 * Monitor Serie: 9600 baudios, 8N1.
 * Lecturas cada 50 ms; diagnostico cada 100 ms.
 * Enviar un setpoint de 0 a 1023 seguido de Enter.
 * Se detiene al alcanzar la banda de ambos sensores o un tope de recorrido.
 */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <stdint.h>

#define LDR_ADC_CHANNEL 0u
#define POT_ADC_CHANNEL 1u
#define MOTOR_IN1 PD4
#define MOTOR_IN2 PD5
#define MOTOR_PWM PB1
#define SETPOINT_INICIAL 512u
#define MOTOR_ABRE_IN1 1u
#define POT_ADC_SUBE_AL_ABRIR 1u
#define BANDA_MUERTA 18
#define PWM_MINIMO 110u
#define PWM_MAXIMO 170u
#define POSICION_MINIMA 25u
#define POSICION_MAXIMA 1000u
#define UART_BUFFER_TAM 32u
#define UART_BUFFER_MASK (UART_BUFFER_TAM - 1u)

uint16_t EEMEM eeprom_setpoint;
static uint16_t setpoint = SETPOINT_INICIAL;
static volatile uint8_t uart_buffer[UART_BUFFER_TAM];
static volatile uint8_t uart_escritura;
static volatile uint8_t uart_lectura;
static volatile uint8_t uart_error;

static volatile uint32_t reloj_ms;
static uint16_t lectura_ldr;
static uint16_t posicion_persiana;
static uint8_t pwm_aplicado;
static const char *sentido_actual = "PARADO";


ISR(TIMER0_COMPA_vect)
{
    reloj_ms++;
}

ISR(USART_RX_vect)
{
    uint8_t estado = UCSR0A;
    uint8_t dato = UDR0;
    uint8_t siguiente = (uint8_t)((uart_escritura + 1u) & UART_BUFFER_MASK);

    if ((estado & (_BV(FE0) | _BV(DOR0) | _BV(UPE0))) ||
        siguiente == uart_lectura)
    {
        uart_error = 1;
    }
    else
    {
        uart_buffer[uart_escritura] = dato;
        uart_escritura = siguiente;
    }
}

static uint32_t tiempo_actual_ms(void)
{
    uint32_t ahora;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        ahora = reloj_ms;
    }

    return ahora;
}

static void temporizador_iniciar(void)
{
    TCCR0A = _BV(WGM01);
    TCCR0B = 0;

    TCNT0 = 0;

    OCR0A = 249;

    TIFR0 = _BV(OCF0A);

    TIMSK0 = _BV(OCIE0A);

    TCCR0B =
        _BV(CS01)
        |
        _BV(CS00);
}

static void adc_iniciar(void)
{
    ADMUX = _BV(REFS0);

    ADCSRA =
        _BV(ADEN)
        |
        _BV(ADPS2)
        |
        _BV(ADPS1)
        |
        _BV(ADPS0);

    DIDR0 =
        _BV(ADC0D)
        |
        _BV(ADC1D);
}

static uint16_t adc_leer(uint8_t canal)
{
    ADMUX =
        (uint8_t)(
            _BV(REFS0)
            |
            (canal & 0x0Fu)
        );

    ADCSRA |= _BV(ADSC);

    while (ADCSRA & _BV(ADSC))
    {
    }

    return ADC;
}

static uint16_t adc_promedio(uint8_t canal)
{
    uint32_t suma = 0;

    (void)adc_leer(canal);

    for (uint8_t i = 0; i < 8; i++)
    {
        suma += adc_leer(canal);
    }

    return (uint16_t)(suma / 8u);
}

static void uart_iniciar(void)
{
    uint16_t divisor =
        (uint16_t)(
            F_CPU / (16UL * 9600UL) - 1UL
        );

    UCSR0A = 0;

    UBRR0H =
        (uint8_t)(divisor >> 8);

    UBRR0L =
        (uint8_t)divisor;

    UCSR0C =
        _BV(UCSZ01)
        |
        _BV(UCSZ00);

    UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);
}

static void uart_enviar_caracter(char caracter)
{
    while (!(UCSR0A & _BV(UDRE0)))
    {
    }

    UDR0 = (uint8_t)caracter;
}

static void uart_enviar_texto(const char *texto)
{
    while (*texto)
    {
        uart_enviar_caracter(*texto++);
    }
}

static void uart_enviar_numero(uint16_t numero)
{
    char digitos[5];

    uint8_t cantidad = 0;

    do
    {
        digitos[cantidad++] =
            (char)(
                '0'
                +
                (numero % 10u)
            );

        numero /= 10u;

    }
    while (
        numero > 0
        &&
        cantidad < sizeof(digitos)
    );

    while (cantidad > 0)
    {
        uart_enviar_caracter(
            digitos[--cantidad]
        );
    }
}

static void setpoint_cargar_eeprom(void)
{
    uint16_t guardado;

    guardado =
        eeprom_read_word(
            &eeprom_setpoint
        );

    if (guardado <= 1023u)
    {
        setpoint = guardado;
    }
    else
    {
        setpoint = SETPOINT_INICIAL;

        eeprom_update_word(
            &eeprom_setpoint,
            setpoint
        );
    }
}

static void setpoint_guardar_eeprom(void)
{
    eeprom_update_word(
        &eeprom_setpoint,
        setpoint
    );
}

static void uart_procesar_caracter(
    char caracter
)
{
    static char digitos[5];

    static uint8_t cantidad;

    static uint8_t invalido;

    uint32_t nuevo_setpoint = 0; /* Evita desbordamiento con cinco digitos. */


    if (
        caracter == '\r'
        ||
        caracter == '\n'
    )
    {
        if (
            cantidad == 0
            &&
            !invalido
        )
        {
            return;
        }


        if (
            !invalido
            &&
            cantidad > 0
        )
        {
            for (
                uint8_t i = 0;
                i < cantidad;
                i++
            )
            {
                nuevo_setpoint =
                    (uint32_t)(
                        nuevo_setpoint * 10UL
                        +
                        (uint8_t)(
                            digitos[i] - '0'
                        )
                    );
            }


            if (nuevo_setpoint <= 1023u)
            {
                setpoint =
                    nuevo_setpoint;


                setpoint_guardar_eeprom();


                uart_enviar_texto(
                    "Setpoint actualizado y guardado: "
                );


                uart_enviar_numero(
                    setpoint
                );


                uart_enviar_texto(
                    "\r\n"
                );
            }
            else
            {
                uart_enviar_texto(
                    "Error: ingresar un entero de 0 a 1023.\r\n"
                );
            }
        }
        else
        {
            uart_enviar_texto(
                "Error: ingresar un entero de 0 a 1023.\r\n"
            );
        }


        cantidad = 0;

        invalido = 0;

        return;
    }


    if (
        caracter == ' '
        ||
        caracter == '\t'
    )
    {
        return;
    }


    if (
        caracter >= '0'
        &&
        caracter <= '9'
        &&
        cantidad < sizeof(digitos)
    )
    {
        digitos[cantidad++] =
            caracter;
    }
    else
    {
        invalido = 1;
    }
}

/* Extrae cada caracter y comprueba errores antes de procesar la entrada. */
static void uart_atender_entrada(void)
{
    for (;;)
    {
        uint8_t error = 0;
        uint8_t disponible = 0;
        char caracter = 0;

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            if (uart_error)
            {
                uart_lectura = uart_escritura;
                uart_error = 0;
                error = 1;
            }
            else if (uart_lectura != uart_escritura)
            {
                caracter = (char)uart_buffer[uart_lectura];
                uart_lectura = (uint8_t)((uart_lectura + 1u) & UART_BUFFER_MASK);
                disponible = 1;
            }
        }

        if (error)
        {
            /* Marca la linea incompleta como invalida hasta recibir Enter. */
            uart_procesar_caracter('\0');
            uart_enviar_texto("Error UART; presione Enter y reenvie el setpoint.\r\n");
            return;
        }
        if (!disponible) return;
        uart_procesar_caracter(caracter);
    }
}

/* Control del motor y limites de recorrido. */
static void pwm_iniciar(void)
{
    DDRB |= _BV(MOTOR_PWM);

    TCCR1A =
        _BV(COM1A1)
        |
        _BV(WGM10);

    TCCR1B =
        _BV(WGM12)
        |
        _BV(CS11)
        |
        _BV(CS10);

    OCR1A = 0;
}

static void motor_parar(void)
{
    OCR1A = 0;

    PORTD &=
        (uint8_t)~(
            _BV(MOTOR_IN1)
            |
            _BV(MOTOR_IN2)
        );

    pwm_aplicado = 0;

    sentido_actual = "PARADO";
}

static uint8_t posicion_en_tope(
    uint8_t aumentar_luz
)
{
    uint8_t abrir =
        aumentar_luz;

    uint8_t adc_subir_al_abrir =
        POT_ADC_SUBE_AL_ABRIR;

    if (abrir)
    {
        if (adc_subir_al_abrir)
        {
            return
                posicion_persiana
                >=
                POSICION_MAXIMA;
        }

        return
            posicion_persiana
            <=
            POSICION_MINIMA;
    }

    if (adc_subir_al_abrir)
    {
        return
            posicion_persiana
            <=
            POSICION_MINIMA;
    }

    return
        posicion_persiana
        >=
        POSICION_MAXIMA;
}

static void motor_mover(
    uint8_t aumentar_luz,
    uint8_t pwm
)
{
    uint8_t in1_alto;

    if (
        posicion_en_tope(
            aumentar_luz
        )
    )
    {
        motor_parar();

        sentido_actual = "TOPE";

        return;
    }

    in1_alto =
        aumentar_luz
        ?
        MOTOR_ABRE_IN1
        :
        !MOTOR_ABRE_IN1;

    if (in1_alto)
    {
        PORTD |=
            _BV(MOTOR_IN1);

        PORTD &=
            (uint8_t)~_BV(MOTOR_IN2);
    }
    else
    {
        PORTD &=
            (uint8_t)~_BV(MOTOR_IN1);

        PORTD |=
            _BV(MOTOR_IN2);
    }

    OCR1A = pwm;

    pwm_aplicado = pwm;

    sentido_actual =
        aumentar_luz
        ?
        "ABRIR"
        :
        "CERRAR";
}

static void controlar_luz(void)
{
    /*
     * Ahora tenemos DOS errores.
     *
     * Error de iluminacion:
     * setpoint - LDR
     *
     * Error de posicion:
     * setpoint - potenciometro
     */

    int16_t error_ldr =
        (int16_t)setpoint
        -
        (int16_t)lectura_ldr;


    int16_t error_posicion =
        (int16_t)setpoint
        -
        (int16_t)posicion_persiana;


    /*
     * Comprobamos por separado si LDR y POT
     * estan dentro de la banda muerta.
     */

    uint8_t ldr_en_setpoint =
        (
            error_ldr >= -BANDA_MUERTA
            &&
            error_ldr <= BANDA_MUERTA
        );


    uint8_t posicion_en_setpoint =
        (
            error_posicion >= -BANDA_MUERTA
            &&
            error_posicion <= BANDA_MUERTA
        );


    /*
     * IMPORTANTE:
     *
     * El motor solamente se detiene si
     * LOS DOS llegaron al setpoint.
     *
     * LDR OK + POT OK = PARAR
     */

    if (
        ldr_en_setpoint
        &&
        posicion_en_setpoint
    )
    {
        motor_parar();

        return;
    }


    /*
     * Elegimos que error utilizar para controlar
     * el movimiento del motor.
     *
     * Si uno de los sensores ya llego,
     * usamos el otro para seguir corrigiendo.
     */

    int16_t error_control;


    if (ldr_en_setpoint && !posicion_en_setpoint)
    {
        /*
         * LDR ya llego.
         * Falta acomodar posicion.
         */
        error_control =
            error_posicion;
    }

    else if (!ldr_en_setpoint && posicion_en_setpoint)
    {
        /*
         * Posicion ya llego.
         * Falta acomodar iluminacion.
         */
        error_control =
            error_ldr;
    }

    else
    {
        /*
         * Ninguno llego.
         *
         * Se utiliza un promedio de ambos errores
         * para decidir hacia donde mover la persiana.
         */

        error_control =
            (int16_t)(
                (
                    (int32_t)error_ldr
                    +
                    (int32_t)error_posicion
                )
                /
                2
            );
    }


    /*
     * Si los errores estan en sentidos contrarios
     * y el promedio queda exactamente en cero,
     * se prioriza el error mas grande.
     */

    if (error_control == 0)
    {
        int16_t absoluto_ldr =
            error_ldr >= 0
            ?
            error_ldr
            :
            -error_ldr;


        int16_t absoluto_posicion =
            error_posicion >= 0
            ?
            error_posicion
            :
            -error_posicion;


        if (absoluto_ldr >= absoluto_posicion)
        {
            error_control =
                error_ldr;
        }
        else
        {
            error_control =
                error_posicion;
        }
    }


    /*
     * Calculamos magnitud del error.
     */

    uint16_t magnitud =
        (uint16_t)(
            error_control > 0
            ?
            error_control
            :
            -error_control
        );


    /*
     * Calculamos PWM proporcional al error.
     */

    uint16_t pwm =
        PWM_MINIMO
        +
        (uint16_t)(
            (
                (uint32_t)magnitud
                *
                (PWM_MAXIMO - PWM_MINIMO)
            )
            /
            1023u
        );


    if (pwm > PWM_MAXIMO)
    {
        pwm = PWM_MAXIMO;
    }


    /*
     * Error positivo:
     * falta aumentar -> ABRIR.
     *
     * Error negativo:
     * sobra -> CERRAR.
     */

    motor_mover(
        error_control > 0,
        (uint8_t)pwm
    );
}

/* Diagnostico del control de iluminacion. */
static void enviar_diagnostico(void)
{
    uart_enviar_texto("SP=");
    uart_enviar_numero(setpoint);
    uart_enviar_texto(",");
    uart_enviar_texto("LDR=");
    uart_enviar_numero(lectura_ldr);
    uart_enviar_texto(",POS=");
    uart_enviar_numero(posicion_persiana);
    uart_enviar_texto(",PWM=");
    uart_enviar_numero(pwm_aplicado);
    uart_enviar_texto(",DIR=");
    uart_enviar_texto(sentido_actual);
    uart_enviar_texto("\r\n");
}

int main(void)
{
    uint32_t siguiente_control = 0;
    uint32_t siguiente_diagnostico = 0;

    DDRC &= (uint8_t)~(_BV(PC0) | _BV(PC1));
    PORTC &= (uint8_t)~(_BV(PC0) | _BV(PC1));

    /* Inicializa ENA y las entradas del puente en nivel bajo. */
    TCCR1A = 0;
    TCCR1B = 0;
    PORTB &= (uint8_t)~_BV(MOTOR_PWM);
    DDRB |= _BV(MOTOR_PWM);
    PORTD &= (uint8_t)~(_BV(MOTOR_IN1) | _BV(MOTOR_IN2));
    DDRD |= _BV(MOTOR_IN1) | _BV(MOTOR_IN2);

    adc_iniciar();
    uart_iniciar();
    pwm_iniciar();
    motor_parar();
    temporizador_iniciar();
    setpoint_cargar_eeprom();
    sei();

    uart_enviar_texto("Setpoint cargado desde EEPROM: ");
    uart_enviar_numero(setpoint);
    uart_enviar_texto("\r\nEnviar setpoint ADC de 0 a 1023 y Enter.\r\n");
    uart_enviar_texto("Control de iluminacion listo.\r\n");

    for (;;)
    {
        uart_atender_entrada();
        uint32_t ahora = tiempo_actual_ms();

        if ((int32_t)(ahora - siguiente_control) >= 0)
        {
            lectura_ldr = adc_promedio(LDR_ADC_CHANNEL);
            posicion_persiana = adc_promedio(POT_ADC_CHANNEL);
            controlar_luz();
            siguiente_control = ahora + 50u;
        }

        if ((int32_t)(ahora - siguiente_diagnostico) >= 0)
        {
            enviar_diagnostico();
            siguiente_diagnostico = ahora + 100u;
        }
    }
}
