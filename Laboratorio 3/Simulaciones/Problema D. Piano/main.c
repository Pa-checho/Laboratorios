#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdint.h>

/*
  ARDUINO UNO - ATmega328P - 16 MHz

  Pulsadores conectados a GND:
  Do: D2
  Re: D3
  Mi: D4
  Fa: D5
  Sol: D6
  La: D7
  Si: D8
  Do agudo: D10

  Buzzers mediante transistores:
  Buzzer 1: D9
  Buzzer 2: D11

  LCD 16x2 con modulo I2C PCF8574:
  GND = GND
  VCC = 5V
  SDA = A4 (PC4 / SDA)
  SCL = A5 (PC5 / SCL)

  Direccion I2C asumida: 0x27
  Mapeo PCF8574 asumido:
  P0=RS, P1=RW, P2=E, P3=Backlight, P4-P7=D4-D7

  Monitor Serie: 9600 baudios.

  C1: Megalovania
  C2: Dragonborn
  C3: Bloody Tears
  C4: Game of Thrones
  P: detener y volver al piano

  Programa con main() propio.
*/

#define SIN_NOTA 255u
#define SILENCIO 0u
#define SOSTENER 255u

#define PASO_MS 125u
#define C1_PASOS 160u

#define C2_PASOS 144u
#define C2_PASO_MS 125u

#define C3_PASOS 256u
#define C3_PASO_MS 125u

#define C4_PASOS 552u
#define C4_PASO_MS 176u

/*
  Cada pareja contiene las notas de los dos buzzers.
  0: silencio.
  255: prolongar la nota anterior.
  Otros valores: notas MIDI.
*/

// C4: Game of Thrones.
// 92 compases, aproximadamente 170 BPM.
// Segunda voz en los compases 73 a 88.
static const uint8_t cancion_c4[C4_PASOS][2] PROGMEM = {
    {69,0},{255,0},{62,0},{255,0},{65,0},{67,0}, // 1
    {69,0},{255,0},{62,0},{255,0},{65,0},{67,0}, // 2
    {69,0},{255,0},{62,0},{255,0},{65,0},{67,0}, // 3
    {69,0},{255,0},{62,0},{255,0},{65,0},{67,0}, // 4
    {69,0},{255,0},{62,0},{255,0},{66,0},{67,0}, // 5
    {69,0},{255,0},{62,0},{255,0},{66,0},{67,0}, // 6
    {69,0},{255,0},{62,0},{255,0},{66,0},{67,0}, // 7
    {69,0},{255,0},{62,0},{255,0},{66,0},{67,0}, // 8
    {69,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 9
    {255,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 10
    {0,0},{0,0},{0,0},{0,0},{0,0},{0,0}, // 11
    {0,0},{0,0},{0,0},{0,0},{0,0},{0,0}, // 12
    {64,0},{255,0},{57,0},{255,0},{60,0},{62,0}, // 13
    {64,0},{255,0},{57,0},{255,0},{60,0},{62,0}, // 14
    {64,0},{255,0},{57,0},{255,0},{60,0},{62,0}, // 15
    {64,0},{255,0},{57,0},{255,0},{60,0},{255,0}, // 16
    {0,0},{0,0},{0,0},{0,0},{0,0},{0,0}, // 17
    {0,0},{0,0},{0,0},{0,0},{0,0},{0,0}, // 18
    {0,0},{0,0},{0,0},{0,0},{0,0},{0,0}, // 19
    {0,0},{0,0},{0,0},{0,0},{0,0},{0,0}, // 20
    {62,0},{255,0},{55,0},{255,0},{58,0},{60,0}, // 21
    {62,0},{255,0},{55,0},{255,0},{58,0},{60,0}, // 22
    {62,0},{255,0},{55,0},{255,0},{58,0},{60,0}, // 23
    {62,0},{255,0},{55,0},{255,0},{0,0},{0,0}, // 24
    {69,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 25
    {62,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 26
    {65,0},{67,0},{69,0},{255,0},{255,0},{255,0}, // 27
    {62,0},{255,0},{255,0},{255,0},{65,0},{67,0}, // 28
    {64,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 29
    {255,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 30
    {255,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 31
    {255,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 32
    {67,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 33
    {60,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 34
    {65,0},{64,0},{67,0},{255,0},{255,0},{255,0}, // 35
    {60,0},{255,0},{255,0},{255,0},{65,0},{64,0}, // 36
    {62,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 37
    {255,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 38
    {255,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 39
    {255,0},{255,0},{69,0},{255,0},{74,0},{255,0}, // 40
    {81,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 41
    {74,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 42
    {77,0},{79,0},{81,0},{255,0},{255,0},{255,0}, // 43
    {74,0},{255,0},{255,0},{255,0},{77,0},{79,0}, // 44
    {76,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 45
    {255,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 46
    {255,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 47
    {255,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 48
    {79,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 49
    {72,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 50
    {76,0},{255,0},{255,0},{77,0},{255,0},{255,0}, // 51
    {76,0},{255,0},{255,0},{72,0},{255,0},{255,0}, // 52
    {74,0},{255,0},{255,0},{255,0},{255,0},{255,0}, // 53
    {255,0},{255,0},{255,0},{255,0},{82,0},{84,0}, // 54
    {86,0},{255,0},{81,0},{255,0},{82,0},{84,0}, // 55
    {86,0},{255,0},{81,0},{255,0},{82,0},{84,0}, // 56
    {74,0},{255,0},{65,0},{255,0},{69,0},{70,0}, // 57
    {74,0},{255,0},{65,0},{255,0},{72,0},{74,0}, // 58
    {72,0},{255,0},{65,0},{255,0},{69,0},{70,0}, // 59
    {72,0},{255,0},{65,0},{255,0},{69,0},{255,0}, // 60
    {70,0},{255,0},{62,0},{255,0},{67,0},{69,0}, // 61
    {70,0},{255,0},{62,0},{255,0},{67,0},{69,0}, // 62
    {69,0},{255,0},{62,0},{255,0},{67,0},{69,0}, // 63
    {69,0},{255,0},{62,0},{255,0},{67,0},{69,0}, // 64
    {65,0},{255,0},{58,0},{255,0},{62,0},{64,0}, // 65
    {65,0},{255,0},{58,0},{255,0},{64,0},{65,0}, // 66
    {65,0},{255,0},{58,0},{255,0},{65,0},{255,0}, // 67
    {67,0},{255,0},{60,0},{255,0},{67,0},{255,0}, // 68
    {62,0},{255,0},{55,0},{255,0},{58,0},{60,0}, // 69
    {62,0},{255,0},{55,0},{255,0},{58,0},{60,0}, // 70
    {62,0},{255,0},{55,0},{255,0},{58,0},{60,0}, // 71
    {62,0},{255,0},{255,0},{255,0},{0,0},{0,0}, // 72
    {86,74},{255,255},{255,255},{255,255},{255,255},{255,255}, // 73
    {255,255},{255,255},{255,255},{255,255},{255,255},{255,255}, // 74
    {84,72},{255,255},{255,255},{255,255},{255,255},{255,255}, // 75
    {255,255},{255,255},{255,255},{255,255},{255,255},{255,255}, // 76
    {82,70},{74,62},{255,255},{255,255},{255,255},{255,255}, // 77
    {255,255},{255,255},{255,255},{255,255},{255,255},{255,255}, // 78
    {81,69},{255,255},{255,255},{255,255},{255,255},{255,255}, // 79
    {255,255},{255,255},{255,255},{255,255},{255,255},{255,255}, // 80
    {70,58},{255,255},{255,255},{255,255},{255,255},{255,255}, // 81
    {255,255},{255,255},{255,255},{255,255},{255,255},{255,255}, // 82
    {77,65},{255,255},{255,255},{255,255},{255,255},{255,255}, // 83
    {76,64},{255,255},{255,255},{255,255},{255,255},{255,255}, // 84
    {74,62},{255,255},{255,255},{255,255},{82,70},{84,72}, // 85
    {86,74},{255,255},{81,69},{255,255},{82,70},{84,72}, // 86
    {86,74},{255,255},{81,69},{255,255},{82,70},{84,72}, // 87
    {86,74},{255,255},{81,69},{255,255},{82,70},{84,72}, // 88
    {0,0},{0,0},{0,0},{0,0},{70,0},{72,0}, // 89
    {74,0},{255,0},{69,0},{255,0},{70,0},{72,0}, // 90
    {74,0},{255,0},{69,0},{255,0},{70,0},{72,0}, // 91
    {74,0},{255,0},{255,0},{255,0},{0,0},{0,0} // 92
};

// C1: Megalovania.
static const uint8_t cancion_c1[C1_PASOS][2] PROGMEM = {
    {0,50},{0,50},{62,0},{0,0},{57,0},{0,0},{0,0},{56,0},
    {0,0},{55,0},{0,0},{53,0},{255,0},{50,0},{53,0},{55,0},
    {0,48},{0,48},{62,0},{0,0},{57,0},{0,0},{0,0},{56,0},
    {0,0},{55,0},{0,0},{53,0},{255,0},{50,0},{53,0},{55,0},
    {0,47},{0,47},{62,0},{0,0},{57,0},{0,0},{0,0},{56,0},
    {0,0},{55,0},{0,0},{53,0},{255,0},{50,0},{53,0},{55,0},
    {0,46},{0,46},{62,0},{0,0},{57,0},{0,0},{0,0},{56,0},
    {0,0},{55,0},{0,0},{53,0},{255,0},{50,0},{53,0},{55,0},
    {0,38},{0,50},{62,38},{0,255},{57,38},{0,38},{0,0},{56,38},
    {0,0},{55,38},{0,0},{53,38},{255,38},{50,38},{53,38},{55,255},
    {0,36},{0,48},{62,36},{0,255},{57,36},{0,36},{0,0},{56,36},
    {0,0},{55,36},{0,0},{53,36},{255,36},{50,36},{53,36},{55,255},
    {0,35},{0,47},{62,35},{0,255},{57,35},{0,35},{0,0},{56,35},
    {0,0},{55,35},{0,0},{53,35},{255,35},{50,35},{53,35},{55,255},
    {0,34},{0,46},{62,34},{0,255},{57,34},{0,34},{0,0},{56,34},
    {0,0},{55,36},{0,0},{53,36},{255,36},{50,36},{53,36},{55,255},
    {62,26},{62,255},{74,26},{0,255},{69,26},{0,26},{0,0},{68,26},
    {0,0},{67,26},{0,0},{65,26},{255,26},{62,255},{65,26},{67,255},
    {60,24},{60,255},{74,24},{0,255},{69,24},{0,24},{0,0},{68,24},
    {0,0},{67,24},{0,0},{65,24},{255,24},{62,255},{65,24},{67,255}
};

// C2: Dragonborn.
static const uint8_t cancion_c2[C2_PASOS][2] PROGMEM = {
    {86,47},{255,255},{255,255},{255,47},{85,255},{255,255},{255,47},{255,255},
    {83,255},{255,255},{255,47},{255,255},{81,43},{255,255},{255,255},{255,43},
    {79,255},{255,255},{255,43},{255,255},{78,255},{255,255},{255,43},{255,255},
    {76,40},{255,255},{255,255},{255,40},{255,40},{255,255},{255,40},{255,255},
    {74,40},{255,255},{78,40},{255,255},{76,33},{255,255},{255,255},{255,33},
    {255,33},{255,255},{255,33},{255,255},{255,33},{255,255},{86,33},{85,255},
    {86,47},{255,255},{255,35},{255,255},{86,255},{85,255},{86,47},{255,255},
    {255,35},{255,255},{86,255},{85,255},{88,45},{255,255},{86,33},{255,255},
    {85,255},{255,255},{83,47},{255,255},{255,35},{255,255},{83,255},{81,255},
    {83,43},{255,255},{255,31},{255,255},{83,255},{81,255},{83,40},{255,255},
    {255,28},{255,255},{81,255},{83,255},{85,45},{255,255},{86,33},{255,255},
    {81,255},{255,255},{83,47},{255,255},{255,35},{255,255},{83,255},{85,255},
    {86,47},{255,255},{86,35},{255,255},{86,255},{88,255},{90,43},{255,255},
    {255,31},{255,255},{85,255},{86,255},{88,45},{255,255},{86,33},{255,255},
    {85,255},{255,255},{83,47},{255,255},{255,35},{255,255},{83,255},{81,255},
    {83,43},{255,255},{255,31},{255,255},{83,255},{81,255},{83,40},{255,255},
    {255,28},{255,255},{81,255},{83,255},{85,45},{255,255},{86,33},{255,255},
    {81,255},{255,255},{83,47},{255,255},{255,255},{255,255},{255,47},{255,255}
};

// C3: Bloody Tears.
static const uint8_t cancion_c3[C3_PASOS][2] PROGMEM = {
    {70,46},{65,255},{77,255},{65,255},{75,53},{65,255},{73,255},{65,255},
    {72,46},{65,255},{73,255},{65,255},{72,53},{65,255},{70,255},{65,255},
    {72,41},{65,255},{73,255},{65,255},{75,48},{65,255},{73,255},{65,255},
    {72,41},{65,255},{68,255},{65,255},{72,48},{65,255},{70,255},{65,255},
    {70,46},{65,255},{77,255},{65,255},{75,53},{65,255},{73,255},{65,255},
    {72,46},{65,255},{73,255},{65,255},{72,53},{65,255},{70,255},{65,255},
    {72,41},{65,255},{73,255},{65,255},{75,48},{65,255},{73,255},{65,255},
    {72,41},{65,255},{68,255},{65,255},{72,48},{65,255},{70,255},{65,255},
    {75,46},{255,255},{80,255},{77,255},{255,53},{60,255},{58,255},{60,255},
    {61,46},{255,255},{63,255},{255,255},{75,53},{255,255},{73,255},{255,255},
    {75,44},{255,255},{255,255},{80,255},{255,51},{255,255},{77,255},{255,255},
    {255,44},{255,255},{255,255},{255,255},{75,51},{255,255},{73,255},{255,255},
    {75,42},{255,255},{80,255},{77,255},{255,49},{63,255},{61,255},{63,255},
    {65,42},{255,255},{66,255},{255,255},{75,49},{255,255},{77,255},{255,255},
    {78,41},{255,255},{255,255},{80,255},{255,48},{255,255},{255,255},{255,255},
    {77,41},{255,255},{255,255},{78,255},{255,48},{255,255},{255,255},{255,255},
    {75,46},{255,255},{80,255},{77,255},{255,53},{60,255},{58,255},{60,255},
    {61,46},{255,255},{63,255},{255,255},{75,53},{255,255},{73,255},{255,255},
    {75,44},{255,255},{255,255},{80,255},{255,51},{255,255},{77,255},{255,255},
    {255,44},{255,255},{255,255},{255,255},{75,51},{255,255},{73,255},{255,255},
    {75,42},{255,255},{80,255},{77,255},{255,49},{63,255},{61,255},{63,255},
    {65,42},{255,255},{66,255},{255,255},{75,49},{255,255},{77,255},{255,255},
    {78,41},{255,255},{255,255},{80,255},{255,48},{255,255},{255,255},{255,255},
    {77,41},{255,255},{79,255},{70,255},{81,48},{255,255},{84,255},{255,255},
    {72,46},{255,255},{255,255},{70,255},{255,53},{255,255},{82,255},{255,255},
    {72,44},{255,255},{255,255},{70,255},{255,51},{255,255},{82,255},{255,255},
    {72,42},{255,255},{255,255},{70,255},{255,49},{255,255},{82,255},{255,255},
    {73,44},{85,255},{72,255},{84,255},{70,51},{82,255},{68,255},{80,255},
    {72,46},{255,255},{70,255},{82,255},{255,53},{255,255},{255,255},{255,255},
    {72,44},{255,255},{70,255},{82,255},{255,51},{255,255},{255,255},{255,255},
    {72,42},{255,255},{70,255},{82,255},{255,49},{255,255},{255,255},{255,255},
    {85,44},{255,255},{87,255},{255,255},{84,51},{85,255},{0,0},{0,0}
};

// Tablas de frecuencias para notas MIDI 24 a 96.
static const uint16_t ocr1_por_nota[73] PROGMEM = {
    30577,28861,27241,25712,24269,22906,21621,20408,19262,18181,17160,16197,
    15288,14430,13620,12855,12134,11453,10810,10203,9630,9090,8580,8098,
    7644,7214,6810,6427,6066,5726,5404,5101,4815,4544,4289,4049,
    3821,3607,3404,3213,3033,2862,2702,2550,2407,2272,2144,2024,
    1910,1803,1702,1606,1516,1431,1350,1275,1203,1135,1072,1011,
    955,901,850,803,757,715,675,637,601,567,535,505,477
};

static const uint8_t ocr2_por_nota[73] PROGMEM = {
    238,224,212,200,189,178,168,158,149,141,133,126,
    118,112,105,99,94,88,83,79,74,70,66,252,
    238,224,212,200,189,178,168,158,149,141,133,252,
    238,224,212,200,189,178,168,158,149,141,133,252,
    238,224,212,200,189,178,168,158,149,141,133,252,
    238,224,212,200,189,178,168,158,149,141,133,126,118
};

static const uint8_t cs2_por_nota[73] PROGMEM = {
    7,7,7,7,7,7,7,7,7,7,7,7,
    7,7,7,7,7,7,7,7,7,7,7,6,
    6,6,6,6,6,6,6,6,6,6,6,5,
    5,5,5,5,5,5,5,5,5,5,5,4,
    4,4,4,4,4,4,4,4,4,4,4,3,
    3,3,3,3,3,3,3,3,3,3,3,3,3
};

// ================= LCD I2C =================

#define LCD_I2C_ADDR 0x27u

#define LCD_RS 0x01u
#define LCD_RW 0x02u
#define LCD_EN 0x04u
#define LCD_BL 0x08u

static void twi_init(void)
{
    // F_CPU = 16 MHz
    // SCL = 100 kHz con prescaler = 1:
    // TWBR = ((16 MHz / 100 kHz) - 16) / 2 = 72
    TWSR = 0;
    TWBR = 72;
    TWCR = _BV(TWEN);
}

static void twi_start(void)
{
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);

    while (!(TWCR & _BV(TWINT))) {
    }

    TWDR = (uint8_t)(LCD_I2C_ADDR << 1); // SLA + W
    TWCR = _BV(TWINT) | _BV(TWEN);

    while (!(TWCR & _BV(TWINT))) {
    }
}

static void twi_write(uint8_t dato)
{
    TWDR = dato;
    TWCR = _BV(TWINT) | _BV(TWEN);

    while (!(TWCR & _BV(TWINT))) {
    }
}

static void twi_stop(void)
{
    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
}

static void lcd_expander(uint8_t dato)
{
    twi_start();
    twi_write((uint8_t)(dato | LCD_BL));
    twi_stop();
}

static void lcd_pulso_enable(uint8_t dato)
{
    lcd_expander((uint8_t)(dato | LCD_EN));
    _delay_us(1);

    lcd_expander((uint8_t)(dato & (uint8_t)~LCD_EN));
    _delay_us(50);
}

static void lcd_nibble_i2c(uint8_t nibble, uint8_t es_texto)
{
    uint8_t dato = (uint8_t)((nibble & 0x0Fu) << 4);

    if (es_texto)
        dato |= LCD_RS;

    // RW siempre queda en 0: escritura.
    lcd_pulso_enable(dato);
}

static void lcd_byte(uint8_t dato, uint8_t es_texto)
{
    lcd_nibble_i2c((uint8_t)(dato >> 4), es_texto);
    lcd_nibble_i2c((uint8_t)(dato & 0x0Fu), es_texto);

    if (!es_texto && (dato == 0x01u || dato == 0x02u))
        _delay_ms(2);
}

static void lcd_init(void)
{
    twi_init();

    _delay_ms(50);

    // Secuencia de inicializacion HD44780 en modo 4 bits.
    lcd_nibble_i2c(0x03, 0);
    _delay_ms(5);

    lcd_nibble_i2c(0x03, 0);
    _delay_us(150);

    lcd_nibble_i2c(0x03, 0);
    _delay_us(150);

    lcd_nibble_i2c(0x02, 0);
    _delay_us(150);

    lcd_byte(0x28, 0); // 4 bits, 2 lineas, 5x8
    lcd_byte(0x08, 0); // display apagado
    lcd_byte(0x01, 0); // limpiar
    lcd_byte(0x06, 0); // incremento automatico
    lcd_byte(0x0C, 0); // display encendido, cursor apagado
}

static void lcd_linea(uint8_t fila, const char *texto)
{
    lcd_byte(fila ? 0xC0 : 0x80, 0);

    for (uint8_t i = 0; i < 16; ++i) {
        char c = ' ';

        if (*texto)
            c = *texto++;

        lcd_byte((uint8_t)c, 1);
    }
}

static void lcd_nota(uint8_t fila, uint8_t nota)
{
    static const char * const nombres[8] = {
        "Do4", "Re4", "Mi4", "Fa4",
        "Sol4", "La4", "Si4", "Do5"
    };

    char linea[17] = "B1:             ";
    linea[1] = fila ? '2' : '1';

    const char *nombre =
        (nota == SIN_NOTA) ? "--" : nombres[nota];

    for (uint8_t i = 4; i < 16 && *nombre; ++i)
        linea[i] = *nombre++;

    lcd_linea(fila, linea);
}

static void lcd_mostrar_notas(uint8_t nota1, uint8_t nota2)
{
    if (nota1 == SIN_NOTA && nota2 == SIN_NOTA) {
        lcd_linea(0, "Piano listo");
        lcd_linea(1, "Pulsa una tecla");
    } else {
        lcd_nota(0, nota1);
        lcd_nota(1, nota2);
    }
}

// ================= VARIABLES =================

static volatile uint32_t reloj_ms;

static volatile uint8_t rx_buffer[64];
static volatile uint8_t rx_escribir;
static volatile uint8_t rx_leer;
static volatile uint8_t rx_error;

static uint8_t modo_cancion;
static uint8_t esperando_numero;
static uint16_t paso_actual;
static uint16_t total_pasos;
static uint16_t paso_ms;
static uint8_t vueltas_restantes;

static const uint8_t (*partitura)[2];

static uint8_t voz_activa[2];
static uint32_t siguiente_paso;
static uint32_t fin_voz[2];

static uint8_t anterior1 = SIN_NOTA;
static uint8_t anterior2 = SIN_NOTA;

static volatile uint8_t cambio_teclas = 1;
static uint8_t candidata;
static uint8_t estable;
static uint8_t en_rebote = 1;
static uint32_t ultimo_cambio;

// ================= RELOJ =================

static void arrancar_reloj(void)
{
    if (TCCR0B == 0) {
        TCNT0 = 0;
        TIFR0 = _BV(OCF0A);
        TCCR0B = _BV(CS02);
    }
}

static void cambio_en_teclas(void)
{
    cambio_teclas = 1;
    arrancar_reloj();
}

ISR(PCINT0_vect)
{
    cambio_en_teclas();
}

ISR(PCINT2_vect)
{
    cambio_en_teclas();
}

ISR(TIMER0_COMPA_vect)
{
    reloj_ms += 4;
}

ISR(USART_RX_vect)
{
    uint8_t estado = UCSR0A;
    uint8_t dato = UDR0;
    uint8_t siguiente = (uint8_t)((rx_escribir + 1u) & 63u);

    if ((estado & (_BV(FE0) | _BV(DOR0) | _BV(UPE0))) ||
        siguiente == rx_leer) {
        rx_error = 1;
    } else {
        rx_buffer[rx_escribir] = dato;
        rx_escribir = siguiente;
    }
}

static uint32_t ahora_ms(void)
{
    uint32_t valor;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        valor = reloj_ms;
    }

    return valor;
}

static uint8_t vencido(uint32_t ahora, uint32_t plazo)
{
    return (int32_t)(ahora - plazo) >= 0;
}

// ================= UART =================

static void uart_texto(const char *texto)
{
    while (*texto) {
        while (!(UCSR0A & _BV(UDRE0))) {
        }

        UDR0 = (uint8_t)*texto++;
    }
}

static void uart_init(void)
{
    UCSR0A = 0;
    UBRR0H = 0;
    UBRR0L = 103;

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
    UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);
}

// ================= BUZZERS =================

static void tono(uint8_t voz, uint8_t midi)
{
    uint8_t valida = midi >= 24 && midi <= 96;
    uint8_t indice = valida ? (uint8_t)(midi - 24) : 0;

    if (voz == 0) {
        uint16_t valor =
            valida ? pgm_read_word(&ocr1_por_nota[indice]) : 0;

        TCCR1B = 0;
        TCCR1A = 0;
        PORTB &= (uint8_t)~_BV(PB1);
        TCNT1 = 0;

        if (valida) {
            OCR1A = valor;
            TCCR1A = _BV(COM1A0);
            TCCR1B = _BV(WGM12) | _BV(CS11);
        }
    } else {
        uint8_t valor =
            valida ? pgm_read_byte(&ocr2_por_nota[indice]) : 0;

        uint8_t prescaler =
            valida ? pgm_read_byte(&cs2_por_nota[indice]) : 0;

        TCCR2B = 0;
        TCCR2A = 0;
        PORTB &= (uint8_t)~_BV(PB3);
        TCNT2 = 0;

        if (valida) {
            OCR2A = valor;
            TCCR2A = _BV(WGM21) | _BV(COM2A0);
            TCCR2B = prescaler;
        }
    }
}

static void silencio(void)
{
    tono(0, SILENCIO);
    tono(1, SILENCIO);
    voz_activa[0] = voz_activa[1] = 0;
}

// ================= MODOS =================

static void volver_al_piano(void)
{
    modo_cancion = 0;
    silencio();

    anterior1 = anterior2 = SIN_NOTA;
    candidata = estable = 0;
    en_rebote = 1;
    ultimo_cambio = ahora_ms();
    cambio_teclas = 1;

    arrancar_reloj();
    lcd_mostrar_notas(SIN_NOTA, SIN_NOTA);
}

static void iniciar_cancion(uint8_t numero)
{
    silencio();
    vueltas_restantes = 1;

    if (numero == 1) {
        partitura = cancion_c1;
        total_pasos = C1_PASOS;
        paso_ms = PASO_MS;

        lcd_linea(0, "C1: Megalovania");
        uart_texto(
            "\r\nC1: Megalovania (20 s). P: detener.\r\n"
        );

    } else if (numero == 2) {
        partitura = cancion_c2;
        total_pasos = C2_PASOS;
        paso_ms = C2_PASO_MS;

        lcd_linea(0, "C2: Dragonborn");
        uart_texto(
            "\r\nC2: Dragonborn (18 s, 120 BPM). P: detener.\r\n"
        );
    }

    if (numero == 3) {
        partitura = cancion_c3;
        total_pasos = C3_PASOS;
        paso_ms = C3_PASO_MS;
        vueltas_restantes = 2;

        lcd_linea(0, "C3: Bloody Tears");
        uart_texto(
            "\r\nC3: Bloody Tears (64 s, 120 BPM). P: detener.\r\n"
        );
    }

    if (numero == 4) {
        partitura = cancion_c4;
        total_pasos = C4_PASOS;
        paso_ms = C4_PASO_MS;

        lcd_linea(0, "C4: Game Thrones");
        uart_texto(
            "\r\nC4: Game of Thrones (97 s, 170 BPM). P: detener.\r\n"
        );
    }

    lcd_linea(1, "P: volver piano");
    paso_actual = 0;
    arrancar_reloj();
    siguiente_paso = ahora_ms();
    modo_cancion = numero;
}

static void recibir_comandos(void)
{
    if (rx_error) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            rx_leer = rx_escribir;
            rx_error = 0;
        }

        esperando_numero = 0;
        uart_texto("\r\nError UART. Reenviar comando.\r\n");
    }

    while (rx_leer != rx_escribir) {
        uint8_t c = rx_buffer[rx_leer];
        rx_leer = (uint8_t)((rx_leer + 1u) & 63u);

        if (c >= 'a' && c <= 'z')
            c = (uint8_t)(c - 'a' + 'A');

        if (c == '\r' || c == '\n') {
            esperando_numero = 0;
            continue;
        }

        if (c == ' ' || c == '\t')
            continue;

        if (c == 'P') {
            esperando_numero = 0;
            volver_al_piano();
            uart_texto("\r\nModo piano.\r\n");

        } else if (c == 'C') {
            esperando_numero = 1;

        } else if (esperando_numero && c >= '1' && c <= '4') {
            esperando_numero = 0;
            iniciar_cancion((uint8_t)(c - '0'));

        } else {
            esperando_numero = 0;
            uart_texto("\r\nComandos: C1, C2, C3, C4, P.\r\n");
        }
    }
}

// ================= REPRODUCCION =================

static void actualizar_cancion(uint32_t ahora)
{
    while (modo_cancion && vencido(ahora, siguiente_paso)) {
        if (paso_actual >= total_pasos) {
            if (vueltas_restantes > 1) {
                --vueltas_restantes;
                paso_actual = 0;
            } else {
                volver_al_piano();
                uart_texto("\r\nFin de cancion. Modo piano.\r\n");
                return;
            }
        }

        for (uint8_t voz = 0; voz < 2; ++voz) {
            uint8_t nota =
                pgm_read_byte(&partitura[paso_actual][voz]);

            if (nota == SOSTENER)
                continue;

            tono(voz, nota);
            voz_activa[voz] = nota != SILENCIO;

            if (voz_activa[voz]) {
                uint16_t duracion = 1;

                while (
                    paso_actual + duracion < total_pasos &&
                    pgm_read_byte(
                        &partitura[paso_actual + duracion][voz]
                    ) == SOSTENER
                ) {
                    ++duracion;
                }

                fin_voz[voz] =
                    siguiente_paso +
                    (uint32_t)duracion * paso_ms - 12u;
            }
        }

        ++paso_actual;
        siguiente_paso += paso_ms;
    }

    for (uint8_t voz = 0; voz < 2; ++voz) {
        if (voz_activa[voz] && vencido(ahora, fin_voz[voz])) {
            tono(voz, SILENCIO);
            voz_activa[voz] = 0;
        }
    }
}

// ================= TECLADO CORREGIDO =================

static uint8_t leer_teclas(void)
{
    uint8_t d = (uint8_t)~PIND;
    uint8_t b = (uint8_t)~PINB;

    uint8_t teclas = (uint8_t)((d >> 2) & 0x3Fu);

    if (b & _BV(PB0))
        teclas |= _BV(6);

    if (b & _BV(PB2))
        teclas |= _BV(7);

    return teclas;
}

static void actualizar_teclado(uint32_t ahora)
{
    if (modo_cancion)
        return;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        cambio_teclas = 0;
    }

    // Lectura continua: no depende de PCINT.
    uint8_t lectura = leer_teclas();

    if (lectura != candidata) {
        candidata = lectura;
        ultimo_cambio = ahora;
        en_rebote = 1;

    } else if ((uint32_t)(ahora - ultimo_cambio) >= 20) {
        estable = candidata;
        en_rebote = 0;
    }

    uint8_t n1 = SIN_NOTA;
    uint8_t n2 = SIN_NOTA;

    for (uint8_t i = 0; i < 8; ++i) {
        if (estable & (uint8_t)_BV(i)) {
            if (n1 == SIN_NOTA) {
                n1 = i;
            } else {
                n2 = i;
                break;
            }
        }
    }

    static const uint8_t midi_teclas[8] = {
        60, 62, 64, 65, 67, 69, 71, 72
    };

    uint8_t cambio = n1 != anterior1 || n2 != anterior2;

    if (n1 != anterior1) {
        tono(0, n1 == SIN_NOTA ? SILENCIO : midi_teclas[n1]);
        anterior1 = n1;
    }

    if (n2 != anterior2) {
        tono(1, n2 == SIN_NOTA ? SILENCIO : midi_teclas[n2]);
        anterior2 = n2;
    }

    if (cambio)
        lcd_mostrar_notas(n1, n2);
}

static void parar_reloj_si_estable(void)
{
    // Mantener siempre el reloj de 4 ms en funcionamiento.
    arrancar_reloj();
}

// ================= PROGRAMA PRINCIPAL =================

int main(void)
{
    // A4 (PC4/SDA) y A5 (PC5/SCL) quedan reservados para el LCD I2C.
    // Entradas D2 a D7 con pull-ups.
    DDRD &= (uint8_t)~0xFCu;
    PORTD |= 0xFCu;

    // Entradas D8 y D10 con pull-ups.
    DDRB &= (uint8_t)~(_BV(PB0) | _BV(PB2));
    PORTB |= _BV(PB0) | _BV(PB2);

    // Salidas D9 y D11.
    PORTB &= (uint8_t)~(_BV(PB1) | _BV(PB3));
    DDRB |= _BV(PB1) | _BV(PB3);

    silencio();
    lcd_init();
    uart_init();

    // Timer0: interrupcion cada 4 ms.
    TCCR0A = _BV(WGM01);
    OCR0A = 249;
    TCCR0B = 0;
    TIMSK0 = _BV(OCIE0A);

    // Interrupciones de los pulsadores.
    PCMSK0 = _BV(PCINT0) | _BV(PCINT2);
    PCMSK2 = 0xFC;
    PCIFR = _BV(PCIF0) | _BV(PCIF2);
    PCICR = _BV(PCIE0) | _BV(PCIE2);

    volver_al_piano();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sei();

    uart_texto(
        "\r\nPiano listo. 9600 baudios.\r\n"
        "C1: Megalovania; C2: Dragonborn; "
        "C3: Bloody Tears; C4: Game of Thrones; P: piano.\r\n"
    );

    for (;;) {
        recibir_comandos();

        uint32_t ahora = ahora_ms();

        if (modo_cancion)
            actualizar_cancion(ahora);

        actualizar_teclado(ahora_ms());

        cli();

        // Esta es una llamada a la funcion.
        // La funcion esta definida fuera de main().
        parar_reloj_si_estable();

        if (rx_leer != rx_escribir || rx_error) {
            sei();
        } else {
            sleep_enable();
            sei();
            sleep_cpu();
            sleep_disable();
        }
    }
}