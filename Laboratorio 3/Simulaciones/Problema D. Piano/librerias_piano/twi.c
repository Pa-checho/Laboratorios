#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include "twi.h"

// Biblioteca local basada en las operaciones de las paginas 9 a 11 de
// "13. Comunicacion I2C", UTEC, 2026. No requiere el entorno de Arduino.

void TWI_init(void)
{
    // Bus a 100 kHz. Para 16 MHz, TWBR vale 72 (0x48), como en clase.
    TWSR = 0;  // Prescaler igual a 1.
    TWBR = (uint8_t)(((F_CPU / 100000UL) - 16UL) / 2UL);
    TWCR = (1 << TWEN);
}

void TWI_start(void)
{
    // START inicia la comunicacion. La direccion se manda despues con TWI_write.
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT))) {
    }
}

void TWI_write(uint8_t dato)
{
    TWDR = dato;
    TWCR = (1 << TWEN) | (1 << TWINT);
    // Espero a que el hardware termine de transmitir el byte.
    while (!(TWCR & (1 << TWINT))) {
    }
}

void TWI_stop(void)
{
    TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
    // Espero a que termine STOP antes de iniciar otro envio al LCD.
    // En STOP se consulta TWSTO, no TWINT.
    while (TWCR & (1 << TWSTO)) {
    }
}
