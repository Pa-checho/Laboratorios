#ifndef PIANO_TWI_H
#define PIANO_TWI_H

#include <stdint.h>

// Operaciones del bus I2C, siguiendo el ejemplo de la clase 13.
// En el ATmega328P, SDA es PC4 (A4) y SCL es PC5 (A5).
void TWI_init(void);
void TWI_start(void);
void TWI_write(uint8_t dato);
void TWI_stop(void);

#endif
