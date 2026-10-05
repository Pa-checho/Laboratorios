# Problema D: piano

Arduino Uno con ATmega328P a 16 MHz. El programa esta escrito en C y tiene su propio `main()`.

## Archivos

- `Problema D. Piano`: programa principal, teclado, LCD, timers y canciones.
- `librerias_piano/twi.h`: declaraciones de las funciones de I2C.
- `librerias_piano/twi.c`: implementacion de esas funciones.

La biblioteca es local y esta basada en las operaciones mostradas en las paginas 9 a 11 de la clase **13. Comunicacion I2C** (UTEC, 2026). El PDF muestra funciones propias que usan los registros del micro; no presenta una biblioteca externa. Se separaron esas operaciones para que el piano pueda usarlas sin repetir la configuracion del bus.

El envio al modulo de la pantalla sigue estos pasos:

```c
TWI_start();
TWI_write(DIRECCION_LCD << 1);  // Direccion de 7 bits y bit de escritura en 0.
TWI_write(dato);
TWI_stop();
```

La biblioteca cubre escritura, que es lo necesario para esta pantalla. No se incorpora la lectura del sensor del ejemplo de clase. Como en ese ejemplo, las operaciones esperan al hardware; esta version no implementa tiempo limite ni informa errores de ACK.

## Conexiones

| Componente | Pines |
| --- | --- |
| Botones Do, Re, Mi, Fa, Sol, La | D2 a D7, respectivamente |
| Botones Si y Do agudo | D8 y D10 |
| Buzzers pasivos mediante transistores | D9 y D11 |
| LCD con modulo PCF8574 | SDA: A4, SCL: A5, VCC: 5 V, GND: GND |

Los botones van a GND y usan pull-ups internos. El bus I2C necesita pull-ups en SDA y SCL; comprobar si el modulo ya las incluye.

La direccion del LCD es `0x27`. El mapeo usado es P0=RS, P1=RW, P2=E, P3=luz y P4 a P7=D4 a D7. La comunicacion I2C funciona a 100 kHz.

## Compilacion

Ahora hay que compilar el programa principal **junto con `twi.c`**. Incluir solamente el archivo `.h` no alcanza.

En Microchip Studio, agregar ambos archivos fuente al proyecto para ATmega328P. Guardar el principal como `main.c` y conservar la carpeta `librerias_piano` a su lado. No agregar un segundo `main()`.

Tambien se puede compilar desde esta carpeta con AVR-GCC:

```text
avr-gcc -x c -mmcu=atmega328p -DF_CPU=16000000UL -std=gnu99 -Os -Wall -Wextra -Werror "Problema D. Piano" librerias_piano/twi.c -o piano.elf
avr-objcopy -O ihex -R .eeprom piano.elf piano.hex
```

## Uso y prueba

El piano permite hasta dos notas simultaneas. La LCD muestra las notas de cada buzzer. La consola usa 9600 baudios, 8 bits de datos, sin paridad y un bit de parada.

- `C1`: Megalovania.
- `C2`: Dragonborn.
- `C3`: Bloody Tears.
- `C4`: Game of Thrones.
- `P`: detener la cancion y volver al piano.

Para comprobar el cambio en la placa: verificar el mensaje inicial, pulsar una y dos teclas, reproducir una cancion y detenerla con `P`. La compilacion no sustituye esta prueba del LCD y del cableado.
