# Problema A: Calefactor automatizado

Secuencia didactica en C para ATmega328P / Arduino Uno R3 a 16 MHz.
Se reorganiza el programa completo en etapas de aprendizaje. Estas versiones
no representan un historial anterior de desarrollo ni pruebas fisicas realizadas.

| Etapa | Codigo | Explicacion |
|---|---|---|
| 1. Probar los LED | [Problema_A_01_leds.c](Problema_A_01_leds.c) | [Problema_A_01_leds.md](Problema_A_01_leds.md) |
| 2. Agregar el puerto serie | [Problema_A_02_serie.c](Problema_A_02_serie.c) | [Problema_A_02_serie.md](Problema_A_02_serie.md) |
| 3. Usar Timer1 cada 5 segundos | [Problema_A_03_timer.c](Problema_A_03_timer.c) | [Problema_A_03_timer.md](Problema_A_03_timer.md) |
| 4. Medir con DHT11 y controlar los LED | [Problema_A_04_dht11_control.c](Problema_A_04_dht11_control.c) | [Problema_A_04_dht11_control.md](Problema_A_04_dht11_control.md) |
| 5. Agregar la pantalla LCD | [Problema_A_05_lcd.c](Problema_A_05_lcd.c) | [Problema_A_05_lcd.md](Problema_A_05_lcd.md) |
| 6. Ingresar el punto medio por terminal | [Problema_A_06_menu_completo.c](Problema_A_06_menu_completo.c) | [Problema_A_06_menu_completo.md](Problema_A_06_menu_completo.md) |

## Uso

1. Lee Problema_A_CONEXIONES.md y la explicacion de la etapa.
2. En Microchip Studio crea un proyecto C para ATmega328P y usa SOLO el codigo de esa etapa como main.c. No agregues las seis versiones al mismo proyecto: todas tienen main.
3. Compila con AVR-GCC y optimizacion -Os, o XC8 para AVR con -O1.
4. Para Proteus, cada etapa incluye su HEX a 16 MHz. Deten la simulacion antes de cambiarlo.
5. Prueba la etapa, explica su funcionamiento y luego avanza a la siguiente.

Son programas C con registros AVR, no sketches de Arduino. La Uno R3 es compatible
con el micro y el reloj utilizados, pero cargar el HEX fisicamente requiere un
procedimiento de programacion por bootloader o ISP. No modificar fusibles al azar.

Los LED representan calefactor y niveles de ventilacion; no hay salida PWM ni
conexion a equipos de potencia. El programa final mide cada 5 segundos, permite
ingresar el punto medio por UART y muestra datos en LCD. El punto medio vuelve a
22 C al reiniciar; no se guarda en EEPROM.

## Rangos de la version completa

| Accion | Con punto medio P | Con P=22 |
|---|---|---|
| Calefactor | T < P-6 | 0..15 C |
| Reposo | P-6 <= T <= P+6 | 16..28 C |
| 1 LED | P+6 < T <= P+17 | 29..39 C |
| 2 LED | P+17 < T <= P+28 | 40..50 C |
| 3 LED | T > P+28 | Desde 51 C |

Se trabaja con temperaturas enteras. Se incluye 51 C para cubrir el hueco de la
consigna. El DHT11 habitual mide de 0 a 50 C: prueba nivel alto con P=20 y T=49.
El intervalo permitido del punto medio, -10..60, es una decision de implementacion.

El menu usa recepcion sencilla sin cola. Escribe normalmente y espera la
confirmacion; si se detectan caracteres perdidos, cancela con Escape y repite.
Los mensajes periodicos pueden interrumpir visualmente lo que estas escribiendo.

## Validacion

Las seis etapas se compilaron con AVR-GCC y XC8. Los HEX fueron generados por GCC.
Compilar no verifica el cableado, los tiempos electricos del DHT11 ni la LCD.
Cada explicacion contiene pruebas para realizar en Proteus o hardware.


