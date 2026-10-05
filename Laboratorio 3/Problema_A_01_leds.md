# Etapa 1: Probar los LED

Esta es una version completa y compilable. Utiliza `Problema_A_01_leds.c` como unico archivo con main del proyecto, o carga `Problema_A_01_leds.hex` en Proteus a 16 MHz.

## Que se aprende

DDRx define entradas y salidas. PORTx enciende o apaga los pines. La funcion ventilador enciende LED acumulativos.

## Como probar

Conecta solamente los cuatro LED. Debe verse un segundo de calefactor, uno de reposo, uno con 1 LED, uno con 2 y uno con 3. La secuencia se repite.

## Antes de seguir

Explica con tus palabras que configura cada funcion, que ocurre dentro de main y que cambio respecto de la etapa anterior. Realiza las pruebas indicadas y anota tus resultados reales.

## Comprobacion realizada

Compila con AVR-GCC (-Os, -Wall -Wextra -Werror) y XC8 para AVR (-O1). Esta comprobacion no reemplaza probar esta etapa en Proteus o en la placa.
