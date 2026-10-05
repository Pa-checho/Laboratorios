# Etapa 5: Agregar la pantalla LCD

Esta es una version completa y compilable. Utiliza `Problema_A_05_lcd.c` como unico archivo con main del proyecto, o carga `Problema_A_05_lcd.hex` en Proteus a 16 MHz.

## Que se aprende

La LCD recibe cada byte en dos grupos de 4 bits. RS distingue caracteres de comandos, E confirma el dato. Se muestran temperatura, punto medio y rango de reposo.

## Como probar

Completa la LCD y ajusta el contraste. Primero debe mostrar T:-- y PM:22C; luego la temperatura. La segunda linea debe decir Medio:16-28C. Verifica tambien los LED y la terminal.

## Antes de seguir

Explica con tus palabras que configura cada funcion, que ocurre dentro de main y que cambio respecto de la etapa anterior. Realiza las pruebas indicadas y anota tus resultados reales.

## Comprobacion realizada

Compila con AVR-GCC (-Os, -Wall -Wextra -Werror) y XC8 para AVR (-O1). Esta comprobacion no reemplaza probar esta etapa en Proteus o en la placa.
