# Etapa 2: Agregar el puerto serie

Esta es una version completa y compilable. Utiliza `Problema_A_02_serie.c` como unico archivo con main del proyecto, o carga `Problema_A_02_serie.hex` en Proteus a 16 MHz.

## Que se aprende

La UART transmite texto a 9600 baudios. serie_texto envia los caracteres; numero_a_texto prepara numeros sin usar itoa.

## Como probar

Conecta la terminal como indica Problema_A_CONEXIONES.md. A 9600 8N1 debe aparecer un mensaje por segundo coincidente con cada estado de los LED. Aun no mide temperatura.

## Antes de seguir

Explica con tus palabras que configura cada funcion, que ocurre dentro de main y que cambio respecto de la etapa anterior. Realiza las pruebas indicadas y anota tus resultados reales.

## Comprobacion realizada

Compila con AVR-GCC (-Os, -Wall -Wextra -Werror) y XC8 para AVR (-O1). Esta comprobacion no reemplaza probar esta etapa en Proteus o en la placa.
