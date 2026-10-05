# Etapa 3: Usar Timer1 cada 5 segundos

Esta es una version completa y compilable. Utiliza `Problema_A_03_timer.c` como unico archivo con main del proyecto, o carga `Problema_A_03_timer.hex` en Proteus a 16 MHz.

## Que se aprende

Timer1 cuenta 62500 pulsos por segundo: 16000000/256. OCR1A=62499 porque el conteo incluye cero. La ISR cuenta cinco segundos y pone medir=1. volatile indica que la variable puede cambiar en una interrupcion.

## Como probar

Al iniciar, todas las salidas estan apagadas. A los 5 s comienza la secuencia anterior y cambia cada 5 s. Ya no se usa delay de un segundo en main.

## Antes de seguir

Explica con tus palabras que configura cada funcion, que ocurre dentro de main y que cambio respecto de la etapa anterior. Realiza las pruebas indicadas y anota tus resultados reales.

## Comprobacion realizada

Compila con AVR-GCC (-Os, -Wall -Wextra -Werror) y XC8 para AVR (-O1). Esta comprobacion no reemplaza probar esta etapa en Proteus o en la placa.
