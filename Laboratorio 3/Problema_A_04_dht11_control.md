# Etapa 4: Medir con DHT11 y controlar los LED

Esta es una version completa y compilable. Utiliza `Problema_A_04_dht11_control.c` como unico archivo con main del proyecto, o carga `Problema_A_04_dht11_control.hex` en Proteus a 16 MHz.

## Que se aprende

Se reemplaza la secuencia de demostracion por la temperatura real. El DHT11 envia 5 bytes y se verifica su suma. Si falla, se apagan las salidas. Las comparaciones usan punto_medio=22, todavia fijo.

## Como probar

Prueba 10 C: calefactor; 22 C: reposo; 30 C: 1 LED; 45 C: 2 LED. Espera hasta la proxima lectura. Para revisar el nivel alto sin superar 50 C, cambia temporalmente punto_medio a 20, recompila y prueba 49 C. Con el sensor desconectado debe informar error y apagar todo.

## Antes de seguir

Explica con tus palabras que configura cada funcion, que ocurre dentro de main y que cambio respecto de la etapa anterior. Realiza las pruebas indicadas y anota tus resultados reales.

## Comprobacion realizada

Compila con AVR-GCC (-Os, -Wall -Wextra -Werror) y XC8 para AVR (-O1). Esta comprobacion no reemplaza probar esta etapa en Proteus o en la placa.
