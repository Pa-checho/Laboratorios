# Etapa 6: Ingresar el punto medio por terminal

Esta es una version completa y compilable. Utiliza `Problema_A_06_menu_completo.c` como unico archivo con main del proyecto, o carga `Problema_A_06_menu_completo.hex` en Proteus a 16 MHz.

## Que se aprende

El menu lee caracteres sin esperar bloqueado a Enter. static conserva los digitos entre llamadas. Enter valida un entero de -10 a 60 y recalcula todos los rangos. Un cambio aplica la ultima temperatura medida de inmediato.

## Como probar

Con temperatura 30 C, inicialmente hay 1 LED. Escribe 25 y Enter: debe quedar todo apagado, con Medio:19-31C. Prueba 61 y Enter: debe rechazarlo. r restaura 22; m muestra rangos; Escape cancela. Para probar 3 LED: punto medio 20 y sensor 49 C.

## Antes de seguir

Explica con tus palabras que configura cada funcion, que ocurre dentro de main y que cambio respecto de la etapa anterior. Realiza las pruebas indicadas y anota tus resultados reales.

## Comprobacion realizada

Compila con AVR-GCC (-Os, -Wall -Wextra -Werror) y XC8 para AVR (-O1). Esta comprobacion no reemplaza probar esta etapa en Proteus o en la placa.



Prueba adicional: -10 y Enter debe aceptarse; -11 debe rechazarse. En Data Visualizer marcar Add \r\n. El signo menos solo se permite al principio.
