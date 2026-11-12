# Problema D - Caracterización energética avanzada

Esta carpeta reúne los firmwares, los datos crudos y las gráficas obtenidas
para las primeras cuatro comparaciones del Problema D.

## Pruebas realizadas

| Prueba | Referencia (mA) | Optimizada (mA) | Ahorro |
|---|---:|---:|---:|
| ADC mediante PRR | 88,748 | 86,501 | 2,532 % |
| Timer2 mediante PRR | 125,456 | 98,104 | 21,802 % |
| ADC Noise Reduction | 85,992 | 78,554 | 8,649 % |
| Power-Save | 93,392 | 85,612 | 8,330 % |

Las mediciones corresponden al sistema completo alimentado desde 9 V por
`VIN`. La corriente se obtuvo con un shunt de 5 ohm, un LM358 con ganancia 11,
un divisor 2:1 y un segundo Arduino conectado a la computadora.

El resultado de Timer2 es demasiado grande para atribuirlo solamente a ese
periférico y debe informarse como diferencia observada en el montaje completo.
El Arduino Uno utilizado tampoco tiene un cristal asíncrono de 32,768 kHz para
Timer2, por lo que la prueba de Power-Save no demuestra funcionamiento
asíncrono y puede aproximarse a Power-Down.

## Contenido

- `Codigos`: variantes de firmware, adquisición del segundo Arduino y programa
  de registro en Python.
- `CSV`: datos crudos y resúmenes de las ocho capturas aceptadas.
- `Graficas`: capturas individuales y comparación general.
- `CONTEXTO_MEDICIONES.md`: procedimiento, fórmulas, resultados y limitaciones.
- `MAPA_DE_ARCHIVOS.md`: relación entre los nombres originales y los nombres
  organizados.

## Pendiente

- PRR individual de USART0, SPI, Timer0, Timer1 y TWI.
- Tiempo de despertar mínimo, promedio y máximo.
- WDT de 16 ms, 250 ms, 1 s y 8 s en interrupción y reset.
- Consumo frente a las frecuencias configuradas con `CLKPR`.
- Comparación cuantitativa con la hoja de datos.
