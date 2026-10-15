# Laboratorio 4 — Problema B: banda transportadora SPI

Dos ATmega328P programados en C con AVR-GCC y Microchip Studio. Cada micro utiliza un proyecto independiente: `maestro.c` y `esclavo.c` contienen su propio `main()`.

Esta serie de commits registra ahora las etapas reales de desarrollo y correccion del proyecto. Las fechas de los commits corresponden a su publicacion; las revisiones se mantienen sobre los mismos dos archivos.

## Funcionamiento

- El maestro lee boton, potenciometro, LDR y DHT11; muestra informacion en LCD 16x2 y envia ordenes SPI.
- El esclavo controla motor, servo que representa el piston, LED verde, LED rojo y alarma.
- Al encender queda parado. Esperar unos 2,2 s hasta la primera temperatura valida y pulsar para iniciar. Otra pulsacion detiene la banda.
- Una pieza oscurece el LDR, incrementa el contador y activa el servo durante 350 ms; vuelve a reposo y dispone de otros 350 ms para retornar. Dejar al menos un segundo entre piezas y volver a iluminar el LDR.
- Emergencia si T > 40 C o tras tres lecturas DHT11 fallidas: motor parado, servo a reposo, rojo y alarma. Para reconocerla, esperar T <= 37 C sin errores y pulsar. Queda parado; otra pulsacion inicia marcha.
- El contador se reinicia al reset. No implementa EEPROM ni cola de piezas.

Los umbrales 40/37 C, ADC 400/550 y pulsos del servo 1000/1500 us son decisiones de este montaje, ajustables en el codigo.

## SPI

Conectar PB2/SS, PB3/MOSI, PB4/MISO y PB5/SCK entre ambos micros, junto con GND comun. En UNO/Nano corresponden a D10, D11, D12 y D13. No cruzar MOSI/MISO.

Modo 0, MSB primero, F_CPU/128 (125 kHz a 16 MHz). Cada trama contiene comando y valor, con SS bajo durante ambos bytes. No hay ACK ni checksum SPI; `TX:` indica transmision, no confirmacion de ejecucion.

| Comando | Byte 1 | Byte 2 |
|---|---|---|
| START | 1 | PWM inicial, 0..255 |
| STOP | 2 | 0 |
| VELOCIDAD | 3 | PWM, 0..255 |
| PISTON | 4 | 0 |
| EMERGENCIA | 5 | 0 |

El maestro reenvia el estado cada 100 ms. El esclavo descarta tramas de longitud incorrecta o comandos desconocidos; sin una orden reconocida durante 1 s activa alarma y detiene los actuadores. Tras recuperar el enlace, pulsar Stop y Start; VELOCIDAD por si sola no rearranca al esclavo.

## Conexiones del maestro

Usar los nombres de puerto AVR para evitar confundir encapsulados DIP-28 y de 32 pines.

| Elemento | Puerto AVR | UNO/Nano |
|---|---|---|
| Cursor del potenciometro 10 kohm | PC0/ADC0 | A0 |
| Punto medio del divisor LDR | PC1/ADC1 | A1 |
| DATA del DHT11 | PD2 | D2 |
| Boton entre entrada y GND | PD3 | D3 |
| LCD RS | PD4 | D4 |
| LCD E | PD5 | D5 |
| LCD D4, D5 | PD6, PD7 | D6, D7 |
| LCD D6, D7 | PB0, PB1 | D8, D9 |

- Potenciometro: extremos a +5 V y GND; cursor a ADC0.
- LDR: +5 V -> LDR -> ADC1 -> resistencia 10 kohm -> GND. Al oscurecer baja el ADC. Deteccion por debajo de 400; se rearma por encima de 550. Calibrar segun la iluminacion.
- Boton: PD3 a GND al pulsar; pull-up interna y antirrebote de 30 ms.
- DHT11: VCC a +5 V, GND a tierra y pull-up de 4,7 kohm entre DATA y +5 V si no esta integrada. Seguir las etiquetas del modelo/modulo.
- LCD: VSS a GND, VDD a +5 V, RW a GND, VO al cursor de un potenciometro de contraste entre +5 V y GND. D0..D3 sin conectar. Alimentar la luz de fondo segun el modulo.

## Conexiones del esclavo

| Elemento | Puerto AVR | UNO/Nano |
|---|---|---|
| Senal del servo | PB1/OC1A | D9 |
| Gate del MOSFET mediante 330 ohm | PD3/OC2B | D3 |
| Sin conectar | PD4, PD5 | D4, D5 |
| LED verde, rojo | PD6, PD7 | D6, D7 |
| Control de alarma | PD2 | D2 |

Motor: etapa discreta IRLZ44N. PD3 -> 330 ohm -> Gate; 10 kohm entre Gate y Source. Source a GND, Drain al negativo del motor; positivo del motor a su fuente. Diodo de rueda libre en paralelo: catodo al positivo del motor y anodo al Drain. Dimensionar segun corriente de arranque/bloqueo. Para ejemplo pequeno de hasta 1 A se considero 1N5819. Agregar 100 nF en paralelo con el motor. PD4 y PD5 se dejan sin conectar. Permite un solo sentido de giro; el PWM de PD3 ya existente sirve para esta etapa, sin cambiar la logica del programa.

Alarma: buzzer pasivo apto para su alimentacion. Positivo a +5 V, negativo al colector de BC547, emisor a GND y base desde PD2 mediante 2,2 kohm. Para piezo pasivo, resistencia de 4,7 kohm en paralelo para descargarlo. Timer0 alterna PD2 cada 1 ms: tono de 500 Hz. STOP deja PD2 en bajo; los reenvios de emergencia no reinician la fase. Timer1 y Timer2 conservan sus funciones.

LEDs: salida -> 330 ohm -> anodo; catodo a GND. Servo: positivo a 5 V regulados, negativo a GND comun y senal a PB1. Usar fuente con capacidad para el servo; no alimentarlo desde un pin del micro.

## Alimentacion y compilacion

Logica a 5 V. Conectar todas las alimentaciones VCC/AVCC y GND del ATmega328P, incluidos pines ocultos en Proteus; usar redes VCC/GND con fuente definida. En DIP-28: VCC=7, AVCC=20, GND=8/22, RESET=1 con pull-up de 10 kohm. Agregar 100 nF de desacople junto a cada alimentacion y 100 nF de AREF a GND usando referencia AVCC. En otros encapsulados seguir nombres de pin.

Motor con fuente apropiada a su tension/corriente, separada de logica y con GND comun. No unir positivos de fuentes distintas. Los 9 V de una fuente de motor no se conectan a logica, LCD, DHT11 ni servo.

Crear dos proyectos GCC C Executable Project para ATmega328P. Sustituir el `main.c` de cada proyecto por el contenido del archivo correspondiente; evitar dos funciones `main()` en el mismo proyecto. Compilar Release con `-Os`. El usuario genera los dos HEX y carga cada uno en su micro correspondiente en Proteus.

`F_CPU=16000000UL` describe el reloj esperado; no configura el reloj fisico ni los fusibles. Ambos fuentes admiten tambien 8 MHz si se compilan con `F_CPU=8000000UL` y el reloj real coincide. No se incluyen HEX en el repositorio.

## Pantalla y temporizadores

Primera fila: P/M/E (parado/marcha/emergencia), T en grados C y V en porcentaje. `--` significa temperatura no valida. V muestra el ajuste elegido incluso en paro. La segunda fila alterna cada 1,2 s entre contador/ADC del LDR y ultimo evento `TX:`.

Maestro: Timer0 base de 1 ms y Timer1 para medir DHT11. Esclavo: Timer0 base de 1 ms, Timer1 servo de 50 Hz y Timer2 PWM del motor (~976,6 Hz a 16 MHz). El movimiento del servo y la recepcion SPI se gestionan mediante interrupciones; la lectura DHT11 del maestro utiliza esperas limitadas.

## Validacion de esta etapa

Los fuentes iniciales se compilaron con AVR-GCC para ATmega328P a 8 y 16 MHz, `-Os -Wall -Wextra -Werror`, sin advertencias. A 16 MHz: maestro 4048 bytes de programa/118 bytes de RAM estatica; esclavo 912/10 bytes. Estas cifras corresponden exclusivamente a la version inicial y no incluyen pila.

Consigna de referencia: Problema B de Laboratorio 4.pdf proporcionado para el trabajo.

## Cambio a buzzer pasivo

Se sustituyo el buzzer activo por uno pasivo y se agrego el tono al Timer0 del esclavo, sin esperas adicionales. Los HEX actualizados se generan en Microchip Studio; las cifras de memoria anteriores no describen esta revision.

## Reloj y servo en Proteus

En ambos modelos: Clock Frequency=16MHz, CKDIV8=(1) Unprogrammed y CKSEL=1111 para cristal externo en rango 8-16 MHz. Agregar cristal de 16 MHz entre XTAL1 y XTAL2, con 22 pF desde cada extremo a GND como ejemplo de simulacion. En el simbolo de 32 pines utilizado: XTAL1/PB6=7 y XTAL2/PB7=8; en DIP-28 son 9/10. En un montaje fisico elegir capacitores segun el cristal. UNO/Nano ya incorpora su reloj; verificarlo antes de modificar fusibles.

Durante el ajuste se observo un pulso de reposo de aproximadamente 2 ms y servo fijo en +90 grados. Tras corregir el reloj se confirmo en Proteus -90 -> 0 -> -90: reposo de 1000 us y accion de 1500 us con los limites del modelo -90/+90. Comprobar periodo de 20 ms con osciloscopio si se requiere medicion; no se aporto una nueva captura de los anchos tras el ajuste. No fue necesario modificar `F_CPU` ni el calculo del PWM del firmware.
## Correccion de la primera fila del LCD

Se construyen directamente los 16 caracteres de estado, temperatura y porcentaje en `maestro.c`, reemplazando el formato con una cadena temporal de temperatura. Se utiliza la misma variable de temperatura que decide la emergencia. El usuario confirmo que la revision muestra correctamente temperatura y velocidad en Proteus. Recompilar y cargar el HEX del maestro para aplicar este cambio.
## Modelo de sonido y pruebas finales

En Proteus, reemplazar el simbolo BUZZER sin modelo por **SOUNDER** de la biblioteca ACTIVE: modelo piezo digital con salida por tarjeta de sonido. Seleccionarlo desde Pick Devices y mantener Exclude from Simulation desmarcado. Positivo a +5 V y negativo al colector del BC547; mantener resistencia de 4,7 kohm en paralelo. El reemplazo del componente no cambia el codigo ni requiere otro HEX.

Se observo una onda cuadrada de aproximadamente 500 Hz, periodo 2 ms y ciclo util del 50 % durante emergencia. Tras sustituir el componente acustico el usuario confirmo el funcionamiento completo de la simulacion.

| Prueba | Resultado confirmado por el usuario |
|---|---|
| Arranque | LED verde encendido y motor girando |
| Oscurecimiento/iluminacion del LDR | Contador aumenta y servo hace -90 -> 0 -> -90 |
| LCD | Temperatura y velocidad visibles tras la correccion |
| Sobretemperatura | TX:EMERG, motor detenido y LED rojo |
| Alarma pasiva | Senal de 500 Hz observada; confirmacion general de funcionamiento tras SOUNDER |

Comprobar de forma separada para completar las evidencias: variacion de velocidad, pieza mantenida sin repeticion, paro manual, rearme a <=37 C con dos pulsaciones separadas (reconocer y arrancar), tres fallos del DHT11 y perdida/recuperacion de SPI. La confirmacion general no sustituye registros individuales de esas pruebas. Las pruebas fisicas siguen pendientes.

Los HEX utilizados en la simulacion fueron generados por el usuario. No hay aqui un registro de compilacion ni cifras de memoria actualizados de la version final; las cifras de la etapa inicial no se presentan como finales. El repositorio contiene los dos fuentes y esta guia, sin binarios obsoletos ni versiones duplicadas. El proyecto de Proteus y las evidencias de informe/video se incorporaran en otra etapa.