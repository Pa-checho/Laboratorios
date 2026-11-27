# Auto robótico — UTEC

Proyecto de Tecnologías de Microprocesamiento: auto de tracción diferencial con Arduino Uno / ATmega328P a 16 MHz, control Bluetooth y mecanismo de brazo y garra. El HC-SR04 mide distancia para mostrarla en la aplicación y generar un aviso de proximidad.

## Archivos

| Ruta | Contenido |
| --- | --- |
| `Codigo/main.c` | Única fuente vigente, en C con AVR-libc y acceso directo a registros. |
| `Codigo/compilar.ps1` | Compilación para ATmega328P; no carga el programa. |
| `Codigo/build/` | ELF y HEX generados al compilar; excluidos de Git. |
| `Codigo/Respaldo/CODIGO_anterior_160_180.txt` | Original anterior conservado; no usar para compilar. |
| `Aplicacion/APK/` | Instalador Android `AutoPerillaVisible.apk`. |
| `Aplicacion/AIA/` | Proyecto editable `AutoPerillaVisible.aia`. |

## Componentes y conexiones

| Componente / señal | Arduino Uno |
| --- | --- |
| L298N: ENA, IN1, IN2 — motor izquierdo | D5, D7, D8 |
| L298N: ENB, IN3, IN4 — motor derecho | D6, D11, D12 |
| Servo de garra | D9 |
| Servo de brazo | D10 |
| HC-SR04: TRIG / ECHO | A1 / A0 |
| HC-05: TX → RX del Arduino | D0 |
| HC-05: RX ← TX del Arduino, retorno de distancia | D1; adaptar el nivel de la señal al RX del módulo |
| Referencia de señales | GND común entre Arduino, sensores, servos y puente H |

Alimentación declarada: powerbank para Arduino y su rama de componentes; pila de 9 V para tracción, con fusible de 1 A en serie con su positivo. Dos interruptores accesibles, uno para el puente H y otro para Arduino. El docente autorizó la pila no recargable. Confirmar en el esquemático la distribución de alimentación, los puentes del L298N y la adaptación de nivel del HC-05; la tabla describe las señales requeridas, no una inspección física del cableado.

Calibraciones vigentes: PWM izquierdo/derecho **175/190** sobre 255; garra abierta/cerrada **1420/1300 µs**; brazo abajo/medio/arriba **2200/1475/750 µs**. La corrección del sentido del brazo ya está incorporada.

## Compilar y cargar

Se requiere AVR-GCC con AVR-libc y Binutils para AVR. El programa define `main()` y configura los periféricos directamente; se compila como C independiente del núcleo Arduino, sin convertirlo a un sketch `.ino`.

Desde esta carpeta, en PowerShell:

```powershell
.\Codigo\compilar.ps1
# Si no se encuentra la instalación automáticamente:
.\Codigo\compilar.ps1 -AvrBin 'C:\ruta\a\avr-gcc\bin'
```

El script busca las herramientas en PATH o en las instalaciones habituales de Arduino y genera `Codigo/build/auto_concurso.hex`. Compila con `-mmcu=atmega328p`, `-DF_CPU=16000000UL`, optimización `-Os` y advertencias tratadas como errores. Informa el uso de memoria: en un Uno con bootloader estándar hay 32256 bytes de programa disponibles y 2048 bytes de SRAM, que también debe alojar la pila de ejecución.

La carga manual posterior requiere AVRDUDE y el bootloader de Arduino Uno. Desconectar el HC-05 de D0/D1 durante la carga, apagar la tracción y cerrar el monitor serie. Sustituir `COM3` por el puerto real:

```powershell
avrdude -p atmega328p -c arduino -P COM3 -b 115200 -D -U 'flash:w:Codigo/build/auto_concurso.hex:i'
```

Si AVRDUDE no está en PATH, usar la ruta completa a su ejecutable precedida de `&`; si no encuentra su configuración, agregar `-C 'C:\ruta\a\avrdude.conf'`. Los 115200 baudios corresponden al bootloader; la aplicación usa 9600. No se modifican fusibles del microcontrolador. Referencia: [AVRDUDE, ejemplo para Arduino Uno](https://github.com/avrdudes/avrdude).


## Aplicación

Los archivos definitivos ya están ubicados en `Aplicacion/APK/AutoPerillaVisible.apk` y `Aplicacion/AIA/AutoPerillaVisible.aia`. Copiar el APK al teléfono Android, abrirlo e instalarlo con los permisos que solicite el sistema. Emparejar el HC-05 y seleccionarlo desde la aplicación.

Para editar el AIA, abrir [MIT App Inventor](https://ai2.appinventor.mit.edu/) y usar **Projects → Import project (.aia) from my computer**. Exportar el proyecto editable y generar el APK desde la misma versión antes de entregar. [Referencia de MIT](https://mit-cml.github.io/explore/ai2/share.html).

## Protocolo Bluetooth y parada

UART **9600 baudios, 8 bits, sin paridad, 1 bit de parada**. La aplicación debe mantener el envío periódico incluso con el auto quieto y durante el movimiento del brazo.

| Mensaje | Función |
| --- | --- |
| `M<marcha>,<giro>\n` cada 50 ms | Dos enteros entre −100 y 100, sin espacios. `M` debe ser mayúscula. |
| `M0,0\n` | Detener la tracción y mantener activo el enlace. |
| `U` / `B` | Llevar el brazo hacia arriba / abajo. |
| `V` | Mantener la posición actual del brazo. |
| `G` | Alternar el objetivo de apertura/cierre de la garra; enviar una vez por pulsación. |
| `P,<distancia_cm>,<alerta>\n` cada 200 ms | Respuesta del robot: distancia entera y alerta 0/1; distancia −1 si no hay eco válido. |

`\n` representa el byte de salto de línea LF, no dos caracteres literales. También se admite CR o CRLF al recibir movimiento. Los botones admiten minúsculas y deben enviarse entre tramas completas, nunca dentro de una trama `M`.

La mezcla de motores usa `avance = -marcha`, izquierda = avance + giro y derecha = avance − giro, limitadas a ±100 antes de aplicar PWM. Por ello, `M-100,0\n` pide avance y `M100,0\n` retroceso según la convención del montaje.

Tras **300 ms sin una trama de movimiento válida**, se anula la tracción y se congelan los objetivos de ambos servos en la última posición comandada. Los pulsos de los servos siguen activos: la parada no corta la alimentación. Los mensajes inválidos no renuevan el plazo. Un botón inicia el plazo si el enlace estaba inactivo, pero los botones posteriores no sustituyen las tramas periódicas `M`. Se vuelve a conducir al recibir otra trama válida.

El sensor se consulta cada 70 ms, con esperas de eco limitadas a 30 ms por estado. La alerta se activa con una distancia positiva de hasta 20 cm y se libera desde 25 cm; la pérdida de eco envía −1 y desactiva la alerta. **La proximidad solo genera un aviso; no ordena frenar.**

