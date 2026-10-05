# Conexiones comunes a las etapas

Arduino Uno R3 con ATmega328P a 16 MHz. No corresponde a Uno R4.

| Funcion | Puerto AVR | Pin Uno R3 |
|---|---|---|
| DHT11 DATA | PD2 | D2 |
| LED ventilador 1 | PD4 | D4 |
| LED ventilador 2 | PD5 | D5 |
| LED ventilador 3 | PD6 | D6 |
| LED calefactor | PB0 | D8 |
| LCD RS (4) | PB1 | D9 |
| LCD E (6) | PB2 | D10 |
| LCD D4 (11) | PC0 | A0 |
| LCD D5 (12) | PC1 | A1 |
| LCD D6 (13) | PC2 | A2 |
| LCD D7 (14) | PC3 | A3 |

Cada LED: pin -> resistencia propia de 330 ohm -> anodo; catodo -> GND.
DHT11: VCC a 5V, GND a GND y resistencia de 4,7 kohm entre DATA y 5V si el modulo
no la incorpora. Guiarse por etiquetas: la disposicion cambia entre modulos.

LCD paralelo compatible HD44780, sin I2C: 1 VSS a GND, 2 VDD a 5V, 3 VO al
centro de un potenciometro de 10 kohm con extremos a 5V y GND, 5 RW a GND.
Pines 7..10 sin conectar. Luz de fondo: 15 A a 5V mediante 220 ohm como primera
prueba conservadora, 16 K a GND; revisar especificacion del modulo.

Alimentar la Uno por USB para la prueba con LED. GND comun para todos los
componentes. Agregar 100 nF entre alimentacion y GND junto al DHT11 y otro junto
al LCD. Desconectar alimentacion para modificar el cableado. La Uno ya trae
reloj, reset y desacoplo del micro. Dejar AREF libre.

En la Uno, el terminal de PC usa el USB: dejar D0 y D1 libres. Usar 9600, 8N1,
con fin de linea CR o LF para confirmar numeros en la etapa 6.
En Proteus, terminal RXD a PD1/TXD y terminal TXD a PD0/RXD. Desactivar eco local.
Al usar ATmega suelto, alimentar VCC y AVCC y conectar GND; la numeracion fisica
depende del encapsulado. Configurar simulacion a 16 MHz y sin divisor CLKDIV8.

Referencias:
- https://content.arduino.cc/assets/Pinout-UNOrev3_latest.pdf
- https://learn.adafruit.com/dht/connecting-to-a-dhtxx-sensor
- https://learn.adafruit.com/character-lcds/wiring-a-character-lcd

En el montaje del estudiante funciono una resistencia de 2 kohm entre VO y GND como contraste fijo; puede variar segun el LCD.
