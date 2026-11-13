/*
 * LAB 5 / PROBLEMA B / ACTIVIDADES 1-4 - VERSION COMENTADA - TEXTOS FLASH LOCALES
 * ATmega328P, F_CPU=16 MHz, avr-gcc; FSM if/else, sin switch-case.
 * ACTIVO: tareas; STANDBY: IDLE; APAGADO: Power-Down; SEGURO: alarma.
 * UART 9600 8N1: @COMANDO,DATO*HH + CR+LF.
 * HH es el XOR de los bytes ASCII de COMANDO,DATO, incluida la coma.
 * ISR breves, parser en main, textos Flash y TX circular sin copia temporal.
 * WDT: la prueba se selecciona con WDT_PRUEBA_ACTIVA y se recompila.
 *
 * Pines: D13/PB5 LED; D11/PB3 sounder; D12/PB4 marca de prueba WDT.
 * DHT11: D4/PD4 fisico o D3/PD3 en Proteus; D2/PD2 pulsador a GND.
 * A4/PC4 SDA; A5/PC5 SCL; D0/PD0 RX y D1/PD1 TX del HC-05/terminal.
 * Configuracion fisica y enlace HC-05 pendientes; revisar niveles de tension.
 * tiempo_ms excluye Power-Down; SEGURO exige recuperacion local.
 * Base de optimizacion: TMI-UTEC (UART, interrupciones y HC-05).
 * https://lord-jebus.github.io/TMI-UTEC/
 * Ver la guia para protocolo, bitacora, decisiones y limites de las pruebas.
 */

#ifndef F_CPU                                                            // Usa 16 MHz solo si el proyecto no definio ya la frecuencia.
#define F_CPU 16000000UL                                                 // Frecuencia asumida al compilar; no mide el reloj real de la placa.
#endif                                                                   // Cierra la condicion del preprocesador; no se ejecuta en tiempo de programa.

#include <avr/io.h>                                                      // Nombres de registros y bits del ATmega328P.
#include <avr/interrupt.h>                                               // ISR, cli y sei para las interrupciones.
#include <util/delay.h>                                                  // Retardos calculados para F_CPU; no es la base temporal de las tareas.
#include <util/twi.h>                                                    // Constantes de estado TWI para comprobar START y ACK.
#include <stdint.h>                                                      // Enteros de tamaño fijo: uint8_t, uint16_t y uint32_t.
#include <string.h>                                                      // Funciones de cadenas y busqueda de separadores.
#include <avr/sleep.h>                                                   // Seleccion y entrada/salida de los modos Sleep.
#include <avr/wdt.h>                                                     // Macros de configuracion y servicio del Watchdog.
#include <avr/pgmspace.h>                                                // PSTR y pgm_read_byte permiten acceder a textos guardados en Flash.

#if F_CPU != 16000000UL                                                  // Rechaza otra frecuencia: timers y tiempos DHT fueron calculados para 16 MHz.
#error "Esta version requiere revisar los temporizadores si F_CPU no es 16 MHz" // Detiene la compilacion para evitar una configuracion incompatible.
#endif                                                                   // Cierra la condicion del preprocesador; no se ejecuta en tiempo de programa.

#define LED_PIN                         PB5                              // PB5 corresponde a D13: salida de estado.
#define SOUNDER_PIN                     PB3                              // PB3/D11: el sounder se conmuta desde Timer0.
#ifndef DHT_PIN
#define DHT_PIN                         PD4                              // Montaje fisico por defecto: PD4/D4.
#endif

#if DHT_PIN == PD3
#define DHT_DDR_PIN                     DDD3
#define DHT_PCINT_BIT                   PCINT19                          // Simulacion Proteus: PD3/D3.
#elif DHT_PIN == PD4
#define DHT_DDR_PIN                     DDD4
#define DHT_PCINT_BIT                   PCINT20                          // Montaje fisico: PD4/D4.
#else
#error "DHT_PIN debe ser PD3 o PD4"
#endif
#define BOTON_PIN                       PD2                              // PD2/D2: INT0, pulsador a GND con pull-up.
#define WDT_PRUEBA_PIN                  PB4 /* D12: marca observable del WDT. */ // PB4/D12: referencia observable para los ensayos WDT.

#define WDT_PRUEBA_DESPERTADOR          1U                               // Interrupcion nominal de 8 s durante Power-Down.
#define WDT_PRUEBA_RESET                2U                               // Reset nominal de 2 s para recuperar un bloqueo.

#ifndef WDT_PRUEBA_ACTIVA
#define WDT_PRUEBA_ACTIVA               WDT_PRUEBA_DESPERTADOR            // Se puede cambiar con -DWDT_PRUEBA_ACTIVA=2.
#endif

#if WDT_PRUEBA_ACTIVA != WDT_PRUEBA_DESPERTADOR && \
    WDT_PRUEBA_ACTIVA != WDT_PRUEBA_RESET
#error "WDT_PRUEBA_ACTIVA debe ser WDT_PRUEBA_DESPERTADOR o WDT_PRUEBA_RESET"
#endif

#define LCD_RS                          0x01U                            // Bit RS del PCF8574: 0=instruccion, 1=caracter.
#define LCD_E                           0x04U                            // Bit E del expansor: pulso para capturar cada nibble.
#define LCD_LUZ                         0x08U                            // Bit de retroiluminacion del PCF8574, distinto de una instruccion LCD.
#define LCD_DIRECCION_NATIVA            0x3EU                            // JHD-2X16-I2C/AiP31068L utilizado por Proteus.
#define LCD_CONTROL_COMANDO             0x00U                            // Byte de control del LCD I2C nativo para instrucciones.
#define LCD_CONTROL_DATO                0x40U                            // Byte de control del LCD I2C nativo para caracteres.
#define LCD_TIPO_NINGUNO                0U
#define LCD_TIPO_PCF8574                1U
#define LCD_TIPO_NATIVO                 2U

#define PERIODO_LED_MS                  200U                             // Se conmuta cada 200 ms: un ciclo encendido/apagado completo dura 400 ms.
#define PERIODO_LCD_MS                  500U                             // Periodo de actualizacion de pantalla en ACTIVO.
#define PERIODO_DHT_MS                  2000U                            // Separacion minima de 2 s entre inicios de adquisicion DHT.
#define DURACION_SOUNDER_MS             500U                             // Aviso temporal de medio segundo.
#define ANTI_REBOTE_MS                  50UL                             // Separacion minima entre pulsaciones aceptadas.
#define PULSACION_EMERGENCIA_MS         2000UL                           // Mantener el boton 2 s solicita emergencia local.
#define TIEMPO_STANDBY_MS               10000UL                          // Umbral automatico de 10 s desde la ultima actividad.
#define TIEMPO_APAGADO_MS               30000UL                          // Umbral de 30 s desde esa misma actividad, no desde STANDBY.

/*
 * Si vale 1, un cambio intencional del potenciometro cuenta como actividad.
 * Si el potenciometro se usa solo para simular un sensor, cambiar a 0.
 */
#define POTENCIOMETRO_ES_ACTIVIDAD      1U                               // A 1, el cambio intencional de A0 puede renovar actividad.
#define UMBRAL_ACTIVIDAD_ADC            20U                              // Cambio minimo de A0 para considerarlo accion del usuario.


#define RELOJ_RAPIDO                    0U                               // Modo logico con Timer0 a 1 ms.
#define RELOJ_STANDBY                   1U                               // Modo logico con Timer1 a 1 segundo.
#define RELOJ_DETENIDO                  2U                               // Ningun timer sostiene el tiempo mientras duerme en Power-Down.
#define PASO_STANDBY_MS                 1000UL                           // Un despertar por segundo reduce actividad innecesaria en IDLE.

#define UART_RX_TAM                     64U                              // Buffer RX circular; se dejan 63 posiciones utilizables.
#define UART_TX_TAM                     256U                             // Buffer TX circular; se dejan 255 posiciones utilizables.
#define UART_LINEA_TAM                  64U                              // Espacio de una trama recibida y su terminador cero.
#define UART_TIMEOUT_LINEA_MS           3000UL                           // Descarta una trama si pasan 3 s sin recibir otro byte.
#define UART_BAUD                       9600UL                           // Velocidad nominal del enlace de datos.
#define UART_UBRR_9600                  (F_CPU / (16UL * UART_BAUD) - 1UL) // Formula asíncrona normal: a 16 MHz resulta UBRR=103.
#define PROTO_RX_PRESUPUESTO            16U                              // Cota de bytes por llamada para devolver tiempo a otras tareas.

#if UART_TX_TAM != 256U                                                  // Comprueba al compilar las condiciones que requieren los indices circulares.
#error "TX requiere 256 bytes para su envoltura uint8_t"                 // Detiene la compilacion para evitar una configuracion incompatible.
#endif                                                                   // Cierra la condicion del preprocesador; no se ejecuta en tiempo de programa.

#if UART_RX_TAM < 2U || UART_RX_TAM > 256U || (UART_RX_TAM & (UART_RX_TAM - 1U)) // Comprueba al compilar las condiciones que requieren los indices circulares.
#error "RX debe ser potencia de dos entre 2 y 256"                       // Detiene la compilacion para evitar una configuracion incompatible.
#endif                                                                   // Cierra la condicion del preprocesador; no se ejecuta en tiempo de programa.

#define PROTO_TRAMA_TAM                 96U                              // Limite usado al construir y reservar tramas de salida.
#define PROTO_RESERVA_TX                192U /* Una orden pendiente, sin bloquear. */ // Espacio requerido antes de ejecutar una orden pendiente.
#define UART_ERR_FRAME                  0x01U                            // Mascara de error del bit de parada recibido.
#define UART_ERR_OVERRUN                0x02U                            // Mascara de perdida de datos en el receptor hardware.
#define UART_ERR_PARIDAD                0x04U                            // Mascara de error de paridad detectado por hardware.
#define UART_ERR_BUFFER                 0x08U                            // Mascara de desbordamiento del buffer de software.
#define LED_AUTOMATICO                  0U                               // El planificador controla el parpadeo.
#define LED_FORZADO_OFF                 1U                               // Orden remota para mantener LED en cero durante ACTIVO.
#define LED_FORZADO_ON                  2U                               // Orden remota para mantener LED en uno durante ACTIVO.
#define ALARMA_LOCAL                    1U                               // Causa: pulsacion prolongada del boton.
#define ALARMA_REMOTA                   2U                               // Causa: orden EMERGENCIA validada.
#define ALARMA_WDT                      3U                               // Causa identificada con WDRF disponible.
#define ALARMA_TRAS_BLOQUEO              4U                              // Marca de bloqueo sin flag WDRF: no atribuye causa con certeza.
#define ALARMAS_TAM                     4U                               // Historial en SRAM: conserva cuatro eventos, sin persistencia EEPROM.
#define ES(texto, literal) (texto_comparar_P((texto), PSTR(literal)) == 0)       // Compara texto RAM con un literal Flash; devuelve verdadero si coincide.
#define COPIAR(destino, literal) texto_copiar_P((destino), PSTR(literal))      // Copia un literal Flash al buffer RAM de respuesta.
#define ERROR_PROTO(literal) protocolo_error(PSTR(literal))              // Emite el motivo como trama ERROR desde memoria de programa.

/* ACTIVIDAD 3: bits definidos del ATmega328P. Nunca usar PRR=0xFF. */
#define PRR_ADC        _BV(PRADC)       /* 0x01 */                       // Bit 0: a 1 detiene el reloj del ADC; primero se borra ADEN.
#define PRR_USART      _BV(PRUSART0)    /* 0x02 */                       // Bit 1: a 1 quita reloj UART; no recibira Bluetooth dormido.
#define PRR_SPI        _BV(PRSPI)       /* 0x04 */                       // Bit 2: a 1 quita reloj SPI, que esta aplicacion no usa.
#define PRR_TIMER1     _BV(PRTIM1)      /* 0x08 */                       // Bit 3: no apagar si sostiene STANDBY o el muestreo de ACTIVO.
#define PRR_TIMER0     _BV(PRTIM0)      /* 0x20 */                       // Bit 5: no apagar si sostiene reloj rapido y sounder.
#define PRR_TIMER2     _BV(PRTIM2)      /* 0x40 */                       // Bit 6: reloj necesario mientras se capturan pulsos DHT.
#define PRR_TWI        _BV(PRTWI)       /* 0x80 */                       // Bit 7: quita reloj I2C despues de terminar la transferencia.
#define PRR_VALIDOS    (PRR_ADC | PRR_USART | PRR_SPI | PRR_TIMER1 | \
                        PRR_TIMER0 | PRR_TIMER2 | PRR_TWI) /* 0xEF */
#define PRR_ACTIVO     (PRR_SPI | PRR_TIMER2)             /* 0x44 */     // 0x44: SPI y Timer2 sin reloj entre capturas DHT.
#define PRR_STANDBY    (PRR_ADC | PRR_SPI | PRR_TIMER0 | PRR_TIMER2 | PRR_TWI) // 0xE5: solo UART y Timer1 quedan con reloj.
                                                       /* 0xE5 */
#define PRR_SEGURO     (PRR_ADC | PRR_SPI | PRR_TIMER1 | PRR_TIMER2 | PRR_TWI) // 0xCD: UART y Timer0 conservan alarma y respuesta local.
                                                       /* 0xCD */
#define PRR_DORMIR     PRR_VALIDOS                        /* 0xEF */     // 0xEF: detiene los siete modulos; INT0/WDT no pertenecen a PRR.

#define TWI_LIBRE      0U                                                // No hay operacion I2C pendiente.
#define TWI_TRANSFIRIENDO 1U                                             // El bus esta atendiendo una transferencia.
#define TWI_STOP_PENDIENTE 2U                                            // STOP solicitado pero aun no confirmado.
/* Diagnostico: ultimo rechazo conservado hasta el siguiente rechazo. */
#define PRR_OK        0U                                                 // No se detecto impedimento para el perfil solicitado.
#define PRR_ERR_BIT    1U                                                // Se solicito un bit reservado/no definido.
#define PRR_ERR_RELOJ  2U                                                // Se intenta apagar un reloj necesario.
#define PRR_ERR_DHT    3U                                                // Timer2 esta capturando datos del sensor.
#define PRR_ERR_ADC    4U                                                // ADC aun esta convirtiendo.
#define PRR_ERR_UART   5U                                                // UART esta ocupada o debe conservarse.
#define PRR_ERR_TWI    6U                                                // I2C tiene transferencia o STOP pendiente.
#define PRR_ERR_SPI    7U                                                // SPI fue habilitado y no se apaga a ciegas.
#define PRR_ERR_LECTURA 8U                                               // PRR leido no coincide con lo escrito.

#define DHT_INICIO_BAJO_MS              20UL                             // Pulso inicial de 20 ms medido con el reloj, sin esperar en un bucle.

typedef enum {                                                           // Define los cuatro estados principales con nombres, no numeros dispersos.
    ESTADO_ACTIVO = 0,                                                   // Estado de tareas periodicas y comunicacion.
    ESTADO_STANDBY,                                                      // Estado de espera que utiliza Sleep IDLE.
    ESTADO_APAGADO,                                                      // Estado que prepara y utiliza Power-Down.
    ESTADO_SEGURO                                                        // Estado de emergencia; solo se reconoce localmente.
} estado_sistema_t;                                                      // Nombre del tipo usado por la FSM principal.

/* Estados internos del protocolo DHT11. Se almacenan en uint8_t. */
#define DHT_REPOSO                      0U                               // Sensor sin captura activa.
#define DHT_INICIO_BAJO                 1U                               // Microcontrolador mantiene DATA en cero.
#define DHT_ESPERA_RESPUESTA_BAJA       2U                               // Se espera que el sensor tome DATA y la lleve a cero.
#define DHT_RESPUESTA_BAJA              3U                               // Midiendo el bajo de confirmacion del sensor.
#define DHT_RESPUESTA_ALTA              4U                               // Midiendo el alto de confirmacion del sensor.
#define DHT_BIT_BAJO                    5U                               // Midiendo el bajo que precede a un bit.
#define DHT_BIT_ALTO                    6U                               // Midiendo el alto cuya duracion codifica 0/1.
#define DHT_COMPLETO                    7U                               // Ya se capturaron 40 bits; main comprobara checksum.
#define DHT_ERROR                       8U                               // La captura detecto una secuencia o duracion invalida.

static volatile uint32_t tiempo_ms = 0UL;                            // Volatile: reloj logico de milisegundos, detenido en Power-Down.
static volatile uint16_t contador_led_ms = 0U, contador_lcd_ms = 0U, contador_dht_ms = 0U;    // Volatile: tiempo acumulado hasta conmutar LED; tiempo hasta refrescar LCD; tiempo hasta solicitar DHT.
static volatile uint16_t tiempo_sonido_ms = 0U;                         // Volatile: duracion restante del aviso temporal.

static volatile uint8_t bandera_led = 0U, bandera_adc = 0U, bandera_lcd = 0U;         // Volatile: trabajo LED pendiente para main; adquisicion A0 pendiente; actualizacion de pantalla pendiente.
static volatile uint8_t bandera_dht = 0U, bandera_boton = 0U, alarma_continua = 0U;       // Volatile: solicitud de iniciar DHT; pulsacion pendiente de validar; habilitacion del sonido continuo de SEGURO.

static estado_sistema_t estado = ESTADO_ACTIVO;                         // Reserva: estado principal de la FSM.

static uint32_t ultima_actividad = 0UL, inicio_boton = 0UL, ultimo_boton = 0UL;         // Reserva: marca del ultimo evento aceptado; instante inicial de la pulsacion; marca para rechazar rebotes.

static uint8_t boton_presionado = 0U, pulsacion_larga_lista = 0U, lcd_disponible = 0U; // Reserva: pulsacion que main ya esta atendiendo; evita repetir la emergencia de una misma pulsacion; LCD detectada y sin fallo de escritura.
static uint8_t lectura_dht_correcta = 0U, error_dht = 0U, temperatura = 0U; // Reserva: validez de temperatura/humedad; codigo de error DHT publicado; temperatura DHT en grados Celsius.
static uint8_t humedad = 0U;                                             // Reserva: humedad relativa DHT en porcentaje.
static uint16_t valor_adc_actual = 0U, referencia_adc_actividad = 0U;    // Reserva: ultima adquisicion A0 de 10 bits; A0 de referencia para detectar cambios intencionales.
static uint8_t referencia_adc_valida = 0U;                               // Reserva: indica si ya se establecio la referencia.

static uint8_t direccion_lcd = 0x27U, luz_lcd = LCD_LUZ;                 // Direccion detectada y estado de luz del expansor.
static uint8_t tipo_lcd = LCD_TIPO_NINGUNO;                              // Distingue LCD nativo 0x3E de mochila PCF8574.

/* Variables de la lectura temporizada del DHT11. */
static volatile uint8_t estado_dht = DHT_REPOSO;                         // Paso actual de la lectura DHT.
static volatile uint8_t dht_datos[5] = {0U, 0U, 0U, 0U, 0U};             // Humedad, temperatura y checksum recibidos.
static uint32_t dht_marca_ms = 0UL;                                      // Reserva: inicio de la fase DHT para sus plazos.

/* Estado temporal y de comunicacion agregado para B1. */
static volatile uint8_t modo_reloj = RELOJ_RAPIDO, bandera_reloj = 0U;   // Volatile: timer que sostiene el reloj logico; tick de STANDBY pendiente.
static uint16_t fraccion_reloj_us = 0U;                                  // Reserva: fraccion menor a 1 ms conservada al cambiar de reloj.
static volatile uint8_t en_powerdown = 0U;                               // Volatile: identifica el contexto de despertar por INT0.

static volatile uint8_t uart_rx[UART_RX_TAM];                            // Volatile: bytes recibidos pendientes.
static volatile uint8_t uart_rx_cabeza = 0U, uart_rx_cola = 0U, uart_rx_error = 0U; // Volatile: posicion donde la ISR RX publicara el siguiente byte; posicion que main consumira; mascara de errores RX acumulados.
static volatile uint8_t uart_tx[UART_TX_TAM];                            // Volatile: bytes TX que la ISR envia cuando se publican.
static volatile uint8_t uart_tx_cabeza = 0U, uart_tx_cola = 0U, uart_tx_iniciada = 0U; // Volatile: limite de datos TX publicados por main; siguiente byte TX para la ISR; indica si ya se comenzo una transmision.
static char uart_linea[UART_LINEA_TAM];                                  // Reserva: trama recibida antes del despacho.
static uint8_t uart_linea_longitud = 0U, uart_descartar_linea = 0U;      // Reserva: caracteres almacenados entre @ y LF; texto fuera de una trama reconocida.
static uint32_t uart_inicio_linea = 0UL;                                 // Marca del ultimo byte recibido dentro de la trama.

/* Actividad 4: parser y diagnostico. No se usa memoria dinamica. */
static uint8_t uart_en_trama = 0U, uart_trama_pendiente = 0U;             // Reserva: se recibio @ y falta fin de linea; trama completa pendiente.
static uint8_t modo_led = LED_AUTOMATICO, solicitud_sensores = 0U, sensores_listos = 0U; // Reserva: control automatico o forzado de LED; cliente espera resultado diferido; el resultado diferido ya puede transmitirse.
static uint8_t dht_intento_previo = 0U;                                  // Reserva: permite aplicar la separacion minima entre intentos.
static uint32_t dht_ultimo_inicio_ms = 0UL, dht_ultima_lectura_ms = 0UL; // Reserva: marca del ultimo inicio de adquisicion DHT; marca del ultimo resultado DHT valido.
static uint16_t protocolo_aceptadas = 0U, protocolo_errores = 0U, uart_tx_descartadas = 0U; // Reserva: cantidad de ordenes aceptadas; rechazos/errores de protocolo; tramas que no se pudieron publicar.
typedef struct {                                                         // Agrupa los campos del registro o constructor sin memoria dinamica.
    uint32_t numero, tiempo;                                             // ID y marca de reloj logico del evento almacenado.
    uint8_t causa;                                                       // Origen de alarma: local, remota, WDT o marca tras bloqueo.
} alarma_evento_t;                                                       // Tipo de cada una de las cuatro entradas del historial.
static alarma_evento_t alarmas[ALARMAS_TAM];                             // Reserva: historial circular de cuatro eventos en SRAM.
static uint32_t alarmas_total = 0UL;                                     // Reserva: total de alarmas desde el arranque.
static uint8_t alarmas_cabeza = 0U, alarmas_guardadas = 0U;              // Reserva: siguiente posicion del historial para escribir; registros actualmente disponibles, maximo cuatro.

/* Diagnostico PRR en SRAM; no es una medicion de consumo. */
static uint8_t twi_fase = TWI_LIBRE;                                     // Reserva: ocupacion logica del controlador I2C.
static volatile uint8_t prr_ultimo_error = PRR_OK;                       // Volatile: ultimo motivo de rechazo PRR, conservado tras cambios exitosos.
static volatile uint16_t prr_rechazos = 0U, prr_cambios = 0U;            // Volatile: contador de perfiles rechazados; contador de escrituras PRR distintas.
static volatile uint8_t prr_ultimo_sueno = 0U;                           // Volatile: copia de PRR tomada justo antes de dormir.
/* UART se inicializa antes de la definicion de la interfaz PRR. */
static void prr_habilitar(uint8_t mascara);                              // Declara la interfaz: Borra bits PRR para devolver reloj; el driver configura luego el periferico.

/* Causa de reset: .noinit evita que la inicializacion de C borre esta copia. */
uint8_t causa_reinicio __attribute__((section(".noinit")));              // Reserva: copia temprana de MCUSR que .noinit conserva.
/* Solo indica que habia un bloqueo de ensayo pendiente; NO prueba su causa. */
uint32_t marca_bloqueo __attribute__((section(".noinit")));              // Reserva: firma del ensayo de bloqueo en RAM .noinit.
uint32_t marca_bloqueo_inversa __attribute__((section(".noinit")));      // Reserva: complemento de la firma para validar su integridad.
#define MARCA_BLOQUEO_WDT 0x57445432UL                                   // Firma RAM del ensayo; por si sola no demuestra la causa del reset.
static volatile uint8_t bandera_wdt = 0U;                                // Volatile: despertar WDT pendiente de atender.
static volatile uint32_t eventos_wdt = 0UL;                              // Volatile: contador de interrupciones WDT.
static uint8_t wdt_armado = 0U;                                          // Reserva: evita reiniciar el plazo de 8 s en cada vuelta.

/* Interfaces de las pruebas; sus implementaciones se seleccionan al final. */
static void wdt_iniciar_prueba(void);                                    // Declara la interfaz: Inicializa la estrategia WDT seleccionada mediante comentarios.
static void wdt_antes_powerdown(void);                                   // Declara la interfaz: Coordina el WDT con la entrada intencional al sueño profundo.
static void wdt_despues_powerdown(void);                                 // Declara la interfaz: Mantiene o reanuda la estrategia WDT tras despertar.
static void wdt_al_volver_a_ejecucion(void);                             // Declara la interfaz: Coordina el regreso por usuario sin mezclar las dos pruebas WDT.
static void wdt_procesar_evento(void);                                   // Declara la interfaz: Atiende el evento WDT desde main, nunca dentro de la ISR.
static void wdt_supervisar_ciclo(void);                                  // Declara la interfaz: En prueba 2 atiende el WDT solo despues de una vuelta completa de main.
static uint8_t wdt_solicitar_bloqueo(void);                              // Rechaza el ensayo salvo que este seleccionada la prueba de reset.

/* Antes de .data/.bss: guardar MCUSR y detener el WDT que persista tras reset. */
void wdt_capturar_reset_temprano(void)                                   // Guarda la causa del reset y detiene WDT antes de inicializar las variables de C.
    __attribute__((naked, used, section(".init3")));                     // .init3 corre antes de C; naked evita prologo y no debe retornar con RET.

void wdt_capturar_reset_temprano(void) {                                 // Guarda la causa del reset y detiene WDT antes de inicializar las variables de C.
    causa_reinicio = MCUSR;                                              // Conserva los flags de reset antes de que la inicializacion pueda alterarlos.
    MCUSR = 0U;                    /* Limpiar WDRF ANTES de deshabilitar WDT. */
    wdt_disable();                 /* La macro respeta la secuencia protegida. */
    /* Sin return: .init3 continua con la inicializacion normal de C. */
}

/* TEXTOS FLASH: implementaciones locales, sin funciones _P externas. */
static char *texto_copiar_P(char *destino, const char *origen_P) {        // Copia una cadena Flash a SRAM, incluido su terminador cero.
    char *inicio = destino;                                            // Conserva el puntero RAM inicial para devolverlo al terminar.
    uint8_t c;                                                         // Guarda el byte leido mediante LPM, no mediante acceso RAM.
    do {                                                               // Se copia tambien el cero final, incluso si la cadena esta vacia.
        c = pgm_read_byte(origen_P++);                                 // Lee un byte de Flash y avanza a su siguiente direccion.
        *destino++ = (char)c;                                          // Escribe ese byte en SRAM y avanza el destino.
    } while (c != 0U);                                                 // Termina despues de copiar el caracter nulo.
    return inicio;                                                     // Devuelve un puntero al buffer RAM, sin conversion entre espacios.
}

static char *texto_anexar_P(char *destino, const char *origen_P) {        // Agrega una cadena Flash al final del texto RAM existente.
    char *fin = destino;                                               // Comienza a buscar el terminador del destino.
    while (*fin != '\0') fin++;                                        // Avanza solo dentro de la cadena RAM ya construida.
    (void)texto_copiar_P(fin, origen_P);                                // Sustituye el cero por el texto nuevo y copia su cero final.
    return destino;                                                    // Conserva el contrato de devolver el inicio del destino.
}

static int texto_comparar_P(const char *ram, const char *flash) {         // Compara bytes sin signo entre una cadena SRAM y otra Flash.
    uint8_t a, b;                                                      // Dos bytes de los respectivos espacios de memoria.
    do {                                                               // Evalua hasta encontrar diferencia o el terminador comun.
        a = (uint8_t)*ram++;                                           // Lectura normal desde SRAM.
        b = pgm_read_byte(flash++);                                    // Lectura especifica desde Flash.
        if (a != b) return (int)a - (int)b;                             // Cero solo significa igualdad; diferencias conservan su signo.
    } while (a != 0U);                                                 // Si ambos bytes fueron cero, las cadenas son iguales.
    return 0;                                                          // ES utiliza este resultado para aceptar comando o dato exacto.
}

/* INTERRUPCIONES */

ISR(TIMER0_COMPA_vect) {
    tiempo_ms++;                                                         // Suma un milisegundo del reloj rapido o el compare pendiente al detenerlo.
    contador_led_ms++;                                                   // Acumula un tick de 1 ms para el parpadeo.
    if (contador_led_ms >= PERIODO_LED_MS) {                             // Al cumplir 200 ms, agenda una conmutacion del LED y reinicia su contador.
        contador_led_ms = 0U;                                            // Reinicia la fase de ese periodo despues de producir su bandera.
        bandera_led = 1U;                                                // Solicita: trabajo LED pendiente para main.
    }
    contador_lcd_ms++;                                                   // Acumula un tick de 1 ms para el refresco de LCD.
    if (contador_lcd_ms >= PERIODO_LCD_MS) {                             // Al cumplir 500 ms, agenda el refresco de pantalla.
        contador_lcd_ms = 0U;                                            // Reinicia la fase de ese periodo despues de producir su bandera.
        bandera_lcd = 1U;                                                // Solicita: actualizacion de pantalla pendiente.
    }
    contador_dht_ms++;                                                   // Acumula un tick de 1 ms para programar la siguiente lectura DHT.
    if (contador_dht_ms >= PERIODO_DHT_MS) {                             // Al cumplir 2000 ms, permite pedir la siguiente adquisicion DHT.
        contador_dht_ms = 0U;                                            // Reinicia la fase de ese periodo despues de producir su bandera.
        bandera_dht = 1U;                                                // Solicita: solicitud de iniciar DHT.
    }
    /* Cambio cada 1 ms: periodo de 2 ms, frecuencia de 500 Hz. */
    if (alarma_continua || tiempo_sonido_ms > 0U) {                      // Genera sonido si hay alarma o aun queda duracion de aviso.
        PORTB ^= (1U << SOUNDER_PIN);                                    // Conmuta el sounder cada 1 ms: onda de 500 Hz.
        if (!alarma_continua && tiempo_sonido_ms > 0U) {                 // El aviso expira; la alarma continua no se descuenta.
            tiempo_sonido_ms--;                                          // Descuenta 1 ms al aviso temporal, nunca al sonido continuo.
        }
    } else {
        PORTB &= (uint8_t)~(1U << SOUNDER_PIN);                          // Fuerza el pin del sounder a nivel bajo.
    }
}

ISR(TIMER1_COMPA_vect) {
    if (modo_reloj == RELOJ_STANDBY) {                                   // En STANDBY, Timer1 reemplaza la base de tiempo de Timer0.
        tiempo_ms += PASO_STANDBY_MS; /* Mantiene la inactividad en IDLE. */
        bandera_reloj = 1U;                                              // Solicita: tick de STANDBY pendiente.
    } else if (modo_reloj == RELOJ_RAPIDO) {                             // En ACTIVO este compare solo agenda el muestreo ADC; no lo ejecuta en ISR.
        bandera_adc = 1U;             /* Funcion original de A en ACTIVO. */
    }
}

ISR(WDT_vect) {
    bandera_wdt = 1U;              /* Solo notifica; ADC y UART se atienden en main. */
    eventos_wdt++;                                                       // Cuenta el despertar; no suma 8000 ms ficticios a tiempo_ms.
    PORTB ^= _BV(WDT_PRUEBA_PIN);                                        // Cada flanco de D12 representa un despertar WDT.
}

ISR(INT0_vect) {
    bandera_boton = 1U;                                                  // Solicita: pulsacion pendiente de validar.
    if (en_powerdown) {                                                  // En sueño profundo, un nivel bajo sostenido no debe repetir indefinidamente INT0.
        /* El nivel bajo podria repetir la IRQ mientras se mantiene pulsado. */
        EIMSK &= (uint8_t)~_BV(INT0);                                    // Enmascara INT0 mientras se cambia el modo o se atiende nivel mantenido.
    }
}

ISR(USART_RX_vect) {
    uint8_t estado_uart = UCSR0A;     /* Leer errores ANTES de UDR0. */
    uint8_t dato = UDR0;                                                 // Consume el byte recibido; el estado de error ya se leyo antes de este acceso.
    uint8_t siguiente = (uint8_t)((uart_rx_cabeza + 1U) & (UART_RX_TAM - 1U)); // Calcula el proximo indice RX para comprobar si el buffer quedaria lleno.
    uint8_t error = 0U;                                                  // Reserva: resultado o mascara de la validacion actual.
    if ((estado_uart & _BV(FE0)) != 0U) error |= UART_ERR_FRAME;         // Traduce el flag del hardware a la mascara de error del protocolo.
    if ((estado_uart & _BV(DOR0)) != 0U) error |= UART_ERR_OVERRUN;      // Traduce el flag del hardware a la mascara de error del protocolo.
    if ((estado_uart & _BV(UPE0)) != 0U) error |= UART_ERR_PARIDAD;      // Traduce el flag del hardware a la mascara de error del protocolo.
    if (siguiente == uart_rx_cola) error |= UART_ERR_BUFFER;             // Detecta lleno antes de sobrescribir bytes RX pendientes.
    if (error != 0U) {                                                   // Descarta el byte recibido si hay error hardware o overflow del buffer.
        uart_rx_error |= error;     /* No valida ni ejecuta comandos en la ISR. */
    } else {
        uart_rx[uart_rx_cabeza] = dato;                                  // Deposita el byte valido antes de publicar la nueva cabeza RX.
        uart_rx_cabeza = siguiente;                                      // Publica en RX el byte que la ISR ya almaceno.
    }
}

ISR(USART_UDRE_vect) {
    if (uart_tx_cola != uart_tx_cabeza) {                                // Solo consume posiciones TX que main publico.
        UCSR0A = _BV(TXC0);          /* Limpia TXC: aun falta transmitir. */
        UDR0 = uart_tx[uart_tx_cola];                                    // Entrega un byte al transmisor hardware.
        uart_tx_cola = (uint8_t)((uart_tx_cola + 1U) & (UART_TX_TAM - 1U)); // Avanza el consumo TX con vuelta modulo 256.
        uart_tx_iniciada = 1U;                                           // Permite exigir TXC antes de declarar terminada una transmision real.
    } else {
        UCSR0B &= (uint8_t)~_BV(UDRIE0);                                 // Evita IRQ continuas de TX vacio cuando no hay mas bytes.
    }
}

/* TIEMPO Y SALIDAS */

static uint32_t leer_tiempo(void) {                                      // Obtiene una copia atomica del reloj de 32 bits; Power-Down no queda contado.
    uint8_t sreg = SREG;                                                 // Reserva: copia de SREG para restaurar el estado previo de interrupciones.
    uint32_t copia;                                                      // Reserva: copia atomica del reloj para devolver al llamador.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    copia = tiempo_ms;                                                   // Lee los 32 bits con IRQ bloqueadas para evitar mezclar dos instantes.
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
    return copia;                                                        // Entrega la copia coherente y permite que el llamador compare plazos.
}

static void configurar_sonido(uint16_t duracion_ms, uint8_t continua) {  // Configura duracion y alarma en una seccion atomica compartida por las salidas.
    uint8_t sreg = SREG;                                                 // Reserva: copia de SREG para restaurar el estado previo de interrupciones.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    tiempo_sonido_ms = duracion_ms;                                      // Actualiza duracion restante del aviso temporal.
    alarma_continua = continua;                                          // Actualiza habilitacion del sonido continuo de SEGURO.
    if (!continua) PORTB &= (uint8_t)~_BV(SOUNDER_PIN);                  // Fuerza el pin del sounder a nivel bajo.
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
}
static void iniciar_sonido(uint16_t duracion_ms) { configurar_sonido(duracion_ms, 0U); } // Solicita un aviso temporal; Timer0 genera los cambios de nivel.

static void iniciar_alarma(void) { configurar_sonido(0U, 1U); }          // Solicita sonido continuo hasta el reconocimiento local de la emergencia.

static void apagar_sonido(void) { configurar_sonido(0U, 0U); }           // Cancela el aviso/alarma y deja el pin del sounder en cero.

/* ACTIVIDAD 4 - UART 9600 8N1 Y TRANSMISION DE TRAMAS
 * Las ISR solo trasladan bytes. Emisor y parser se ejecutan en main. */

static void uart_init(void) {                                            // Devuelve reloj a USART y configura 9600 bps, 8N1 y recepcion por interrupcion.
    prr_habilitar(_BV(PRUSART0));                                        // Devuelve el reloj de los modulos indicados antes de acceder a sus registros.
    PORTD |= _BV(PD0);              /* RX definido cuando no hay modulo. */
    UCSR0A = 0U;                    /* Velocidad normal, U2X0=0. */
    UBRR0H = (uint8_t)(UART_UBRR_9600 >> 8);                             // Carga la parte alta del divisor calculado.
    UBRR0L = (uint8_t)UART_UBRR_9600; /* 103 a 16 MHz; formato 8N1. */
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8 bits, sin paridad, 1 stop. */
    UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);                      // Habilita RX, TX e IRQ RX; UDRE se arma al publicar datos.
}

static char protocolo_hex(uint8_t valor) {                               // Convierte un nibble de 4 bits en un caracter hexadecimal ASCII.
    valor &= 15U;                                                        // Conserva solo los cuatro bits de un nibble.
    return valor < 10U ? (char)('0' + valor) : (char)('A' + valor - 10U); // 0..9 producen 0..9; 10..15 producen A..F.
}

static uint8_t uart_tx_libre(void) {                                     // Calcula espacio libre modulo 256; una posicion queda reservada.
    return (uint8_t)(uart_tx_cola - uart_tx_cabeza - 1U);                // Resta indices modulo 256 y deja una posicion libre para distinguir lleno/vacio.
}

/* Solo main produce TX; la ISR consume exclusivamente los bytes publicados. */
typedef struct {                                                         // Agrupa los campos del registro o constructor sin memoria dinamica.
    uint8_t cabeza, usados, libres;                                      // Reserva: indice TX privado, todavia no visible para la ISR; bytes de trama ya preparados; espacio libre de TX al comenzar.
    uint8_t checksum;                                                    // Reserva: XOR de los caracteres del cuerpo.
} uart_constructor_t;                                                    // Tipo privado del constructor; no contiene una segunda copia de la trama.

/* Se integra en el emisor para evitar una llamada de funcion por cada byte. */
static inline __attribute__((always_inline))                             // Pide integrar el helper para no pagar una llamada por cada byte.
uint8_t uart_anexar_cuerpo(void *contexto, uint8_t dato) {       // Agrega un byte y su XOR sin publicar aun la trama para la ISR.
    uart_constructor_t *tx = (uart_constructor_t *)contexto; // Evita que el preprocesador .ino necesite conocer el typedef en su prototipo.
    /* Reservar siempre *HH, CR y LF. No publicar una trama incompleta. */
    if (tx->usados >= PROTO_TRAMA_TAM - 6U ||                            // Comprueba limite de trama y espacio libre antes de agregar otro byte.
        (uint16_t)tx->usados + 6U > tx->libres) return 0U;               // Reserva el byte actual mas los cinco bytes finales.
    uart_tx[tx->cabeza++] = dato; /* TX=256: el indice uint8_t vuelve a cero. */
    tx->usados++;                                                        // Avanza la longitud preparada, sin cambiar la cabeza global.
    tx->checksum ^= dato;                                                // XOR del byte actual con el acumulado, no una suma aritmetica.
    return 1U;                                                           // Devuelve verdadero: la operacion termino correctamente.
}

/* Comando en Flash; dato en RAM o Flash segun dato_flash. Sin trama temporal. */
static uint8_t uart_enviar_trama_fuente(const char *comando_flash,       // Construye @COMANDO,DATO*HH/LF directamente en TX y publica al completar.
                                      const char *dato, uint8_t dato_flash) {
    uart_constructor_t tx;                                               // Reserva: constructor local de una trama aun no publicada.
    uint8_t c, sreg;                                                     // Reserva: caracter leido desde RAM o Flash; copia de SREG para restaurar el estado previo de interrupciones.
    tx.cabeza = uart_tx_cabeza;                                          // Toma una cabeza privada; la ISR aun no ve estos bytes.
    tx.usados = 1U;                                                      // Reserva la cuenta del delimitador @.
    tx.libres = uart_tx_libre();                                         // Toma una cota de espacio; la ISR solo puede liberar mas.
    tx.checksum = 0U;                                                    // Inicializa XOR; @ y el sufijo *HH/LF quedan excluidos.
    if (tx.libres < 6U) goto rechazar;                                   // Rechaza si ni siquiera caben los delimitadores y el checksum.
    uart_tx[tx.cabeza++] = '@';                                          // Escribe START sin hacerlo visible aun a la ISR.
    while ((c = pgm_read_byte(comando_flash++)) != 0U) {                 // Lee el comando constante desde memoria de programa.
        if (!uart_anexar_cuerpo(&tx, c)) goto rechazar;                  // Un fallo aborta toda la trama: la cabeza global todavia no se publico.
    }
    if (!uart_anexar_cuerpo(&tx, ',')) goto rechazar;                    // Un fallo aborta toda la trama: la cabeza global todavia no se publico.
    for (;;) {                                                           // Recorre el dato hasta su terminador o un rechazo.
        c = dato_flash ? pgm_read_byte(dato) : (uint8_t)*dato;           // Elige lectura con LPM desde Flash o lectura normal desde RAM.
        dato++;                                                          // Avanza el puntero al siguiente caracter del dato.
        if (c == 0U || c == '\r' || c == '\n') break;                    // Termina el dato en cero, CR o LF; esos bytes no participan del XOR.
        if (c == '@' || c == '*' || !uart_anexar_cuerpo(&tx, c)) goto rechazar; // Rechaza delimitadores dentro del dato o falta de espacio sin publicar parciales.
    }
    uart_tx[tx.cabeza++] = '*';                                          // Marca donde termina el cuerpo y comienza el checksum.
    uart_tx[tx.cabeza++] = protocolo_hex(tx.checksum >> 4);              // Escribe el nibble alto del XOR como digito hexadecimal.
    uart_tx[tx.cabeza++] = protocolo_hex(tx.checksum);                   // Escribe el nibble bajo del XOR como digito hexadecimal.
    uart_tx[tx.cabeza++] = '\r';                                         // Retorno de carro para terminales como la de Proteus.
    uart_tx[tx.cabeza++] = '\n';                                         // Salto de linea; desde aqui la trama ya esta completa.
    sreg = SREG;                                                         // Actualiza copia de SREG para restaurar el estado previo de interrupciones.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    uart_tx_cabeza = tx.cabeza; /* Publicar solo despues del checksum y el fin. */
    UCSR0B |= _BV(UDRIE0);                                               // Activa la ISR de registro TX vacio para drenar el buffer.
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
    return 1U;                                                           // Devuelve verdadero: la operacion termino correctamente.
rechazar:                                                                // Punto comun de salida: la cabeza publicada no se modifico.
    /* Los bytes escritos en espacio libre siguen invisibles para la ISR. */
    uart_tx_descartadas++;                                               // Cuenta un intento que no pudo publicar una trama completa.
    return 0U;                                                           // Devuelve falso/fallo o ausencia de evento al llamador.
}

static uint8_t uart_enviar_trama(const char *comando_flash, const char *dato_ram) { // Envuelve el emisor: comando en Flash y datos variables en RAM.
    return uart_enviar_trama_fuente(comando_flash, dato_ram, 0U);        // Encola una trama completa; main continua mientras la ISR transmite.
}

static uint8_t uart_enviar_trama_P(const char *comando_flash, const char *dato_flash) { // Envuelve el emisor cuando tanto comando como dato residen en Flash.
    return uart_enviar_trama_fuente(comando_flash, dato_flash, 1U);      // Encola una trama completa; main continua mientras la ISR transmite.
}

static uint8_t uart_enviar_texto(const char *texto_ram) {                // Envia como EVENTO un mensaje preparado en RAM.
    return uart_enviar_trama(PSTR("EVENTO"), texto_ram);                 // Encola una trama completa; main continua mientras la ISR transmite.
}

static uint8_t uart_enviar_texto_P(const char *texto_flash) {            // Envia como EVENTO un literal de Flash, ahorrando SRAM.
    return uart_enviar_trama_P(PSTR("EVENTO"), texto_flash);             // Encola una trama completa; main continua mientras la ISR transmite.
}

static void protocolo_error(const char *motivo_flash) {                  // Registra un rechazo y emite su motivo en una trama ERROR.
    protocolo_errores++;                                                 // Cuenta el rechazo/error que se informara al cliente.
    (void)uart_enviar_trama_P(PSTR("ERROR"), motivo_flash);              // Encola una trama completa; main continua mientras la ISR transmite.
}

static void texto_agregar_u32(char *texto, uint32_t valor) {             // Anexa un entero decimal al texto sin usar printf ni memoria dinamica.
    char digitos[10];                                                    // Reserva: digitos temporales en orden inverso.
    uint8_t n = 0U;                                                      // Reserva: cantidad de digitos almacenados.
    uint8_t i = (uint8_t)strlen(texto);                                  // Reserva: indice de recorrido.
    do {
        digitos[n++] = (char)('0' + valor % 10UL);                       // Extrae el digito menos significativo y lo guarda temporalmente.
        valor /= 10UL;                                                   // Elimina el digito decimal que ya se guardo.
    } while (valor != 0UL);                                              // Incluye un digito incluso si el numero original era cero.
    while (n != 0U) texto[i++] = digitos[--n];                           // Reordena los digitos al copiarlos al final del texto.
    texto[i] = '\0';                                                     // Termina la cadena decimal formada.
}

static const char *nombre_estado_P(void) {                               // Devuelve el nombre del estado desde Flash.
    if (estado == ESTADO_ACTIVO) return PSTR("ACTIVO");                  // Nombre ACTIVO almacenado en memoria de programa.
    if (estado == ESTADO_STANDBY) return PSTR("STANDBY");                // Nombre STANDBY almacenado en memoria de programa.
    if (estado == ESTADO_APAGADO) return PSTR("APAGADO");                // Nombre APAGADO almacenado en memoria de programa.
    return PSTR("SEGURO");                                               // El estado restante es SEGURO; tambien se devuelve su nombre desde Flash.
}

static void registrar_alarma(uint8_t causa) {                            // Guarda ID, causa y reloj logico en el historial circular de cuatro eventos.
    alarma_evento_t *registro = &alarmas[alarmas_cabeza];                // Reserva: puntero al evento de alarma que se escribe/consulta.
    alarmas_total++;                                                     // Asigna un nuevo ID de alarma dentro de este arranque.
    registro->numero = alarmas_total;                                    // Guarda el ID del evento en esta posicion del historial.
    registro->tiempo = leer_tiempo(); /* Reloj de firmware: excluye Power-Down. */
    registro->causa = causa;                                             // Distingue emergencia local/remota y recuperacion tras reset.
    alarmas_cabeza = (uint8_t)((alarmas_cabeza + 1U) % ALARMAS_TAM);     // Avanza circularmente entre las cuatro posiciones del historial.
    if (alarmas_guardadas < ALARMAS_TAM) alarmas_guardadas++;            // Verifica el estado/capacidad del registro de alarmas.
}

static const char *nombre_alarma_P(uint8_t causa) {                      // Devuelve desde Flash el nombre de la causa registrada.
    if (causa == ALARMA_LOCAL) return PSTR("LOCAL");                     // Traduce a texto la causa de pulsacion prolongada.
    if (causa == ALARMA_REMOTA) return PSTR("REMOTA");                   // Traduce a texto una emergencia recibida por trama valida.
    if (causa == ALARMA_WDT) return PSTR("WDT");                         // Indica WDT solo cuando esta disponible WDRF.
    return PSTR("TRAS_BLOQUEO");                                         // Marca sin WDRF: informa contexto de bloqueo sin inventar causa de reset.
}

static uint8_t uart_tomar_byte(uint8_t *dato) {                          // Extrae un byte RX sin esperar; devuelve cero si no hay datos.
    if (uart_rx_cola == uart_rx_cabeza) return 0U;                       // Buffer RX vacio: no bloquea esperando un byte.
    *dato = uart_rx[uart_rx_cola];                                       // Devuelve al llamador el byte situado en la cola RX.
    uart_rx_cola = (uint8_t)((uart_rx_cola + 1U) & (UART_RX_TAM - 1U));  // Avanza el consumo RX o descarta sus datos si se detecto perdida.
    return 1U;                                                           // Devuelve verdadero: la operacion termino correctamente.
}

static uint8_t uart_tx_terminada(void) {                                 // Exige buffer vacio y ultimo byte fisicamente transmitido antes de dormir.
    return (uart_tx_cola == uart_tx_cabeza &&                            // No basta vaciar el buffer: tambien debe haber salido el ultimo bit por TX.
            (!uart_tx_iniciada || (UCSR0A & _BV(TXC0)) != 0U)) ? 1U : 0U; // Si se transmitio algun byte, exige TXC=1 antes de permitir el sueño.
}

static void uart_informar_estado(void) {                                 // Construye la respuesta de estado, PRR y emergencia.
    char respuesta[64];                                                  // Reserva: buffer RAM de respuesta de protocolo.
    COPIAR(respuesta, "ESTADO;EST=");                                    // Copia la etiqueta literal desde Flash al buffer RAM.
    texto_anexar_P(respuesta, nombre_estado_P());                              // Anexa el campo/texto de Flash al mensaje RAM.
    texto_anexar_P(respuesta, PSTR(";PRR="));                                  // Anexa el campo/texto de Flash al mensaje RAM.
    {
        uint8_t n = (uint8_t)strlen(respuesta);                          // Reserva: cantidad de digitos almacenados.
        respuesta[n++] = protocolo_hex(PRR >> 4);                        // Agrega un digito hexadecimal del registro o error.
        respuesta[n++] = protocolo_hex(PRR);                             // Agrega un digito hexadecimal del registro o error.
        respuesta[n] = '\0';                                             // Cierra la cadena con cero antes de enviarla.
    }
    texto_anexar_P(respuesta, PSTR(";EMERG="));                                // Anexa el campo/texto de Flash al mensaje RAM.
    texto_agregar_u32(respuesta, estado == ESTADO_SEGURO ? 1U : 0U);     // Anexa el numero decimal usando el formateador compartido.
    (void)uart_enviar_trama(PSTR("OK"), respuesta);                      // Encola una trama completa; main continua mientras la ISR transmite.
}

/* ACTIVIDAD 3 - PRR: VALIDAR, DETENER Y DESPUES QUITAR EL RELOJ
 * Todas las escrituras a PRR pasan por estas funciones.
 * Las ISR no cambian PRR. Las comprobaciones y el cambio son atomicos. */

static void prr_registrar_error(uint8_t codigo) {                        // Conserva el ultimo motivo de rechazo y aumenta su contador.
    prr_ultimo_error = codigo;                                           // Actualiza ultimo motivo de rechazo PRR, conservado tras cambios exitosos.
    prr_rechazos++;                                                      // Incrementa el contador de rechazos del perfil, util para diagnostico.
}

/* Solo cambia el registro; el llamador ya verifico y preparo los modulos. */
static uint8_t prr_escribir(uint8_t valor) {                             // Escribe solo bits permitidos y verifica la lectura posterior del registro.
    valor &= PRR_VALIDOS;                                                // Excluye el bit 4 reservado y cualquier bit no definido.
    if (PRR != valor) {                                                  // Distingue cambio real del registro o fallo al verificar su lectura.
        PRR = valor;                                                     // Aplica el perfil: bit a uno quita reloj, cero lo devuelve.
        prr_cambios++;                                                   // Cuenta un cambio real de PRR; repetir el mismo valor no lo incrementa.
    }
    if (PRR != valor) {                                                  // Distingue cambio real del registro o fallo al verificar su lectura.
        prr_registrar_error(PRR_ERR_LECTURA);                            // Deja disponible el motivo del rechazo para diagnostico.
        return 0U;                                                       // Devuelve falso/fallo o ausencia de evento al llamador.
    }
    return 1U;                                                           // Devuelve verdadero: la operacion termino correctamente.
}

/* Quitar el bit PRR habilita el RELOJ; el driver configura luego el modulo. */
static void prr_habilitar(uint8_t mascara) {                             // Borra bits PRR para devolver reloj; el driver configura luego el periferico.
    uint8_t sreg = SREG;                                                 // Reserva: copia de SREG para restaurar el estado previo de interrupciones.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    if ((mascara & (uint8_t)~PRR_VALIDOS) != 0U) {                       // No acepta escribir el bit reservado ni una mascara ajena al ATmega328P.
        prr_registrar_error(PRR_ERR_BIT);                                // Deja disponible el motivo del rechazo para diagnostico.
    } else {
        (void)prr_escribir((uint8_t)(PRR & (uint8_t)~mascara));          // Escribe solo bits permitidos y verifica la lectura posterior del registro.
    }
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
}

/* Retorna un motivo de rechazo SIN detener ningun periferico. */
static uint8_t prr_validar(uint8_t destino) {                            // Rechaza bits reservados, relojes necesarios y perifericos ocupados.
    uint8_t nuevos = destino & (uint8_t)~PRR;                            // Reserva: bits que se apagaran ahora y aun tenian reloj.
    if ((destino & (uint8_t)~PRR_VALIDOS) != 0U) return PRR_ERR_BIT;     // Se solicito un bit reservado/no definido.
    if (((destino & PRR_TIMER0) != 0U && modo_reloj == RELOJ_RAPIDO) ||  // Protege Timer0 del reloj rapido y Timer1 cuando lo necesita la planificacion.
        ((destino & PRR_TIMER1) != 0U &&
         (modo_reloj == RELOJ_STANDBY ||
          (estado == ESTADO_ACTIVO && modo_reloj != RELOJ_DETENIDO)))) return PRR_ERR_RELOJ;       /* Scheduler/ADC de ACTIVO o reloj de IDLE. */
    if ((destino & PRR_TIMER2) != 0U &&                                  // Impide apagar Timer2 mientras una captura DHT lo utiliza.
        estado_dht >= DHT_ESPERA_RESPUESTA_BAJA && estado_dht <= DHT_BIT_ALTO) return PRR_ERR_DHT; // Timer2 esta capturando datos del sensor.
    if ((nuevos & PRR_ADC) != 0U && (ADCSRA & _BV(ADSC)) != 0U) return PRR_ERR_ADC; // ADC aun esta convirtiendo.
    if ((nuevos & PRR_USART) != 0U &&                                    // No apaga UART con datos pendientes, trama parcial o fuera de APAGADO.
        (estado != ESTADO_APAGADO || !uart_tx_terminada() ||
         uart_rx_cabeza != uart_rx_cola || uart_rx_error ||
         uart_linea_longitud != 0U || uart_en_trama || uart_trama_pendiente ||
         (UCSR0A & _BV(RXC0)) != 0U)) return PRR_ERR_UART;               // UART esta ocupada o debe conservarse.
    if ((nuevos & PRR_TWI) != 0U) {                                      // Antes de apagar TWI, comprueba que el controlador no conserve una operacion.
        if (twi_fase == TWI_STOP_PENDIENTE && (TWCR & _BV(TWSTO)) == 0U) { // No permite quitar reloj TWI hasta finalizar transferencia y STOP.
            twi_fase = TWI_LIBRE;  /* STOP termino despues de su espera limitada. */
        }
        if (twi_fase != TWI_LIBRE || (TWCR & _BV(TWSTO)) != 0U) return PRR_ERR_TWI; // I2C tiene transferencia o STOP pendiente.
    }
    /* SPI no se usa: si otro desarrollo lo habilita, no apagarlo a ciegas. */
    if ((nuevos & PRR_SPI) != 0U && (SPCR & _BV(SPE)) != 0U) return PRR_ERR_SPI; // SPI fue habilitado y no se apaga a ciegas.
    return PRR_OK;                                                       // No se encontro ningun motivo para impedir el perfil.
}

static uint8_t prr_fijar_perfil(uint8_t destino) {                       // Valida todo, detiene los modulos nuevos y aplica PRR de forma atomica.
    uint8_t sreg = SREG, nuevos, error;                                  // Reserva: copia de SREG para restaurar el estado previo de interrupciones; bits que se apagaran ahora y aun tenian reloj; resultado o mascara de la validacion actual.
    uint8_t resultado;                                                   // Reserva: exito/fallo que se devolvera.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    error = prr_validar(destino);                                        // Actualiza resultado o mascara de la validacion actual.
    if (error != PRR_OK) {                                               // Cancela todo el perfil antes de apagar parcialmente algun modulo.
        prr_registrar_error(error);                                      // Deja disponible el motivo del rechazo para diagnostico.
        SREG = sreg;                                                     // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
        return 0U;                 /* Ningun apagado parcial si algo esta ocupado. */
    }
    nuevos = destino & (uint8_t)~PRR;                                    // Identifica solo modulos que aun tenian reloj y se apagaran ahora.
    /* No acceder a registros de modulos cuyo reloj ya estaba quitado. */
    if ((nuevos & PRR_ADC) != 0U) ADCSRA &= (uint8_t)~_BV(ADEN);         // Borra ADEN antes de quitar reloj, evitando cortar una conversion.
    if ((nuevos & PRR_USART) != 0U) UCSR0B = 0U;                         // Deshabilita RX/TX y sus IRQ antes de detener el reloj USART.
    if ((nuevos & PRR_TIMER0) != 0U) { TCCR0B = 0U; TIMSK0 = 0U; }       // Detiene el contador y sus IRQ antes de aplicar el bit de reduccion.
    if ((nuevos & PRR_TIMER1) != 0U) { TCCR1B = _BV(WGM12); TIMSK1 = 0U; } // Detiene el contador y sus IRQ antes de aplicar el bit de reduccion.
    if ((nuevos & PRR_TIMER2) != 0U) { TCCR2B = 0U; TIMSK2 = 0U; }       // Detiene el contador y sus IRQ antes de aplicar el bit de reduccion.
    if ((nuevos & PRR_TWI) != 0U) TWCR = 0U;                             // Deshabilita TWI solo despues de comprobar que no hay operacion pendiente.
    resultado = prr_escribir(destino);                                   // Actualiza exito/fallo que se devolvera.
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
    return resultado;                                                    // Informa si el perfil pudo escribirse y leerse correctamente.
}

static uint8_t prr_deshabilitar(uint8_t mascara) {                       // Agrega bits a PRR pasando por la validacion comun.
    /* Validar primero la mascara original: no ocultar un bit reservado. */
    uint8_t sreg = SREG, resultado;                                      // Reserva: copia de SREG para restaurar el estado previo de interrupciones; exito/fallo que se devolvera.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    resultado = prr_fijar_perfil((uint8_t)(PRR | mascara));              // Aplica el perfil de reloj de este estado despues de preparar los recursos.
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
    return resultado;                                                    // Propaga el resultado de la validacion y aplicacion del perfil.
}

static void prr_informar_valor(const char *etiqueta, uint8_t valor) {    // Enmarca un valor PRR y su error; la etiqueta reside en Flash.
    char mensaje[48];                                                    // Reserva: buffer RAM para construir una notificacion.
    uint8_t i = 4U;                                                      // Reserva: indice de recorrido.
    texto_copiar_P(mensaje, PSTR("PRR:"));                                     // Copia texto desde Flash; no reserva una copia permanente en SRAM.
    uint8_t c;                                                           // Reserva: caracter leido desde RAM o Flash.
    while (i < 30U && (c = pgm_read_byte(etiqueta++)) != 0U) {           // Copia una etiqueta Flash con longitud acotada.
        mensaje[i++] = (char)c;                                          // Agrega el separador o caracter de este campo y avanza su posicion.
    }
    mensaje[i++] = '=';                                                  // Agrega el separador o caracter de este campo y avanza su posicion.
    mensaje[i++] = '0';                                                  // Agrega el separador o caracter de este campo y avanza su posicion.
    mensaje[i++] = 'x';                                                  // Agrega el separador o caracter de este campo y avanza su posicion.
    mensaje[i++] = protocolo_hex(valor >> 4);                            // Agrega un digito hexadecimal del registro o error.
    mensaje[i++] = protocolo_hex(valor);                                 // Agrega un digito hexadecimal del registro o error.
    texto_copiar_P(&mensaje[i], PSTR(",ERR="));                                // Copia texto desde Flash; no reserva una copia permanente en SRAM.
    i += 5U;                                                             // Actualiza indice de recorrido.
    mensaje[i++] = protocolo_hex(prr_ultimo_error);                      // Agrega un digito hexadecimal del registro o error.
    mensaje[i++] = '\r';                                                 // Agrega el fin de linea del mensaje que se encapsulara.
    mensaje[i++] = '\n';                                                 // Agrega el fin de linea del mensaje que se encapsulara.
    mensaje[i] = '\0';                                                   // Cierra la cadena con cero antes de enviarla.
    (void)uart_enviar_texto(mensaje);                                    // Encola aviso como EVENTO con checksum; no transmite esperando en main.
}

static void prr_informar(const char *etiqueta) {                         // Informa el valor actual leido en PRR.
    prr_informar_valor(etiqueta, PRR);                                   // Envia una copia historica si dice ULTIMO_SUENO; no es el PRR actual al transmitir.
}

/* GPIO Y TEMPORIZADORES */

static void gpio_init(void) {                                            // Configura direcciones, pull-ups, INT0 y salidas inicialmente apagadas.
    ACSR = _BV(ACD);                /* Comparador analogico no usado. */
    (void)prr_deshabilitar(PRR_SPI);              /* SPI no se usa en esta aplicacion. */
    DIDR0 |= _BV(ADC0D);            /* Evita entrada digital innecesaria en A0. */
    DDRB |= _BV(DDB5) | _BV(DDB3) | _BV(DDB4);                           // Declara salidas LED, sounder y marca WDT.
    PORTB &= (uint8_t)~(_BV(LED_PIN) | _BV(SOUNDER_PIN) | _BV(WDT_PRUEBA_PIN)); // Deja apagados LED, sounder y marca D12 al configurar las salidas.
    DDRD &= (uint8_t)~(1U << DHT_DDR_PIN);                                      // Libera DATA del DHT: pin como entrada.
    PORTD |= (1U << DHT_PIN);                                            // Habilita pull-up al liberar DATA para que el sensor pueda llevarla a cero.
    DDRD &= (uint8_t)~(1U << DDD2);                                      // Configura el pulsador INT0 como entrada.
    PORTD |= (1U << BOTON_PIN);                                          // Habilita pull-up: sin pulsar se lee uno y al pulsar, cero.
    EICRA = (uint8_t)((EICRA &                                           // Prepara ISC01:00 para flanco descendente, conservando los demas bits.
             (uint8_t)~((1U << ISC01) | (1U << ISC00))) |
             (1U << ISC01));                                             // ISC01=1 e ISC00=0 dejan INT0 por flanco descendente en ejecucion normal.
    EIFR = (1U << INTF0);                                                // Limpia el flag INT0 antes de habilitar su interrupcion.
    EIMSK |= (1U << INT0);                                               // Permite que INT0 genere su interrupcion.
}

static void timer0_init(void) {                                          // Configura CTC de 1 ms: 16 MHz / 64 / 250.
    prr_habilitar(_BV(PRTIM0));                                          // Devuelve el reloj de los modulos indicados antes de acceder a sus registros.
    TIFR0 = _BV(OCF0A) | _BV(OCF0B) | _BV(TOV0);                         // Limpia flags pendientes escribiendo unos antes de rearmar el timer.
    /* 16 MHz / 64 = 250 kHz; 250 cuentas = 1 ms. */
    TCCR0A = (1U << WGM01);                                              // Selecciona Timer0 CTC: reinicio al alcanzar OCR0A.
    TCCR0B = (1U << CS01) | (1U << CS00);                                // Prescaler Timer0=64: 4 us por cuenta a 16 MHz.
    OCR0A = 249U;                                                        // Cuenta 0..249: 250 cuentas por 4 us producen 1 ms.
    TCNT0 = 0U;                                                          // Reinicia la fase del contador Timer0.
    TIMSK0 = (1U << OCIE0A);                                             // Habilita la interrupcion de compare A de Timer0.
}

static void timer1_init(void) {                                          // Configura Timer1 segun ACTIVO o STANDBY.
    prr_habilitar(_BV(PRTIM1));                                          // Devuelve el reloj de los modulos indicados antes de acceder a sus registros.
    TIFR1 = _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);                         // Limpia flags pendientes escribiendo unos antes de rearmar el timer.
    TCCR1A = 0U;                                                         // Desactiva salidas de comparacion; Timer1 se usa como temporizador.
    if (modo_reloj == RELOJ_STANDBY) {
        /* 16 MHz / 256 = 62,5 kHz; 62500 cuentas = 1 segundo. */
        TCCR1B = _BV(WGM12) | _BV(CS12);                                 // CTC y prescaler 256: 16 us por cuenta.
        OCR1A = 62499U;                                                  // Un despertar por segundo en STANDBY.
    } else {
        /* 16 MHz / 64 = 250 kHz; 25000 cuentas = 100 ms. */
        TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10);                    // CTC y prescaler 64: 4 us por cuenta.
        OCR1A = 24999U;                                                  // Agenda el ADC cada 100 ms en ACTIVO.
    }
    TCNT1 = 0U;                                                          // Reinicia el contador de 16 bits antes de comenzar una fase.
    TIMSK1 = (1U << OCIE1A);                                             // Habilita compare A de Timer1.
}

static void timer2_init(void) {                                          // Prepara Timer2 normal para DHT y lo deja sin reloj hasta la captura.
    prr_habilitar(_BV(PRTIM2));                                          // Devuelve el reloj de los modulos indicados antes de acceder a sus registros.
    /* Modo normal, prescaler 8: 0,5 us por cuenta. */
    TCCR2A = 0U;                                                         // Configura Timer2 en modo normal, sin PWM.
    TCCR2B = 0U;                  /* Se arrancara al liberar el pulso DHT. */
    TCNT2 = 0U;                                                          // Pone a cero la medida: el siguiente flanco dara la duracion del nivel.
    TIMSK2 = 0U;                                                         // Timer2 mide tiempo sin ISR propia; los flancos los atiende PCINT2.
    TIFR2 = (1U << TOV2);                                                // Limpia flags pendientes escribiendo unos antes de rearmar el timer.
    (void)prr_deshabilitar(PRR_TIMER2);                                  // Solicita quitar reloj, pasando antes por la validacion de recursos ocupados.
}

/* ADC Y ACTIVIDAD DEL POTENCIOMETRO */

static void adc_habilitar(void) {                                        // Devuelve reloj al ADC, selecciona AVCC/A0 y habilita prescaler 128.
    prr_habilitar(_BV(PRADC));                                           // Devuelve el reloj de los modulos indicados antes de acceder a sus registros.
    ADMUX = (1U << REFS0);                                               // Selecciona A0 y AVCC como referencia; resultado ajustado a la derecha.
    ADCSRA = (1U << ADEN) |                                              // Habilita ADC; la continuacion selecciona divisor 128.
             (1U << ADPS2) | (1U << ADPS1) | (1U << ADPS0);              // ADPS2:0=111: 16 MHz/128=125 kHz para el ADC.
}

static void adc_deshabilitar(void) {                                     // Borra ADEN solamente si el ADC tiene reloj y no esta convirtiendo.
    if ((PRR & PRR_ADC) == 0U && (ADCSRA & _BV(ADSC)) == 0U) {           // No accede al ADC sin reloj ni cancela una conversion en curso.
        ADCSRA &= (uint8_t)~_BV(ADEN);                                   // Borra ADEN antes de quitar reloj, evitando cortar una conversion.
    }
}

static void adc_init(void) {                                             // Configura A0 como entrada analogica y prepara el conversor.
    DDRC &= (uint8_t)~(1U << DDC0);                                      // A0/PC0 es entrada, no una salida digital.
    PORTC &= (uint8_t)~(1U << PORTC0);                                   // Desactiva el pull-up de A0 para no modificar la señal analogica.
    adc_habilitar();                                                     // Devuelve reloj al ADC, selecciona AVCC/A0 y habilita prescaler 128.
}

static uint16_t adc_leer_a0(void) {                                      // Inicia una conversion y espera su fin en main; esta rutina si usa polling.
    if ((PRR & PRR_ADC) != 0U || (ADCSRA & _BV(ADEN)) == 0U) {           // Devuelve reloj y ADEN si el ADC estaba deshabilitado antes de adquirir.
        adc_habilitar();                                                 // Devuelve reloj al ADC, selecciona AVCC/A0 y habilita prescaler 128.
    }
    ADMUX = (1U << REFS0);                                               // Selecciona A0 y AVCC como referencia; resultado ajustado a la derecha.
    ADCSRA |= (1U << ADSC);                                              // Inicia una conversion ADC; ADSC permanece en uno mientras trabaja.
    while ((ADCSRA & (1U << ADSC)) != 0U) {                              // Polling ADC en main; a 16 MHz son 104 us normales o 200 us tras habilitar.
        /* A 125 kHz: 104 us normales; primera tras ADEN, 200 us. */
    }
    return ADC;                                                          // Lee el resultado ADC de 10 bits; avr-gcc ordena el acceso a ambos bytes.
}

static void revisar_actividad_adc(uint16_t nueva_lectura, uint32_t ahora) { // Considera actividad un cambio de A0 mayor o igual a 20 cuentas, si esta habilitado.
#if POTENCIOMETRO_ES_ACTIVIDAD                                           // Incluye deteccion de actividad por A0 solo si la opcion vale uno.
    uint16_t diferencia;                                                 // Reserva: distancia absoluta entre A0 actual y su referencia.
    if (!referencia_adc_valida) {                                        // La primera lectura establece referencia; no se interpreta como movimiento.
        referencia_adc_actividad = nueva_lectura;                        // Establece la nueva referencia para detectar movimientos de A0.
        referencia_adc_valida = 1U;                                      // Evita interpretar la primera adquisicion como actividad.
        return;                                                          // Cede inmediatamente para que main continue con las otras tareas.
    }
    diferencia = (nueva_lectura > referencia_adc_actividad)              // Actualiza distancia absoluta entre A0 actual y su referencia.
        ? (uint16_t)(nueva_lectura - referencia_adc_actividad)
        : (uint16_t)(referencia_adc_actividad - nueva_lectura);          // Resta en el orden correcto para obtener un cambio absoluto sin signo.
    if (diferencia >= UMBRAL_ACTIVIDAD_ADC) {                            // Acepta movimiento de A0 solo cuando supera el umbral configurado.
        referencia_adc_actividad = nueva_lectura;                        // Establece la nueva referencia para detectar movimientos de A0.
        ultima_actividad = ahora;                                        // Guarda una accion valida para los umbrales de 10/30 s.
    }
#else                                                                    // Alternativa compilada cuando no se cumple la condicion anterior.
    (void)nueva_lectura;                                                 // Si A0 no cuenta como actividad, descarta explicitamente este parametro.
    (void)ahora;                                                         // Evita advertencias por el parametro sin uso cuando la opcion esta en cero.
#endif                                                                   // Cierra la condicion del preprocesador; no se ejecuta en tiempo de programa.
}

/* TWI / LCD 16x2 CON PCF8574 */

static uint8_t twi_esperar(void) {                                       // Espera TWINT con limite; evita una espera infinita si falla el bus.
    uint16_t timeout;                                                    // Reserva: contador acotado para no esperar indefinidamente.
    for (timeout = 0U; timeout < 3000U; timeout++) {                     // Espera acotada con contador: si el bus no responde se devuelve fallo.
        if ((TWCR & (1U << TWINT)) != 0U) return 1U;                     // TWINT indica que el controlador termino la accion solicitada.
        _delay_us(1);                                                    // Retardo acotado de 1 us para el bus/LCD; no es un temporizador de tareas.
    }
    return 0U;                                                           // Devuelve falso/fallo o ausencia de evento al llamador.
}

static void twi_init(void) {                                             // Devuelve reloj y configura TWI a 100 kHz antes de escribir la LCD.
    prr_habilitar(_BV(PRTWI));                                           // Devuelve el reloj de los modulos indicados antes de acceder a sus registros.
    DDRC &= (uint8_t)~((1U << DDC4) | (1U << DDC5));                     // Prepara PC4/SDA y PC5/SCL como entradas para el controlador TWI.
    PORTC |= (1U << PORTC4) | (1U << PORTC5);                            // Habilita pull-ups internos en SDA/SCL; las resistencias del montaje se verifican aparte.
    TWSR = 0U;                                                           // Selecciona prescaler TWI=1.
    TWBR = (uint8_t)(((F_CPU / 100000UL) - 16UL) / 2UL);                 // A 16 MHz da 72: SCL nominal de 100 kHz.
    TWCR = (1U << TWEN);                                                 // Habilita el controlador TWI despues de devolverle reloj.
}

static uint8_t twi_iniciar(uint8_t direccion_escritura) {                // Genera START, envia direccion de escritura y comprueba ACK.
    uint8_t estado_twi;                                                  // Reserva: codigo de estado del controlador TWI.
    twi_fase = TWI_TRANSFIRIENDO;                                        // Actualiza la ocupacion I2C que consulta la proteccion PRR.
    TWCR = (1U << TWINT) | (1U << TWSTA) | (1U << TWEN);                 // Borra TWINT y solicita START manteniendo TWI habilitado.
    if (!twi_esperar()) return 0U;                                       // Un timeout del bus devuelve fallo en lugar de esperar indefinidamente.
    estado_twi = TWSR & 0xF8U;                                           // Extrae el codigo TWI ignorando bits bajos de prescaler.
    if (estado_twi != TW_START && estado_twi != TW_REP_START) return 0U; // START debe haber sido aceptado antes de enviar la direccion.
    TWDR = direccion_escritura;                                          // Carga direccion I2C de 7 bits desplazada y bit R/W=0.
    TWCR = (1U << TWINT) | (1U << TWEN);                                 // Borra TWINT para que el hardware envie el contenido de TWDR.
    if (!twi_esperar()) return 0U;                                       // Un timeout del bus devuelve fallo en lugar de esperar indefinidamente.
    return ((TWSR & 0xF8U) == TW_MT_SLA_ACK) ? 1U : 0U;                  // Devuelve exito solo si el esclavo respondio ACK a su direccion de escritura.
}

static uint8_t twi_enviar(uint8_t dato) {                                // Transmite un byte I2C y verifica que el esclavo lo confirme.
    TWDR = dato;                                                         // Carga el byte que se transmitira al esclavo I2C.
    TWCR = (1U << TWINT) | (1U << TWEN);                                 // Borra TWINT para que el hardware envie el contenido de TWDR.
    if (!twi_esperar()) return 0U;                                       // Un timeout del bus devuelve fallo en lugar de esperar indefinidamente.
    return ((TWSR & 0xF8U) == TW_MT_DATA_ACK) ? 1U : 0U;                 // Devuelve exito solo si el esclavo confirmo el byte enviado.
}

static void twi_stop(void) {                                             // Solicita STOP y conserva la ocupacion si el hardware aun no termina.
    uint16_t timeout;                                                    // Reserva: contador acotado para no esperar indefinidamente.
    twi_fase = TWI_STOP_PENDIENTE;                                       // Actualiza la ocupacion I2C que consulta la proteccion PRR.
    TWCR = (1U << TWINT) | (1U << TWEN) | (1U << TWSTO);                 // Solicita STOP; TWSTO se borrara cuando el bus complete la accion.
    for (timeout = 0U; timeout < 3000U; timeout++) {                     // Espera acotada con contador: si el bus no responde se devuelve fallo.
        if ((TWCR & (1U << TWSTO)) == 0U) {                              // STOP confirmado: el controlador puede declararse libre.
            twi_fase = TWI_LIBRE;                                        // Actualiza la ocupacion I2C que consulta la proteccion PRR.
            break;                                                       // Termina este recorrido cuando ya se encontro o completo lo necesario.
        }
        _delay_us(1);                                                    // Retardo acotado de 1 us para el bus/LCD; no es un temporizador de tareas.
    }
}

static uint8_t lcd_expansor(uint8_t valor) {                             // Escribe al PCF8574, incorporando el bit de retroiluminacion.
    uint8_t correcto;                                                    // Reserva: confirmacion acumulada de la operacion.
    correcto = twi_iniciar((uint8_t)(direccion_lcd << 1));               // Actualiza confirmacion acumulada de la operacion.
    if (correcto) {                                                      // Envia el dato solo si primero hubo ACK a la direccion.
        correcto = twi_enviar((uint8_t)(valor | luz_lcd));               // Actualiza confirmacion acumulada de la operacion.
    }
    twi_stop();                                                          // Libera el bus I2C mediante STOP, incluso despues de un fallo.
    return correcto;                                                     // Entrega al llamador el resultado de escribir el expansor.
}

static uint8_t lcd_nativo_enviar(uint8_t control, uint8_t valor) {       // Protocolo del JHD-2X16-I2C/AiP31068L de Proteus.
    uint8_t correcto;
    correcto = twi_iniciar((uint8_t)(LCD_DIRECCION_NATIVA << 1));
    if (correcto) correcto = twi_enviar(control);
    if (correcto) correcto = twi_enviar(valor);
    twi_stop();
    return correcto;
}

static void lcd_encender_luz(void) {                                     // Activa el bit de luz del expansor si se detecto la LCD.
    if (!lcd_disponible) return;                                         // Evita transacciones hacia una LCD ausente o con fallo previo.
    if (tipo_lcd != LCD_TIPO_PCF8574) return;                             // El LCD nativo se enciende con el comando 0x0C.
    luz_lcd = LCD_LUZ;                                                   // Actualiza bit de retroiluminacion que acompana cada envio.
    (void)lcd_expansor(0U);                                              // Actualiza la salida del expansor, incluido su bit de luz.
}

static void lcd_apagar_luz(void) {                                       // Apaga la retroiluminacion mediante el expansor, sin cortar su alimentacion.
    if (!lcd_disponible) return;                                         // Evita transacciones hacia una LCD ausente o con fallo previo.
    if (tipo_lcd != LCD_TIPO_PCF8574) return;                             // El LCD nativo se apaga con el comando 0x08.
    luz_lcd = 0U;                                                        // Actualiza bit de retroiluminacion que acompana cada envio.
    (void)lcd_expansor(0U);                                              // Actualiza la salida del expansor, incluido su bit de luz.
}

static uint8_t lcd_pulso(uint8_t valor) {                                // Genera el pulso E que permite a la LCD capturar un nibble.
    if (!lcd_expansor((uint8_t)(valor | LCD_E))) return 0U;              // Propaga un fallo de comunicacion y evita seguir usando una pantalla no disponible.
    _delay_us(1);                                                        // Retardo acotado de 1 us para el bus/LCD; no es un temporizador de tareas.
    if (!lcd_expansor((uint8_t)(valor & (uint8_t)~LCD_E))) return 0U;    // Propaga un fallo de comunicacion y evita seguir usando una pantalla no disponible.
    _delay_us(50);                                                       // Retardo acotado de 50 us para el bus/LCD; no es un temporizador de tareas.
    return 1U;                                                           // Devuelve verdadero: la operacion termino correctamente.
}

static uint8_t lcd_medio_byte(uint8_t valor) {                           // Coloca cuatro bits en el expansor y genera el pulso de captura.
    if (!lcd_expansor(valor)) return 0U;                                 // Propaga un fallo de comunicacion y evita seguir usando una pantalla no disponible.
    return lcd_pulso(valor);                                             // El nibble ya esta colocado; falta que E permita capturarlo.
}

static uint8_t lcd_byte(uint8_t valor, uint8_t es_dato) {                // Divide el byte en dos nibbles y distingue dato de instruccion mediante RS.
    uint8_t control = es_dato ? LCD_RS : 0U;                             // Reserva: bit RS que distingue dato de instruccion LCD.
    if (!lcd_medio_byte((uint8_t)((valor & 0xF0U) | control))) return 0U; // Propaga un fallo de comunicacion y evita seguir usando una pantalla no disponible.
    if (!lcd_medio_byte((uint8_t)(((valor << 4) & 0xF0U) | control))) return 0U; // Propaga un fallo de comunicacion y evita seguir usando una pantalla no disponible.
    return 1U;                                                           // Devuelve verdadero: la operacion termino correctamente.
}

static void lcd_comando(uint8_t comando) {                               // Envia una instruccion HD44780; clear/home necesitan espera adicional.
    if (!lcd_disponible) return;                                         // Evita transacciones hacia una LCD ausente o con fallo previo.
    if (tipo_lcd == LCD_TIPO_NATIVO) {
        if (!lcd_nativo_enviar(LCD_CONTROL_COMANDO, comando)) {
            lcd_disponible = 0U;
            return;
        }
    } else if (!lcd_byte(comando, 0U)) {                                 // Camino para mochila PCF8574.
        lcd_disponible = 0U;                                             // Actualiza LCD detectada y sin fallo de escritura.
        return;                                                          // Cede inmediatamente para que main continue con las otras tareas.
    }
    if (comando == 0x01U || comando == 0x02U) {                          // Clear y retorno a inicio tardan mas que una instruccion LCD habitual.
        _delay_ms(2);                                                    // Retardo acotado de 2 ms para el bus/LCD; no es un temporizador de tareas.
    } else if (tipo_lcd == LCD_TIPO_NATIVO) {
        _delay_us(50);
    }
}

static void lcd_dato(uint8_t dato) {                                     // Envia un caracter a la posicion actual de la LCD.
    if (!lcd_disponible) return;                                         // Evita transacciones hacia una LCD ausente o con fallo previo.
    if (tipo_lcd == LCD_TIPO_NATIVO) {
        if (!lcd_nativo_enviar(LCD_CONTROL_DATO, dato)) {
            lcd_disponible = 0U;
            return;
        }
        _delay_us(50);
    } else if (!lcd_byte(dato, 1U)) {                                    // Camino para mochila PCF8574.
        lcd_disponible = 0U;                                             // Actualiza LCD detectada y sin fallo de escritura.
    }
}

static uint8_t lcd_init(void) {                                          // Detecta automaticamente LCD nativo o mochila PCF8574.
    static const uint8_t direcciones_pcf[] = {0x27U, 0x3FU};
    uint8_t indice, encontrado = 0U;
    _delay_ms(50);

    /* Proteus JHD-2X16-I2C/AiP31068L: direccion de 7 bits 0x3E. */
    if (twi_iniciar((uint8_t)(LCD_DIRECCION_NATIVA << 1))) {
        twi_stop();
        direccion_lcd = LCD_DIRECCION_NATIVA;
        tipo_lcd = LCD_TIPO_NATIVO;
        lcd_disponible = 1U;
        lcd_comando(0x38U);                                              // Interfaz nativa de 8 bits, dos lineas y matriz 5x8.
        lcd_comando(0x08U);                                              // Display apagado mientras se configura.
        lcd_comando(0x01U);                                              // Limpia DDRAM.
        lcd_comando(0x06U);                                              // Cursor incremental.
        lcd_comando(0x0CU);                                              // Display encendido, sin cursor.
        return lcd_disponible;
    }
    twi_stop();

    /* LCD fisico con mochila PCF8574: direcciones habituales 0x27/0x3F. */
    for (indice = 0U; indice < (uint8_t)sizeof(direcciones_pcf); indice++) {
        if (twi_iniciar((uint8_t)(direcciones_pcf[indice] << 1))) {
            direccion_lcd = direcciones_pcf[indice];
            encontrado = 1U;
            twi_stop();
            break;
        }
        twi_stop();
    }
    if (!encontrado) return 0U;

    tipo_lcd = LCD_TIPO_PCF8574;
    lcd_disponible = 1U;
    (void)lcd_medio_byte(0x30U);
    _delay_ms(5);
    (void)lcd_medio_byte(0x30U);
    _delay_us(150);
    (void)lcd_medio_byte(0x30U);
    (void)lcd_medio_byte(0x20U);
    lcd_comando(0x28U);
    lcd_comando(0x0CU);
    lcd_comando(0x01U);
    lcd_comando(0x06U);
    return lcd_disponible;
}

static void linea_limpiar(char linea[17]) {                              // Prepara una fila de 16 espacios y su terminador de cadena.
    uint8_t i;                                                           // Reserva: indice de recorrido.
    for (i = 0U; i < 16U; i++) {                                         // Recorre exactamente las 16 columnas de la pantalla.
        linea[i] = ' ';                                                  // Rellena una columna para borrar contenido anterior.
    }
    linea[16] = '\0';                                                    // Reserva el caracter 17 al terminador; la LCD muestra solo 16.
}

static void linea_texto_P(char linea[17], uint8_t posicion, const char *texto_P) { // Copia texto desde Flash sin sobrepasar las 16 columnas.
    uint8_t c;                                                           // Reserva: caracter leido desde RAM o Flash.
    while (posicion < 16U && (c = pgm_read_byte(texto_P++)) != 0U) {     // Limita la copia de texto a las columnas fisicas de la fila.
        linea[posicion++] = (char)c;                                     // Copia una letra Flash en la columna actual y avanza.
    }
}

static void linea_numero(char linea[17], uint8_t posicion,               // Escribe una cantidad fija de digitos decimales en la fila.
                         uint16_t numero, uint8_t cifras) {
    while (cifras > 0U) {                                                // Formatea exactamente la cantidad de cifras solicitada.
        cifras--;                                                        // Avanza a la siguiente posicion hacia la izquierda.
        linea[posicion + cifras] = (char)('0' + (numero % 10U));         // Coloca el digito desde la derecha dentro del ancho solicitado.
        numero /= 10U;                                                   // Pasa al siguiente digito decimal.
    }
}

static void lcd_linea(uint8_t fila, const char linea[17]) {              // Selecciona la fila y envia sus 16 caracteres.
    uint8_t i;                                                           // Reserva: indice de recorrido.
    lcd_comando(fila == 0U ? 0x80U : 0xC0U);                             // Selecciona direccion DDRAM de fila 0 o fila 1 (0x40).
    for (i = 0U; i < 16U; i++) {                                         // Recorre exactamente las 16 columnas de la pantalla.
        lcd_dato((uint8_t)linea[i]);                                     // Envia un caracter de la fila seleccionada.
    }
}

static void mostrar_activo(void) {                                       // Prepara ADC y DHT, o el error correspondiente, para ambas filas.
    char linea1[17];                                                     // Reserva: primera fila de pantalla.
    char linea2[17];                                                     // Reserva: segunda fila de pantalla.
    linea_limpiar(linea1);                                               // Prepara la fila con espacios y terminador antes de formatearla.
    linea_limpiar(linea2);                                               // Prepara la fila con espacios y terminador antes de formatearla.
    linea_texto_P(linea1, 0U, PSTR("ACTIVO ADC:"));                      // Coloca texto fijo desde Flash en las columnas indicadas.
    linea_numero(linea1, 11U, valor_adc_actual, 4U);                     // Inserta el valor numerico en su posicion y ancho de pantalla.
    if (lectura_dht_correcta) {                                          // Presenta el dato del DHT solo si su ultima captura fue valida.
        linea_texto_P(linea2, 0U, PSTR("T:"));                           // Coloca texto fijo desde Flash en las columnas indicadas.
        linea_numero(linea2, 2U, temperatura, 2U);                       // Inserta el valor numerico en su posicion y ancho de pantalla.
        linea_texto_P(linea2, 4U, PSTR("C H:"));                         // Coloca texto fijo desde Flash en las columnas indicadas.
        linea_numero(linea2, 8U, humedad, 3U);                           // Inserta el valor numerico en su posicion y ancho de pantalla.
        linea2[11] = '%';                                                // Completa la unidad de humedad en la posicion prevista.
    } else if (error_dht != 0U) {
        linea_texto_P(linea2, 0U, PSTR("DHT ERROR:"));                   // Coloca texto fijo desde Flash en las columnas indicadas.
        linea_numero(linea2, 11U, error_dht, 1U);                        // Inserta el valor numerico en su posicion y ancho de pantalla.
    } else {
        linea_texto_P(linea2, 0U, PSTR("DHT: ESPERANDO"));               // Coloca texto fijo desde Flash en las columnas indicadas.
    }
    lcd_linea(0U, linea1);                                               // Transfiere la fila completa a la memoria de caracteres de la LCD.
    lcd_linea(1U, linea2);                                               // Transfiere la fila completa a la memoria de caracteres de la LCD.
}

/* Un buffer y una rutina para los tres mensajes de estado. */
static void mostrar_mensaje_estado_P(const char *titulo_P, const char *detalle_P) { // Reutiliza una fila temporal para presentar los mensajes de los estados.
    char linea[17];                                                      // Reserva: fila temporal de 16 caracteres mas terminador.
    linea_limpiar(linea);                                                // Prepara la fila con espacios y terminador antes de formatearla.
    linea_texto_P(linea, 0U, titulo_P);                                  // Coloca texto fijo desde Flash en las columnas indicadas.
    lcd_linea(0U, linea);                                                // Transfiere la fila completa a la memoria de caracteres de la LCD.
    linea_limpiar(linea);                                                // Prepara la fila con espacios y terminador antes de formatearla.
    linea_texto_P(linea, 0U, detalle_P);                                 // Coloca texto fijo desde Flash en las columnas indicadas.
    lcd_linea(1U, linea);                                                // Transfiere la fila completa a la memoria de caracteres de la LCD.
}

/* DHT11: pulso inicial programado y captura breve por sondeo. */

static void liberar_dht(void) {                                          // Pasa DATA a entrada con pull-up para que el DHT pueda responder.
    DDRD &= (uint8_t)~(1U << DHT_DDR_PIN);                                      // Libera DATA del DHT: pin como entrada.
    PORTD |= (1U << DHT_PIN);                                            // Habilita pull-up al liberar DATA para que el sensor pueda llevarla a cero.
}

static void cancelar_dht(void) {                                         // Desarma la captura, libera DATA y quita reloj de Timer2.
    uint8_t sreg = SREG;                                                 // Reserva: copia de SREG para restaurar el estado previo de interrupciones.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    PCMSK2 &= (uint8_t)~(1U << DHT_PCINT_BIT);                                 // Desarma DHT_PCINT_BIT para no capturar mas flancos del DHT.
    estado_dht = DHT_REPOSO;                                             // Transicion de la captura: Sensor sin captura activa.
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
    liberar_dht();                                                       // Deja que el sensor tome DATA; no seguir conduciendo el pin en bajo.
    (void)prr_deshabilitar(PRR_TIMER2);                                  // Solicita quitar reloj, pasando antes por la validacion de recursos ocupados.
    bandera_dht = 0U;                                                    // Consume/limpia: solicitud de iniciar DHT.
}

/* Espera a que DATA deje el nivel indicado, con un limite en microsegundos. */
static uint8_t dht_esperar_mientras(uint8_t nivel_alto, uint16_t timeout_us) {
    while (((PIND & _BV(DHT_PIN)) != 0U) == (nivel_alto != 0U)) {
        if (timeout_us == 0U) return 0U;
        timeout_us--;
        _delay_us(1);
    }
    return 1U;
}

/* La parte critica dura cerca de 4 ms. Se deshabilitan interrupciones para
 * que UART o timers no alteren la medida de los pulsos del DHT11. */
static uint8_t dht_capturar_datos(void) {
    uint8_t i;
    uint8_t correcto = 1U;
    uint8_t sreg = SREG;

    cli();

    /* Respuesta inicial del sensor: 80 us bajo y 80 us alto. */
    if (!dht_esperar_mientras(1U, 100U) ||
        !dht_esperar_mientras(0U, 100U) ||
        !dht_esperar_mientras(1U, 100U)) {
        correcto = 0U;
    }

    for (i = 0U; i < 40U && correcto; i++) {
        if (!dht_esperar_mientras(0U, 80U)) {
            correcto = 0U;
            break;
        }

        /* A los 40 us, un cero ya termino y un uno sigue en nivel alto. */
        _delay_us(40);
        dht_datos[i / 8U] <<= 1U;
        if ((PIND & _BV(DHT_PIN)) != 0U) {
            dht_datos[i / 8U] |= 1U;
            if (!dht_esperar_mientras(1U, 100U)) {
                correcto = 0U;
                break;
            }
        }
    }

    SREG = sreg;
    return correcto;
}

static void dht_comenzar(uint32_t ahora) {                               // Inicia el pulso bajo de 20 ms sin bloquear main durante ese intervalo.
    uint8_t sreg = SREG;                                                 // Reserva: copia de SREG para restaurar el estado previo de interrupciones.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    PCMSK2 &= (uint8_t)~(1U << DHT_PCINT_BIT);                                 // Desarma DHT_PCINT_BIT para no capturar mas flancos del DHT.
    dht_datos[0] = 0U;                                                   // Limpia este byte antes de recibir una nueva trama DHT.
    dht_datos[1] = 0U;                                                   // Limpia este byte antes de recibir una nueva trama DHT.
    dht_datos[2] = 0U;                                                   // Limpia este byte antes de recibir una nueva trama DHT.
    dht_datos[3] = 0U;                                                   // Limpia este byte antes de recibir una nueva trama DHT.
    dht_datos[4] = 0U;                                                   // Limpia este byte antes de recibir una nueva trama DHT.
    estado_dht = DHT_INICIO_BAJO;                                        // Transicion de la captura: Microcontrolador mantiene DATA en cero.
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
    /* Pulso inicial sin espera activa: el main volvera despues de 20 ms. */
    DDRD |= (1U << DHT_DDR_PIN);                                                // Toma control de DATA para generar el pulso inicial.
    PORTD &= (uint8_t)~(1U << DHT_PIN);                                  // Lleva DATA a cero y comienza el pulso de solicitud DHT.
    dht_marca_ms = ahora;                                                // Inicia el plazo de la fase DHT que se acaba de activar.
    dht_ultimo_inicio_ms = ahora;                                        // Base para respetar los 2 s minimos antes de otro intento DHT.
    dht_intento_previo = 1U;                                             // A partir de ahora se aplica el intervalo minimo entre lecturas.
}

static void dht_liberar_y_capturar(uint32_t ahora) {
    liberar_dht();
    _delay_us(20);                                                       // Tiempo para que el sensor tome el bus.

    if (dht_capturar_datos()) {
        estado_dht = DHT_COMPLETO;
    } else {
        estado_dht = DHT_ERROR;
    }

    dht_marca_ms = ahora;
}

static void dht_procesar_resultado(void) {                               // Copia los cinco bytes, verifica suma y actualiza temperatura/humedad.
    uint8_t datos[5];                                                    // Reserva: copia local de los cinco bytes DHT.
    uint8_t sreg = SREG;                                                 // Reserva: copia de SREG para restaurar el estado previo de interrupciones.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    datos[0] = dht_datos[0];                                             // Copia un byte compartido con la ISR dentro de la seccion critica.
    datos[1] = dht_datos[1];                                             // Copia un byte compartido con la ISR dentro de la seccion critica.
    datos[2] = dht_datos[2];                                             // Copia un byte compartido con la ISR dentro de la seccion critica.
    datos[3] = dht_datos[3];                                             // Copia un byte compartido con la ISR dentro de la seccion critica.
    datos[4] = dht_datos[4];                                             // Copia un byte compartido con la ISR dentro de la seccion critica.
    estado_dht = DHT_REPOSO;                                             // Transicion de la captura: Sensor sin captura activa.
    (void)prr_deshabilitar(PRR_TIMER2); /* Termino captura: PRR vuelve a 0x44. */
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
    if ((uint8_t)(datos[0] + datos[1] + datos[2] + datos[3]) ==          // Verifica la suma de los cuatro bytes contra el checksum del DHT.
        datos[4]) {
        humedad = datos[0];                                              // El primer byte DHT contiene la parte entera de humedad.
        temperatura = datos[2];                                          // El tercer byte DHT contiene la parte entera de temperatura.
        lectura_dht_correcta = 1U;                                       // Publica si los valores DHT son validos; no confundir datos viejos con nuevos.
        dht_ultima_lectura_ms = leer_tiempo();                           // Guarda la edad del dato valido para responder al cliente.
        error_dht = 0U;                                                  // Publica el codigo de fallo; 6=checksum, 7=timeout o captura sin codigo.
    } else {
        lectura_dht_correcta = 0U;                                       // Publica si los valores DHT son validos; no confundir datos viejos con nuevos.
        error_dht = 6U;                                                  // Publica el codigo de fallo; 6=checksum, 7=timeout o captura sin codigo.
    }
    if (solicitud_sensores) sensores_listos = 1U;                        // Solo notifica/cancela si existe una solicitud de sensores pendiente.
    bandera_lcd = 1U;                                                    // Solicita: actualizacion de pantalla pendiente.
}

static void tarea_dht(uint32_t ahora) {                                  // Avanza la lectura del sensor desde main.
    uint8_t copia_estado, sreg;
    if (estado != ESTADO_ACTIVO) {                                       // Solo ACTIVO ejecuta sensores; fuera de el se cancela la captura.
        cancelar_dht();                                                  // Cancela captura y devuelve Timer2 al perfil de ahorro.
        return;                                                          // Cede inmediatamente para que main continue con las otras tareas.
    }
    sreg = SREG;                                                         // Actualiza copia de SREG para restaurar el estado previo de interrupciones.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    copia_estado = estado_dht;                                           // Actualiza estado DHT leido coherentemente.
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
    if (bandera_dht && copia_estado == DHT_REPOSO &&                     // Consume trabajo pendiente sin realizarlo dentro de la ISR.
        (!dht_intento_previo ||
         (uint32_t)(ahora - dht_ultimo_inicio_ms) >= PERIODO_DHT_MS)) {
        bandera_dht = 0U;                                                // Consume/limpia: solicitud de iniciar DHT.
        dht_comenzar(ahora);                                             // Comienza la adquisicion sin esperar aqui todo el pulso inicial.
    } else if (copia_estado == DHT_INICIO_BAJO &&                        // El reloj determina cuando terminar el pulso inicial de 20 ms.
               (uint32_t)(ahora - dht_marca_ms) >= DHT_INICIO_BAJO_MS) {
        dht_liberar_y_capturar(ahora);                                   // Termina el pulso de solicitud y comienza la medida por flancos.
    } else if (copia_estado == DHT_COMPLETO) {
        dht_procesar_resultado();
    } else if (copia_estado == DHT_ERROR) {
        lectura_dht_correcta = 0U;                                       // Publica si los valores DHT son validos; no confundir datos viejos con nuevos.
        error_dht = 7U;                                                   // El sensor no completo la secuencia esperada.
        cancelar_dht();                                                  // Cancela captura y devuelve Timer2 al perfil de ahorro.
        /* Programa un nuevo intento, respetando los 2 s del DHT11. */
        dht_ultimo_inicio_ms = ahora;
        dht_intento_previo = 1U;
        bandera_dht = 1U;
        if (solicitud_sensores) sensores_listos = 1U;                    // Solo notifica/cancela si existe una solicitud de sensores pendiente.
        bandera_lcd = 1U;                                                // Solicita: actualizacion de pantalla pendiente.
    }
}

/* RELOJ Y RESTAURACION DE PERIFERICOS - ACTIVIDAD 1 */

/* Llamar con interrupciones deshabilitadas. Conserva la fraccion transcurrida. */
static void reloj_capturar_y_detener(void) {                             // Conserva tiempo parcial antes de detener timers; requiere interrupciones deshabilitadas.
    uint32_t parcial_us = fraccion_reloj_us;                             // Reserva: tiempo parcial del timer expresado en microsegundos.
    if (modo_reloj == RELOJ_RAPIDO) {                                    // El reloj rapido obtiene su fase parcial de TCNT0.
        TCCR0B = 0U;                                                     // Detiene el contador Timer0 antes de trasladar/quitar su reloj.
        if ((TIFR0 & _BV(OCF0A)) != 0U) {                                // Incorpora un compare ocurrido que aun no habia atendido su ISR.
            tiempo_ms++;           /* Compare ocurrido, aun sin atender su ISR. */
        }
        parcial_us += (uint32_t)TCNT0 * 4UL;                             // Timer0 usa 4 us por cuenta en ACTIVO.
    } else if (modo_reloj == RELOJ_STANDBY) {                            // El reloj lento obtiene su fase parcial desde el contador de Timer1.
        TCCR1B = _BV(WGM12);        /* CTC se conserva; reloj detenido. */
        if ((TIFR1 & _BV(OCF1A)) != 0U) {                                // Incorpora un compare ocurrido que aun no habia atendido su ISR.
            tiempo_ms += PASO_STANDBY_MS;                                // Suma el paso de 1 segundo de Timer1.
        }
        parcial_us += (uint32_t)TCNT1 * 16UL;                            // Timer1 usa 16 us por cuenta en STANDBY.
    }
    tiempo_ms += parcial_us / 1000UL;                                    // Incorpora al reloj los milisegundos completos del contador detenido.
    fraccion_reloj_us = (uint16_t)(parcial_us % 1000UL);                 // Conserva el resto de microsegundos para no perderlo entre cambios.
    if ((PRR & PRR_TIMER0) == 0U) {                                      // Solo accede al timer si todavia tiene reloj.
        TCCR0B = 0U;                                                     // Detiene el contador Timer0 antes de trasladar/quitar su reloj.
        TIMSK0 = 0U;                                                     // Deshabilita IRQ del timer antes de quitar su reloj.
        TIFR0 = _BV(OCF0A) | _BV(OCF0B) | _BV(TOV0);                     // Limpia flags pendientes escribiendo unos antes de rearmar el timer.
    }
    if ((PRR & PRR_TIMER1) == 0U) {                                      // Solo accede al timer si todavia tiene reloj.
        TCCR1B = _BV(WGM12);                                             // Mantiene CTC y deja CS12:0=000: Timer1 detenido.
        TIMSK1 = 0U;                                                     // Deshabilita IRQ del timer antes de quitar su reloj.
        TIFR1 = _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);                     // Limpia flags pendientes escribiendo unos antes de rearmar el timer.
    }
    modo_reloj = RELOJ_DETENIDO;                                         // Informa que base temporal debe atender la ISR y proteger PRR.
}

static void reloj_rapido(void) {                                         // Restaura Timer0/Timer1 y las fases de las tareas de ACTIVO.
    uint8_t sreg = SREG;                                                 // Reserva: copia de SREG para restaurar el estado previo de interrupciones.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    if (modo_reloj != RELOJ_RAPIDO) {                                    // Evita reinicializar fases si el reloj rapido ya funciona.
        reloj_capturar_y_detener();                                      // Guarda el tiempo parcial antes de detener los timers de la base temporal.
        contador_led_ms = 0U;                                            // Reinicia la fase de ese periodo despues de producir su bandera.
        contador_lcd_ms = 0U;                                            // Reinicia la fase de ese periodo despues de producir su bandera.
        contador_dht_ms = 0U;                                            // Reinicia la fase de ese periodo despues de producir su bandera.
        bandera_reloj = 0U;                                              // Consume/limpia: tick de STANDBY pendiente.
        modo_reloj = RELOJ_RAPIDO;                                       // Informa que base temporal debe atender la ISR y proteger PRR.
        timer0_init();                                                   // Configura CTC de 1 ms: 16 MHz / 64 / 250.
        timer1_init();                                                   // Configura CTC de 100 ms: 16 MHz / 64 / 25000.
        timer2_init();                                                   // Prepara Timer2 normal para DHT y lo deja sin reloj hasta la captura.
    }
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
}

static void reloj_standby(void) {                                        // Traslada el reloj a Timer1 para despertar una vez por segundo en IDLE.
    uint8_t sreg = SREG;                                                 // Reserva: copia de SREG para restaurar el estado previo de interrupciones.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    reloj_capturar_y_detener();                                          // Guarda el tiempo parcial antes de detener los timers de la base temporal.
    bandera_reloj = 0U;                                                  // Consume/limpia: tick de STANDBY pendiente.
    modo_reloj = RELOJ_STANDBY;                                          // Informa que base temporal debe atender la ISR y proteger PRR.
    timer1_init();                 /* Solo 1 IRQ/s para contar la inactividad. */
    (void)prr_deshabilitar(PRR_TIMER0 | PRR_TIMER2 | PRR_ADC | PRR_SPI); // Solicita quitar reloj, pasando antes por la validacion de recursos ocupados.
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
}

static void restaurar_para_activo(void) {                                // Recupera timers, TWI y ADC antes de usarlos al volver a ACTIVO.
    reloj_rapido();                                                      // Restaura Timer0/Timer1 y las fases de las tareas de ACTIVO.
    /* SEGURO ya tenia Timer0 rapido, pero habia quitado el reloj de Timer1. */
    if ((PRR & PRR_TIMER1) != 0U) timer1_init();                         // SEGURO apagaba Timer1: restaurarlo aunque Timer0 ya estuviera rapido.
    twi_init();                    /* Antes de intentar escribir a la LCD. */
    adc_habilitar();                                                     // Devuelve reloj al ADC, selecciona AVCC/A0 y habilita prescaler 128.
}

static void int0_flanco_descendente(void) {                              // Restaura INT0 por flanco para la ejecucion normal.
    EIMSK &= (uint8_t)~_BV(INT0);                                        // Enmascara INT0 mientras se cambia el modo o se atiende nivel mantenido.
    EICRA = (uint8_t)((EICRA & (uint8_t)~(_BV(ISC01) | _BV(ISC00))) |    // Prepara ISC01:00 para flanco descendente, conservando los demas bits.
                     _BV(ISC01));                                        // Restaura flanco descendente despues de utilizar nivel bajo para despertar.
    EIFR = _BV(INTF0);                                                   // Limpia el flag externo pendiente escribiendo uno.
    EIMSK |= _BV(INT0);                                                  // Habilita la fuente INT0 del pulsador.
}

/* Esta comprobacion se llama con IRQ deshabilitadas. */
static uint8_t hay_evento_pendiente(void) {                              // Comprueba trabajo pendiente antes de permitir que la CPU duerma.
    return (bandera_boton || bandera_reloj || bandera_wdt || uart_rx_error || // Un evento, byte o boton ya bajo impide dormir hasta que main lo atienda.
            uart_rx_cola != uart_rx_cabeza || (UCSR0A & _BV(RXC0)) != 0U ||
            (PIND & _BV(BOTON_PIN)) == 0U) ? 1U : 0U;                    // Incluye el nivel del pin por si la pulsacion llego antes de armar la IRQ.
}

static void dormir_idle(void) {                                          // Entra en IDLE solo si STANDBY no tiene eventos por atender.
    uint8_t sreg = SREG;                                                 // Reserva: copia de SREG para restaurar el estado previo de interrupciones.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    if (estado == ESTADO_STANDBY && !hay_evento_pendiente()) {           // Comprobacion atomica para no dormir dejando trabajo pendiente.
        set_sleep_mode(SLEEP_MODE_IDLE);                                 // Selecciona IDLE: el reloj de perifericos habilitados sigue disponible.
        sleep_enable();                                                  // Arma SE en SMCR; la CPU aun no esta dormida.
        sei();                                                           // Habilita IRQ; la siguiente instruccion debe ser SLEEP para no perder un evento.
        sleep_cpu();               /* SEI y SLEEP deben ser consecutivos. */
        sleep_disable();                                                 // Borra SE para impedir una entrada accidental posterior al sueño.
    }
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
}

static void dormir_powerdown(void) {                                     // Finaliza TX, prepara INT0/PRR, duerme y restaura el contexto al despertar.
    uint8_t sreg, prr_anterior, usart_anterior;                          // Reserva: copia de SREG para restaurar el estado previo de interrupciones; perfil que se restaurara despues del sueño; habilitaciones UART previas al sueño.
    if (!uart_tx_terminada() || uart_en_trama || uart_trama_pendiente) return;                    /* Termina de enviar el aviso sin bloquear main. */
    sreg = SREG;                                                         // Actualiza copia de SREG para restaurar el estado previo de interrupciones.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    if (estado != ESTADO_APAGADO || hay_evento_pendiente() || !uart_tx_terminada()) { // Revalida estado/eventos/TX con IRQ bloqueadas antes de dormir.
        SREG = sreg;                                                     // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
        return;                                                          // Cede inmediatamente para que main continue con las otras tareas.
    }
    /* INT0 por nivel bajo: fuente asincrona valida en POWER-DOWN. */
    EIMSK &= (uint8_t)~_BV(INT0);                                        // Enmascara INT0 mientras se cambia el modo o se atiende nivel mantenido.
    EICRA &= (uint8_t)~(_BV(ISC01) | _BV(ISC00));                        // ISC01:00=00: nivel bajo asincrono para despertar de Power-Down.
    EIFR = _BV(INTF0);                                                   // Limpia el flag externo pendiente escribiendo uno.
    en_powerdown = 1U;                                                   // Actualiza el contexto usado para gestionar INT0 al entrar/salir del sueño.
    EIMSK |= _BV(INT0);                                                  // Habilita la fuente INT0 del pulsador.
    if ((PIND & _BV(BOTON_PIN)) == 0U) {                                 // Si el boton llego antes de SLEEP, atiende el evento sin entrar al sueño.
        bandera_boton = 1U;         /* Pulsacion llegada antes de dormir. */
        en_powerdown = 0U;                                               // Actualiza el contexto usado para gestionar INT0 al entrar/salir del sueño.
        int0_flanco_descendente();                                       // Recupera el modo de pulsador normal despues del despertar.
        SREG = sreg;                                                     // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
        return;                                                          // Cede inmediatamente para que main continue con las otras tareas.
    }
    wdt_antes_powerdown();         /* Prueba 1 arma IRQ; prueba 2 pausa supervision. */
    prr_anterior = PRR;                                                  // Actualiza perfil que se restaurara despues del sueño.
    usart_anterior = UCSR0B;                                             // Actualiza habilitaciones UART previas al sueño.
    reloj_capturar_y_detener();                                          // Guarda el tiempo parcial antes de detener los timers de la base temporal.
    if (!prr_fijar_perfil(PRR_DORMIR)) {                                 // Si algun recurso impide dormir, recupera main y no ejecuta SLEEP.
        /* Un recurso ocupado impide dormir: conservar main y su reloj. */
        reloj_standby();                                                 // Traslada el reloj a Timer1 para despertar una vez por segundo en IDLE.
        wdt_despues_powerdown();                                         // WDT 1 conserva su intervalo; WDT 2 reanuda supervision.
        en_powerdown = 0U;                                               // Actualiza el contexto usado para gestionar INT0 al entrar/salir del sueño.
        int0_flanco_descendente();                                       // Recupera el modo de pulsador normal despues del despertar.
        SREG = sreg;                                                     // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
        return;                                                          // Cede inmediatamente para que main continue con las otras tareas.
    }
    uart_linea_longitud = 0U;                                            // Vacia el texto parcial y permite empezar desde un nuevo inicio.
    uart_descartar_linea = 0U;                                           // Limpia/marca el texto recibido fuera de un inicio valido.
    PCMSK2 &= (uint8_t)~_BV(DHT_PCINT_BIT);                                    // Desarma DHT_PCINT_BIT para no capturar mas flancos del DHT.
    PCICR &= (uint8_t)~_BV(PCIE2);                                       // Desactiva el grupo D durante la preparacion del sueño.
    prr_ultimo_sueno = PRR;         /* Copia historica: UART ahora no tiene reloj. */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);                                 // Selecciona sueño profundo; UART convencional no funciona como despertar.
    sleep_enable();                                                      // Arma SE en SMCR; la CPU aun no esta dormida.
#if defined(BODS) && defined(BODSE)
    sleep_bod_disable();                                                 // Apaga BOD durante Power-Down para reducir consumo.
#endif
    sei();                                                               // Habilita IRQ; la siguiente instruccion debe ser SLEEP para no perder un evento.
    sleep_cpu();                   /* El pulsador debe mantenerse durante el arranque. */
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    sleep_disable();                                                     // Borra SE para impedir una entrada accidental posterior al sueño.
    /* Se continua aqui tras el despertar; la SRAM conserva la FSM. */
    (void)prr_fijar_perfil(prr_anterior);                                // Aplica el perfil de reloj de este estado despues de preparar los recursos.
    uart_init();                   /* Reconfigurar USART tras devolverle el reloj. */
    UCSR0B = usart_anterior;                                             // Restaura las habilitaciones UART despues de reinicializar su reloj y formato.
    reloj_standby();               /* Reloj disponible antes de validar el evento. */
    wdt_despues_powerdown();                                             // WDT 1 conserva su intervalo; WDT 2 reanuda supervision.
    en_powerdown = 0U;                                                   // Actualiza el contexto usado para gestionar INT0 al entrar/salir del sueño.
    int0_flanco_descendente();                                           // Recupera el modo de pulsador normal despues del despertar.
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
    prr_informar_valor(PSTR("ULTIMO_SUENO"), prr_ultimo_sueno);          // Envia una copia historica si dice ULTIMO_SUENO; no es el PRR actual al transmitir.
    /* main valida el boton y entrar_activo restaura el resto. */
}

static void gestionar_sueno(void) {                                      // Selecciona el modo Sleep segun la FSM; ACTIVO y SEGURO no duermen aqui.
    if (estado == ESTADO_STANDBY) {                                      // La FSM STANDBY corresponde a Sleep IDLE, no al modo hardware Standby.
        dormir_idle();                                                   // Entra en IDLE solo si STANDBY no tiene eventos por atender.
    } else if (estado == ESTADO_APAGADO) {
        dormir_powerdown();                                              // Finaliza TX, prepara INT0/PRR, duerme y restaura el contexto al despertar.
    }
    /* En SEGURO se conserva el reloj rapido y la alarma original. */
}

/* FSM DEL SISTEMA - SIN SWITCH-CASE */

static void cancelar_solicitud_sensores(void) {                          // Notifica si un cambio de estado interrumpe una adquisicion solicitada.
    if (solicitud_sensores) {                                            // Solo notifica/cancela si existe una solicitud de sensores pendiente.
        solicitud_sensores = sensores_listos = 0U;                       // Cierra la solicitud para no notificarla otra vez.
        ERROR_PROTO("SENSORES_CANCELADOS");                              // Informa que el cambio de estado cancelo una solicitud pendiente.
    }
}

static void entrar_activo(uint32_t ahora, uint8_t registrar_actividad) { // Restaura tareas, pantalla y perifericos; registra actividad si corresponde.
    modo_led = LED_AUTOMATICO;                                           // Selecciona parpadeo automatico o salida fija solicitada.
    wdt_al_volver_a_ejecucion();    /* Un usuario puede interrumpir la prueba dormida. */
    alarma_continua = 0U;                                                // Actualiza habilitacion del sonido continuo de SEGURO.
    restaurar_para_activo();                                             // Recupera timers, TWI y ADC antes de usarlos al volver a ACTIVO.
    lcd_encender_luz();                                                  // Activa el bit de luz del expansor si se detecto la LCD.
    lcd_comando(0x0CU);                                                  // Enciende el display con cursor y parpadeo de cursor apagados.
    estado = ESTADO_ACTIVO;                                              // Actualiza la FSM principal; las acciones de entrada configuran sus recursos.
    PORTB &= (uint8_t)~(1U << LED_PIN);                                  // Deja LED apagado.
    if (registrar_actividad) {                                           // Solo el evento valido solicitado renueva la marca de actividad.
        ahora = leer_tiempo();      /* Incluye fraccion al salir del reloj lento. */
        ultima_actividad = ahora;                                        // Guarda una accion valida para los umbrales de 10/30 s.
    }
    (void)prr_fijar_perfil(PRR_ACTIVO);                                  // Aplica el perfil de reloj de este estado despues de preparar los recursos.
    (void)uart_enviar_texto_P(PSTR("EV:ACTIVO\r\n"));                    // Encola aviso como EVENTO con checksum; no transmite esperando en main.
    prr_informar(PSTR("ACTIVO"));                                        // Publica el PRR leido para comprobar el perfil aplicado.
    bandera_adc = 1U;                                                    // Solicita: adquisicion A0 pendiente.
    bandera_lcd = 1U;                                                    // Solicita: actualizacion de pantalla pendiente.
    /*
     * Al volver de STANDBY se deja DATA en reposo y se espera un periodo
     * completo antes de pedir otra lectura. Esto evita consultar el DHT
     * mientras se estan restaurando los perifericos.
     */
    liberar_dht();
    estado_dht = DHT_REPOSO;
    dht_ultimo_inicio_ms = leer_tiempo();
    dht_intento_previo = 1U;
    error_dht = 0U;
    bandera_dht = 1U;
}

static void entrar_standby(void) {                                       // Reduce trabajo y perifericos manteniendo Timer1 y UART disponibles.
    cancelar_solicitud_sensores();                                       // Notifica la interrupcion de una solicitud antes del cambio de estado.
    modo_led = LED_AUTOMATICO;                                           // Selecciona parpadeo automatico o salida fija solicitada.
    estado = ESTADO_STANDBY;                                             // Actualiza la FSM principal; las acciones de entrada configuran sus recursos.
    adc_deshabilitar();                                                  // Borra ADEN solamente si el ADC tiene reloj y no esta convirtiendo.
    cancelar_dht();                                                      // Cancela captura y devuelve Timer2 al perfil de ahorro.
    PORTB &= (uint8_t)~(1U << LED_PIN);                                  // Deja LED apagado.
    twi_init();                    /* Puede proceder de un despertar APAGADO. */
    lcd_encender_luz();                                                  // Activa el bit de luz del expansor si se detecto la LCD.
    lcd_comando(0x0CU);                                                  // Enciende el display con cursor y parpadeo de cursor apagados.
    mostrar_mensaje_estado_P(PSTR("STANDBY"), PSTR("Esperando..."));     // Presenta el titulo y la indicacion al usuario con un solo buffer.
    apagar_sonido();                                                     // Detiene el sonido por la ruta local/permitida antes de seguir.
    reloj_standby();                                                     // Traslada el reloj a Timer1 para despertar una vez por segundo en IDLE.
    (void)prr_fijar_perfil(PRR_STANDBY);                                 // Aplica el perfil de reloj de este estado despues de preparar los recursos.
    (void)uart_enviar_texto_P(PSTR("EV:STANDBY\r\n"));                   // Encola aviso como EVENTO con checksum; no transmite esperando en main.
    prr_informar(PSTR("STANDBY"));                                       // Publica el PRR leido para comprobar el perfil aplicado.
}

static void entrar_apagado(void) {                                       // Apaga pantalla/luz y prepara Power-Down; el sueño ocurre al terminar TX.
    cancelar_solicitud_sensores();                                       // Notifica la interrupcion de una solicitud antes del cambio de estado.
    modo_led = LED_AUTOMATICO;                                           // Selecciona parpadeo automatico o salida fija solicitada.
    estado = ESTADO_APAGADO;                                             // Actualiza la FSM principal; las acciones de entrada configuran sus recursos.
    adc_deshabilitar();                                                  // Borra ADEN solamente si el ADC tiene reloj y no esta convirtiendo.
    cancelar_dht();                                                      // Cancela captura y devuelve Timer2 al perfil de ahorro.
    apagar_sonido();                                                     // Detiene el sonido por la ruta local/permitida antes de seguir.
    PORTB &= (uint8_t)~(1U << LED_PIN);                                  // Deja LED apagado.
    twi_init();                    /* TWI estaba detenido en STANDBY. */
    lcd_comando(0x0CU);                                                  // Enciende el display con cursor y parpadeo de cursor apagados.
    mostrar_mensaje_estado_P(PSTR("APAGADO"), PSTR("Pulse el boton"));   // Presenta el titulo y la indicacion al usuario con un solo buffer.
    _delay_ms(5);                                                        // Retardo acotado de 5 ms para el bus/LCD; no es un temporizador de tareas.
    lcd_comando(0x08U);                                                  // Apaga la visualizacion LCD; la luz se controla por otro bit del expansor.
    lcd_apagar_luz();                                                    // Apaga la retroiluminacion mediante el expansor, sin cortar su alimentacion.
    reloj_standby();               /* Tambien permite ACTIVO -> APAGADO por orden. */
    (void)prr_fijar_perfil(PRR_STANDBY); /* T1 y UART hasta completar los avisos. */
    (void)uart_enviar_texto_P(PSTR("EV:APAGADO\r\n"));                   // Encola aviso como EVENTO con checksum; no transmite esperando en main.
    prr_informar(PSTR("PREVIO_SUENO"));                                  // Publica el PRR leido para comprobar el perfil aplicado.
}

static void entrar_seguro(uint8_t causa) {                               // Registra la alarma, mantiene Timer0/UART y exige recuperacion local.
    cancelar_solicitud_sensores();                                       // Notifica la interrupcion de una solicitud antes del cambio de estado.
    registrar_alarma(causa);                                             // Guarda el evento antes de activar las acciones de emergencia.
    modo_led = LED_AUTOMATICO;                                           // Selecciona parpadeo automatico o salida fija solicitada.
    wdt_al_volver_a_ejecucion();                                         // En WDT 1 cancela despertador; en WDT 2 conserva supervision.
    restaurar_para_activo();       /* La alarma necesita Timer0 a 1 ms. */
    estado = ESTADO_SEGURO;                                              // Actualiza la FSM principal; las acciones de entrada configuran sus recursos.
    adc_deshabilitar();                                                  // Borra ADEN solamente si el ADC tiene reloj y no esta convirtiendo.
    cancelar_dht();                                                      // Cancela captura y devuelve Timer2 al perfil de ahorro.
    lcd_encender_luz();                                                  // Activa el bit de luz del expansor si se detecto la LCD.
    lcd_comando(0x0CU);                                                  // Enciende el display con cursor y parpadeo de cursor apagados.
    mostrar_mensaje_estado_P(PSTR("EMERGENCIA"), PSTR("Pulse para salir")); // Presenta el titulo y la indicacion al usuario con un solo buffer.
    iniciar_alarma();                                                    // La alarma queda activa; Timer0 la mantiene hasta reconocimiento local.
    (void)prr_fijar_perfil(PRR_SEGURO); /* Timer0 mantiene alarma y pulsador. */
    (void)uart_enviar_texto_P(PSTR("EV:SEGURO\r\n"));                    // Encola aviso como EVENTO con checksum; no transmite esperando en main.
    prr_informar(PSTR("SEGURO"));                                        // Publica el PRR leido para comprobar el perfil aplicado.
}

static void revisar_inactividad(uint32_t ahora) {                        // Aplica 10 s y 30 s desde la MISMA ultima actividad valida.
    uint32_t inactividad = (uint32_t)(ahora - ultima_actividad);         // Reserva: milisegundos logicos desde la ultima actividad valida.
    if (estado == ESTADO_ACTIVO &&                                       // Desde ACTIVO, diez segundos sin actividad llevan a la espera.
        inactividad >= TIEMPO_STANDBY_MS) {
        entrar_standby();                                                // Ejecuta las acciones de entrada al estado de espera IDLE.
    } else if (estado == ESTADO_STANDBY &&                               // Desde STANDBY, los 30 s siguen contandose desde la misma ultima actividad.
               inactividad >= TIEMPO_APAGADO_MS) {
        entrar_apagado();                                                // Prepara el estado de sueño profundo; espera TX antes de ejecutarlo.
    }
}

/* PULSADOR */

static void procesar_boton(uint32_t ahora) {                             // Valida evento y antirrebote; permite despertar o reconocer SEGURO localmente.
    uint8_t sreg = SREG, evento, salia_de_seguro;                        // Reserva: copia de SREG para restaurar el estado previo de interrupciones; copia local de la bandera del boton; recuerda si esta pulsacion reconoce una emergencia.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    evento = bandera_boton;                                              // Actualiza copia local de la bandera del boton.
    bandera_boton = 0U;                                                  // Consume/limpia: pulsacion pendiente de validar.
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
    if (!evento || boton_presionado || (PIND & _BV(BOTON_PIN)) != 0U) return; // Descarta falsos eventos, repeticion de una pulsacion y boton ya liberado.
    if ((uint32_t)(ahora - ultimo_boton) < ANTI_REBOTE_MS) return;       // Rechaza rebotes: no acepta eventos demasiado cercanos.
    salia_de_seguro = (estado == ESTADO_SEGURO) ? 1U : 0U;               // Actualiza recuerda si esta pulsacion reconoce una emergencia.
    if (salia_de_seguro) {                                               // Esta pulsacion local puede reconocer y apagar la alarma.
        apagar_sonido();                                                 // Detiene el sonido por la ruta local/permitida antes de seguir.
    }
    if (estado != ESTADO_ACTIVO) {                                       // Recupera ACTIVO antes de atender el evento que lo requiere.
        entrar_activo(ahora, 1U);                                        // Restaura las tareas/perifericos antes de tratar la actividad del usuario.
    }
    ahora = leer_tiempo();          /* Reloj rapido ya restaurado. */
    ultima_actividad = ahora;                                            // Guarda una accion valida para los umbrales de 10/30 s.
    ultimo_boton = ahora;                                                // Actualiza la marca usada por el antirrebote.
    inicio_boton = ahora;                                                // Desde esta marca se miden los 2 s de pulsacion mantenida.
    boton_presionado = 1U;                                               // Marca/libera el seguimiento de esta pulsacion en main.
    pulsacion_larga_lista = salia_de_seguro;                             // Evita generar otra emergencia por la misma pulsacion o reconocimiento.
    iniciar_sonido(DURACION_SOUNDER_MS);                                 // Solicita el aviso temporal sin un retardo de 500 ms en main.
}

static void revisar_boton_mantenido(uint32_t ahora) {                    // Detecta liberacion y pulsacion de 2 s para emergencia local.
    uint8_t presionado =
        ((PIND & (1U << BOTON_PIN)) == 0U) ? 1U : 0U;                    // El pulsador va a GND: nivel bajo significa presionado.
    if (boton_presionado && !presionado) {                               // La liberacion habilita el reconocimiento de una nueva pulsacion.
        boton_presionado = 0U;                                           // Marca/libera el seguimiento de esta pulsacion en main.
        pulsacion_larga_lista = 0U;                                      // Evita generar otra emergencia por la misma pulsacion o reconocimiento.
    }
    if (boton_presionado && presionado && !pulsacion_larga_lista &&      // Solo una pulsacion sostenida, aun no atendida, puede activar emergencia.
        estado != ESTADO_SEGURO &&
        (uint32_t)(ahora - inicio_boton) >=
            PULSACION_EMERGENCIA_MS) {
        pulsacion_larga_lista = 1U;                                      // Evita generar otra emergencia por la misma pulsacion o reconocimiento.
        entrar_seguro(ALARMA_LOCAL);                                     // Entra en emergencia y registra su causa; su salida requiere boton local.
    }
}

/* ACTIVIDAD 4 - PARSER Y COMANDOS, TODO FUERA DE LAS ISR
 * @COMANDO,DATO*HH + LF; XOR de COMANDO,DATO; CR opcional antes de LF.
 * Maximo 63 caracteres entre @ y LF; timeout total de 500 ms.
 * Una trama recibida puede esperar espacio de TX sin bloquear main. */

static void protocolo_actividad(uint8_t activar) {                       // Cuenta una orden aceptada y, si corresponde, recupera ACTIVO.
    protocolo_aceptadas++;                                               // Cuenta solo una orden validada y permitida.
    if (activar && estado != ESTADO_ACTIVO && estado != ESTADO_SEGURO) { // Un comando aceptado puede recuperar ACTIVO; SEGURO conserva su restriccion.
        entrar_activo(leer_tiempo(), 1U);                                // Restaura las tareas/perifericos antes de tratar la actividad del usuario.
    }
    ultima_actividad = leer_tiempo();                                    // Guarda una accion valida para los umbrales de 10/30 s.
}

static void protocolo_datos_sensores(char *respuesta) {                  // Formatea ADC/DHT con validez, edad y error; no inventa una lectura.
    COPIAR(respuesta, "ADC=");                                           // Copia la etiqueta literal desde Flash al buffer RAM.
    texto_agregar_u32(respuesta, valor_adc_actual);                      // Anexa el numero decimal usando el formateador compartido.
    texto_anexar_P(respuesta, PSTR(";T="));                                    // Anexa el campo/texto de Flash al mensaje RAM.
    if (lectura_dht_correcta) texto_agregar_u32(respuesta, temperatura); // Presenta el dato del DHT solo si su ultima captura fue valida.
    else texto_anexar_P(respuesta, PSTR("NA"));                                // Anexa el campo/texto de Flash al mensaje RAM.
    texto_anexar_P(respuesta, PSTR(";H="));                                    // Anexa el campo/texto de Flash al mensaje RAM.
    if (lectura_dht_correcta) texto_agregar_u32(respuesta, humedad);     // Presenta el dato del DHT solo si su ultima captura fue valida.
    else texto_anexar_P(respuesta, PSTR("NA"));                                // Anexa el campo/texto de Flash al mensaje RAM.
    texto_anexar_P(respuesta, PSTR(";DHT="));                                  // Anexa el campo/texto de Flash al mensaje RAM.
    texto_agregar_u32(respuesta, lectura_dht_correcta);                  // Anexa el numero decimal usando el formateador compartido.
    texto_anexar_P(respuesta, PSTR(";EDAD_MS="));                              // Anexa el campo/texto de Flash al mensaje RAM.
    if (lectura_dht_correcta) texto_agregar_u32(respuesta,               // Presenta el dato del DHT solo si su ultima captura fue valida.
                      (uint32_t)(leer_tiempo() - dht_ultima_lectura_ms)); // Edad segun el reloj logico: no incluye el tiempo transcurrido en Power-Down.
    else texto_anexar_P(respuesta, PSTR("NA"));                                // Anexa el campo/texto de Flash al mensaje RAM.
    texto_anexar_P(respuesta, PSTR(";ERR="));                                  // Anexa el campo/texto de Flash al mensaje RAM.
    texto_agregar_u32(respuesta, error_dht);                             // Anexa el numero decimal usando el formateador compartido.
}

static void protocolo_notificar_sensores(void) {                         // Envia el resultado diferido cuando esta listo y hay espacio TX.
    char respuesta[80];                                                  // Reserva: buffer RAM de respuesta de protocolo.
    if (solicitud_sensores && sensores_listos &&                         // Solo notifica/cancela si existe una solicitud de sensores pendiente.
        uart_tx_libre() >= PROTO_TRAMA_TAM) {                            // Calcula espacio libre modulo 256; una posicion queda reservada.
        protocolo_datos_sensores(respuesta);                             // Construye la respuesta final con validez y edad del DHT.
        if (uart_enviar_trama(PSTR("SENSORES"), respuesta)) {            // Encola una trama completa; main continua mientras la ISR transmite.
            solicitud_sensores = sensores_listos = 0U;                   // Cierra la solicitud para no notificarla otra vez.
        }
    }
}

static void protocolo_comando(const char *comando, const char *dato) {   // Valida argumentos y permisos del estado antes de actuar.
    char respuesta[80];                                                  // Reserva: buffer RAM de respuesta de protocolo.
    if (ES(comando, "ESTADO")) {                                         // Rama del comando ESTADO: validar dato/estado antes de ejecutar.
        if (!ES(dato, "?")) { ERROR_PROTO("DATO"); return; }             // Rechaza un argumento que no pertenece al comando, antes de actuar.
        protocolo_actividad(1U);                                         // Solo aqui la orden aceptada renueva actividad; puede recuperar ACTIVO.
        uart_informar_estado();                                          // Construye la respuesta de estado, PRR y emergencia.
    } else if (ES(comando, "SENSORES")) {                                // Rama del comando SENSORES: validar dato/estado antes de ejecutar.
        if (!ES(dato, "?")) { ERROR_PROTO("DATO"); return; }             // Rechaza un argumento que no pertenece al comando, antes de actuar.
        if (estado == ESTADO_SEGURO) { ERROR_PROTO("SEGURO"); return; }  // Impide que una orden remota desactive o modifique la emergencia.
        protocolo_actividad(1U);                                         // Solo aqui la orden aceptada renueva actividad; puede recuperar ACTIVO.
        valor_adc_actual = adc_leer_a0();                                // Adquiere o actualiza A0; el dato queda disponible para LCD/protocolo.
        solicitud_sensores = 1U;                                         // Deja al cliente esperando el resultado de una captura DHT posterior.
        sensores_listos = 0U;                                            // La nueva solicitud todavia no tiene un resultado final.
        bandera_dht = 1U;           /* La FSM respeta >=2 s entre intentos DHT. */
        (void)uart_enviar_trama_P(PSTR("OK"), PSTR("SENSORES;PENDIENTE")); // Encola una trama completa; main continua mientras la ISR transmite.
    } else if (ES(comando, "MODO")) {                                    // Rama del comando MODO: validar dato/estado antes de ejecutar.
        uint8_t destino;                                                 // Reserva: estado o mascara PRR solicitado.
        if (ES(dato, "ACTIVO")) destino = ESTADO_ACTIVO;                 // Traduce el argumento textual al modo solicitado o elige el resultado de consulta.
        else if (ES(dato, "STANDBY")) destino = ESTADO_STANDBY;          // Traduce el argumento textual al modo solicitado o elige el resultado de consulta.
        else if (ES(dato, "APAGADO")) destino = ESTADO_APAGADO;          // Traduce el argumento textual al modo solicitado o elige el resultado de consulta.
        else { ERROR_PROTO("DATO"); return; }                            // Rechaza un argumento que no pertenece al comando, antes de actuar.
        if (estado == ESTADO_SEGURO) { ERROR_PROTO("SEGURO"); return; }  // Impide que una orden remota desactive o modifique la emergencia.
        protocolo_actividad(0U);                                         // Solo aqui la orden aceptada renueva actividad; puede recuperar ACTIVO.
        if (estado != destino) {                                         // Evita repetir las acciones de entrada cuando ya se esta en ese estado.
            if (destino == ESTADO_ACTIVO) entrar_activo(leer_tiempo(), 1U); // Restaura las tareas/perifericos antes de tratar la actividad del usuario.
            else if (destino == ESTADO_STANDBY) entrar_standby();        // Si el destino es STANDBY, aplica sus acciones de entrada.
            else entrar_apagado();                                       // Prepara el estado de sueño profundo; espera TX antes de ejecutarlo.
        }
        COPIAR(respuesta, "MODO;");                                      // Copia la etiqueta literal desde Flash al buffer RAM.
        texto_anexar_P(respuesta, nombre_estado_P());                          // Anexa el campo/texto de Flash al mensaje RAM.
        (void)uart_enviar_trama(PSTR("OK"), respuesta);                  // Encola una trama completa; main continua mientras la ISR transmite.
    } else if (ES(comando, "LED")) {                                     // Rama del comando LED: validar dato/estado antes de ejecutar.
        uint8_t ajuste;                                                  // Reserva: modo LED o configuracion WDT que se aplicara.
        if (ES(dato, "0")) ajuste = LED_FORZADO_OFF;                     // Traduce el argumento textual al modo solicitado o elige el resultado de consulta.
        else if (ES(dato, "1")) ajuste = LED_FORZADO_ON;                 // Traduce el argumento textual al modo solicitado o elige el resultado de consulta.
        else if (ES(dato, "AUTO")) ajuste = LED_AUTOMATICO;              // Traduce el argumento textual al modo solicitado o elige el resultado de consulta.
        else { ERROR_PROTO("DATO"); return; }                            // Rechaza un argumento que no pertenece al comando, antes de actuar.
        if (estado == ESTADO_SEGURO) { ERROR_PROTO("SEGURO"); return; }  // Impide que una orden remota desactive o modifique la emergencia.
        protocolo_actividad(1U);                                         // Solo aqui la orden aceptada renueva actividad; puede recuperar ACTIVO.
        modo_led = ajuste;                                               // Selecciona parpadeo automatico o salida fija solicitada.
        if (ajuste == LED_FORZADO_OFF) PORTB &= (uint8_t)~_BV(LED_PIN);  // Deja LED apagado.
        else if (ajuste == LED_FORZADO_ON) PORTB |= _BV(LED_PIN);        // Fuerza LED encendido.
        COPIAR(respuesta, "LED;"); strcat(respuesta, dato);              // Copia la etiqueta literal desde Flash al buffer RAM.
        (void)uart_enviar_trama(PSTR("OK"), respuesta);                  // Encola una trama completa; main continua mientras la ISR transmite.
    } else if (ES(comando, "SONIDO")) {                                  // Rama del comando SONIDO: validar dato/estado antes de ejecutar.
        if (!ES(dato, "0") && !ES(dato, "1")) { ERROR_PROTO("DATO"); return; } // Rechaza un argumento que no pertenece al comando, antes de actuar.
        if (estado == ESTADO_SEGURO) { ERROR_PROTO("SEGURO"); return; }  // Impide que una orden remota desactive o modifique la emergencia.
        protocolo_actividad(1U);                                         // Solo aqui la orden aceptada renueva actividad; puede recuperar ACTIVO.
        if (ES(dato, "1")) iniciar_sonido(DURACION_SOUNDER_MS);          // Traduce el argumento textual al modo solicitado o elige el resultado de consulta.
        else apagar_sonido();                                            // Detiene el sonido por la ruta local/permitida antes de seguir.
        COPIAR(respuesta, "SONIDO;"); strcat(respuesta, dato);           // Copia la etiqueta literal desde Flash al buffer RAM.
        (void)uart_enviar_trama(PSTR("OK"), respuesta);                  // Encola una trama completa; main continua mientras la ISR transmite.
    } else if (ES(comando, "EMERGENCIA")) {                              // Rama del comando EMERGENCIA: validar dato/estado antes de ejecutar.
        if (!ES(dato, "1")) { ERROR_PROTO("DATO"); return; }             // Rechaza un argumento que no pertenece al comando, antes de actuar.
        protocolo_actividad(0U);                                         // Solo aqui la orden aceptada renueva actividad; puede recuperar ACTIVO.
        if (estado != ESTADO_SEGURO) entrar_seguro(ALARMA_REMOTA);       // La emergencia no se abandona por timeout de inactividad.
        (void)uart_enviar_trama_P(PSTR("OK"), PSTR("EMERGENCIA;SEGURO")); // Encola una trama completa; main continua mientras la ISR transmite.
    } else if (ES(comando, "ALARMAS")) {                                 // Rama del comando ALARMAS: validar dato/estado antes de ejecutar.
        if (ES(dato, "?")) {                                             // Traduce el argumento textual al modo solicitado o elige el resultado de consulta.
            protocolo_actividad(1U);                                     // Solo aqui la orden aceptada renueva actividad; puede recuperar ACTIVO.
            COPIAR(respuesta, "ALARMAS;TOTAL=");                         // Copia la etiqueta literal desde Flash al buffer RAM.
            texto_agregar_u32(respuesta, alarmas_total);                 // Anexa el numero decimal usando el formateador compartido.
            texto_anexar_P(respuesta, PSTR(";GUARDADAS="));                    // Anexa el campo/texto de Flash al mensaje RAM.
            texto_agregar_u32(respuesta, alarmas_guardadas);             // Anexa el numero decimal usando el formateador compartido.
            texto_anexar_P(respuesta, PSTR(";ACTIVA="));                       // Anexa el campo/texto de Flash al mensaje RAM.
            texto_agregar_u32(respuesta, estado == ESTADO_SEGURO ? 1U : 0U); // Anexa el numero decimal usando el formateador compartido.
        } else if (dato[0] >= '0' && dato[0] <= '3' && dato[1] == '\0') {
            uint8_t indice = (uint8_t)(dato[0] - '0');                   // Reserva: posicion de busqueda o registro.
            const alarma_evento_t *registro;                             // Reserva: puntero al evento de alarma que se escribe/consulta.
            if (indice >= alarmas_guardadas) { ERROR_PROTO("SIN_EVENTO"); return; } // Rechaza un indice de historial que aun no existe.
            protocolo_actividad(1U);                                     // Solo aqui la orden aceptada renueva actividad; puede recuperar ACTIVO.
            indice = (uint8_t)((alarmas_cabeza + ALARMAS_TAM - 1U - indice) % ALARMAS_TAM); // Indice 0 es la ultima alarma; resta circularmente hacia las anteriores.
            registro = &alarmas[indice];                                 // Apunta al registro solicitado sin copiar toda la estructura.
            COPIAR(respuesta, "ALARMA;ID="); texto_agregar_u32(respuesta, registro->numero); // Copia la etiqueta literal desde Flash al buffer RAM.
            texto_anexar_P(respuesta, PSTR(";CAUSA=")); texto_anexar_P(respuesta, nombre_alarma_P(registro->causa)); // Anexa el campo/texto de Flash al mensaje RAM.
            texto_anexar_P(respuesta, PSTR(";MS_RELOJ=")); texto_agregar_u32(respuesta, registro->tiempo); // Anexa el campo/texto de Flash al mensaje RAM.
        } else { ERROR_PROTO("DATO"); return; }                          // Rechaza un argumento que no pertenece al comando, antes de actuar.
        (void)uart_enviar_trama(PSTR("OK"), respuesta);                  // Encola una trama completa; main continua mientras la ISR transmite.
    } else if (ES(comando, "PRR")) {                                     // Rama del comando PRR: validar dato/estado antes de ejecutar.
        if (!ES(dato, "?")) { ERROR_PROTO("DATO"); return; }             // Rechaza un argumento que no pertenece al comando, antes de actuar.
        protocolo_actividad(1U);                                         // Solo aqui la orden aceptada renueva actividad; puede recuperar ACTIVO.
        prr_informar(PSTR("ACTUAL"));                                    // Publica el PRR leido para comprobar el perfil aplicado.
    } else if (ES(comando, "BLOQUEO")) {                                 // Rama del comando BLOQUEO: validar dato/estado antes de ejecutar.
        if (!ES(dato, "1")) { ERROR_PROTO("DATO"); return; }             // Rechaza un argumento que no pertenece al comando, antes de actuar.
        if (estado == ESTADO_SEGURO) { ERROR_PROTO("SEGURO"); return; }  // Impide que una orden remota desactive o modifique la emergencia.
        if (!wdt_solicitar_bloqueo()) ERROR_PROTO("PRUEBA2_NO_ACTIVA"); // No provoca bloqueo si la estrategia reset no esta seleccionada.
    } else {
        ERROR_PROTO("COMANDO");                                          // Rechaza un nombre de comando desconocido.
    }
}

static uint8_t protocolo_desde_hex(char c) {                             // Decodifica un digito hexadecimal; 0xFF identifica un caracter invalido.
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');                 // Convierte este rango ASCII al valor numerico del digito.
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);            // Convierte este rango ASCII al valor numerico del digito.
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);            // Convierte este rango ASCII al valor numerico del digito.
    return 0xFFU;                                                        // Valor centinela: el caracter no representa un digito hexadecimal.
}

static void uart_ejecutar_linea(void) {                                  // Verifica formato y XOR completos antes de despachar la orden.
    uint8_t i, calculado = 0U, alto;                                     // Reserva: indice de recorrido; XOR calculado con los caracteres del cuerpo; nibble alto del checksum recibido.
    uint8_t bajo;                                                        // Reserva: nibble bajo del checksum recibido.
    char *asterisco = NULL;                                             // Puntero RAM al primer *; NULL indica que aun no se encontro.
    char *coma = NULL;                                                  // Puntero RAM a la primera coma; no se convierte un retorno de biblioteca.
    for (i = 0U; i < uart_linea_longitud; i++) {                         // Busca dentro de los caracteres recibidos, sin salir del buffer.
        if (!asterisco && uart_linea[i] == '*') asterisco = &uart_linea[i]; // Conserva la primera coincidencia, igual que la busqueda anterior.
        if (!coma && uart_linea[i] == ',') coma = &uart_linea[i];       // La direccion se obtiene directamente del mismo buffer en SRAM.
    }
    /* Validar forma y checksum completos ANTES de modificar cualquier estado. */
    if (!asterisco || !coma || coma == uart_linea || coma >= asterisco || // Exige separadores y campos presentes, con checksum exactamente al final.
        coma + 1 == asterisco || (uint8_t)(asterisco - uart_linea) + 3U != uart_linea_longitud) {
        ERROR_PROTO("FORMATO"); return;                                  // Rechaza campos/separadores o digitos de checksum mal formados.
    }
    alto = protocolo_desde_hex(asterisco[1]);                            // Decodifica el primer digito del checksum recibido.
    bajo = protocolo_desde_hex(asterisco[2]);                            // Decodifica el segundo digito del checksum recibido.
    if (alto == 0xFFU || bajo == 0xFFU) { ERROR_PROTO("FORMATO"); return; } // Rechaza campos/separadores o digitos de checksum mal formados.
    for (i = 0U; &uart_linea[i] < asterisco; i++) calculado ^= (uint8_t)uart_linea[i]; // Calcula XOR del cuerpo, incluyendo la coma y excluyendo @ y *HH.
    if (calculado != (uint8_t)((alto << 4) | bajo)) { ERROR_PROTO("CHECKSUM"); return; } // Rechaza la trama si el XOR recibido no coincide.
    *asterisco = '\0';                                                   // Separa el cuerpo del checksum una vez validado.
    *coma = '\0';                                                        // Separa comando y dato dentro del mismo buffer, sin copiarlos.
    protocolo_comando(uart_linea, coma + 1);                             // Despacha comando y argumento ya separados y con checksum valido.
}

static void protocolo_reiniciar_rx(void) {                               // Borra el estado del parser para empezar una trama nueva.
    uart_en_trama = 0U;                                                  // Inicia o termina la recepcion entre @ y LF.
    uart_trama_pendiente = 0U;                                           // Retira la marca de trama completa pendiente.
    uart_linea_longitud = 0U;                                            // Vacia el texto parcial y permite empezar desde un nuevo inicio.
    uart_descartar_linea = 0U;                                           // Limpia/marca el texto recibido fuera de un inicio valido.
}

static void procesar_uart(uint32_t ahora) {                              // Procesa hasta 16 bytes o una orden por vuelta, con errores y timeout.
    uint8_t dato, cantidad, sreg = SREG;                                 // Reserva: byte o argumento que se esta procesando; bytes procesados en esta llamada; copia de SREG para restaurar el estado previo de interrupciones.
    uint8_t error;                                                       // Reserva: resultado o mascara de la validacion actual.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    error = uart_rx_error;                                               // Actualiza resultado o mascara de la validacion actual.
    uart_rx_error = 0U;                                                  // Consume los errores copiados; los nuevos se acumularan aparte.
    if (error) uart_rx_cola = uart_rx_cabeza; /* No unir datos a ambos lados de una perdida. */
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
    if (error) {                                                         // Tras una perdida RX, informa el error y comienza sin datos parciales viejos.
        char motivo[8] = "RX_00";                                        // Reserva: mascara RX convertida a texto para la respuesta ERROR.
        protocolo_reiniciar_rx();                                        // Descarta el estado parcial del parser para resincronizar.
        motivo[3] = protocolo_hex(error >> 4);                           // Inserta en ASCII un nibble de la mascara de error RX.
        motivo[4] = protocolo_hex(error);                                // Inserta en ASCII un nibble de la mascara de error RX.
        protocolo_errores++;                                             // Cuenta el rechazo/error que se informara al cliente.
        (void)uart_enviar_trama(PSTR("ERROR"), motivo);                  // Encola una trama completa; main continua mientras la ISR transmite.
    }
    if (uart_en_trama && (uint32_t)(ahora - uart_inicio_linea) >= UART_TIMEOUT_LINEA_MS) { // Mide inactividad desde el ultimo byte de la trama.
        protocolo_reiniciar_rx();                                        // Descarta el estado parcial del parser para resincronizar.
        ERROR_PROTO("TIMEOUT");                                          // Descarta una trama que no termino dentro del plazo.
    }
    for (cantidad = 0U; cantidad < PROTO_RX_PRESUPUESTO; cantidad++) {   // Hasta 16 bytes por llamada; el resto espera la siguiente vuelta de main.
        if (uart_rx_error) return; /* main vacia RX al comenzar la proxima vuelta. */
        if (uart_trama_pendiente) {                                      // Una trama completa espera a poder ser atendida y respondida.
            if (uart_tx_libre() < PROTO_RESERVA_TX) return;              // Cede main hasta disponer de espacio; no espera transmitiendo en un bucle.
            uart_linea[uart_linea_longitud] = '\0';                      // Termina la cadena para que las comparaciones no lean fuera de la trama.
            uart_ejecutar_linea();                                       // Solo una trama completa llega a formato, checksum y despacho.
            protocolo_reiniciar_rx();                                    // Descarta el estado parcial del parser para resincronizar.
            return;               /* Ceder a boton, sensores y WDT tras una orden. */
        }
        if (!uart_tomar_byte(&dato)) return;                             // Sin bytes disponibles cede main; no hace espera activa de recepcion.
        if (dato == '@') {                                               // Un nuevo @ inicia o resincroniza una trama.
            if (uart_en_trama) ERROR_PROTO("SIN_FIN");                   // Notifica que un nuevo @ interrumpio una trama incompleta.
            protocolo_reiniciar_rx();                                    // Descarta el estado parcial del parser para resincronizar.
            uart_en_trama = 1U;                                          // Inicia o termina la recepcion entre @ y LF.
            uart_inicio_linea = leer_tiempo();                           // Inicia el timeout de inactividad con este nuevo @.
        } else if (dato == '\r' || dato == '\n') {                       // Proteus puede enviar CR, LF o ambos al pulsar Enter.
            if (uart_en_trama) {                                         // El terminador solo completa una trama iniciada con @.
                uart_en_trama = 0U;                                      // Inicia o termina la recepcion entre @ y LF.
                uart_trama_pendiente = 1U;                               // La trama termino; falta validarla y atenderla cuando haya espacio TX.
            } else if (uart_descartar_linea) {
                uart_descartar_linea = 0U;                               // Limpia/marca el texto recibido fuera de un inicio valido.
                ERROR_PROTO("INICIO");                                   // Notifica una linea que no comenzo con @.
            }
        } else if (!uart_en_trama) {
            uart_descartar_linea = 1U;                                   // Texto sin @ se marca para informar INICIO al terminar la linea.
        } else if (dato < 0x20U || dato > 0x7EU) {                       // Dentro de la trama se exige ASCII imprimible.
            protocolo_reiniciar_rx();                                    // Descarta el estado parcial del parser para resincronizar.
            ERROR_PROTO("CARACTER");                                     // Rechaza un byte no permitido o CR fuera de posicion.
        } else if (uart_linea_longitud >= UART_LINEA_TAM - 1U) {         // Protege la ultima posicion reservada al terminador cero.
            protocolo_reiniciar_rx();                                    // Descarta el estado parcial del parser para resincronizar.
            ERROR_PROTO("LONGITUD");                                     // Evita escribir fuera del buffer de la trama.
        } else {
            uart_linea[uart_linea_longitud++] = (char)dato;              // Guarda el caracter recibido y avanza la longitud, ya comprobada.
            uart_inicio_linea = leer_tiempo();                           // Cada byte valido renueva el plazo de recepcion.
        }
    }
}

/* TAREAS COOPERATIVAS */

static void ejecutar_tareas(uint32_t ahora) {                            // Consume banderas de ACTIVO y ejecuta trabajo fuera de las ISR.
    if (bandera_led) {                                                   // Consume trabajo pendiente sin realizarlo dentro de la ISR.
        bandera_led = 0U;                                                // Consume/limpia: trabajo LED pendiente para main.
        if (modo_led == LED_AUTOMATICO) PORTB ^= (1U << LED_PIN);        // Invierte el estado del LED.
    }
    if (bandera_adc) {                                                   // Consume trabajo pendiente sin realizarlo dentro de la ISR.
        bandera_adc = 0U;                                                // Consume/limpia: adquisicion A0 pendiente.
        valor_adc_actual = adc_leer_a0();                                // Adquiere o actualiza A0; el dato queda disponible para LCD/protocolo.
        revisar_actividad_adc(valor_adc_actual, ahora);                  // Considera actividad un cambio de A0 mayor o igual a 20 cuentas, si esta habilitado.
    }
    tarea_dht(ahora);                                                    // Avanza la FSM del sensor desde main, con intervalo minimo y timeout.
    if (bandera_lcd) {                                                   // Consume trabajo pendiente sin realizarlo dentro de la ISR.
        bandera_lcd = 0U;                                                // Consume/limpia: actualizacion de pantalla pendiente.
        mostrar_activo();                                                // Prepara ADC y DHT, o el error correspondiente, para ambas filas.
    }
}

static void descartar_tareas(void) {                                     // Descarta tareas de sensores/LCD fuera de ACTIVO; conserva LED de alarma.
    bandera_adc = 0U;                                                    // Consume/limpia: adquisicion A0 pendiente.
    bandera_dht = 0U;                                                    // Consume/limpia: solicitud de iniciar DHT.
    bandera_lcd = 0U;                                                    // Consume/limpia: actualizacion de pantalla pendiente.
    if (bandera_led) {                                                   // Consume trabajo pendiente sin realizarlo dentro de la ISR.
        bandera_led = 0U;                                                // Consume/limpia: trabajo LED pendiente para main.
        if (estado == ESTADO_SEGURO) {                                   // En emergencia el LED mantiene su señal de estado.
            PORTB ^= (1U << LED_PIN);                                    // Invierte el estado del LED.
        } else {
            PORTB &= (uint8_t)~(1U << LED_PIN);                          // Deja LED apagado.
        }
    }
}

/* ACTIVIDAD 2 - DOS PRUEBAS INDEPENDIENTES.
 * WDT_PRUEBA_ACTIVA=1: interrupcion de 8 s como despertador.
 * WDT_PRUEBA_ACTIVA=2: reset de 2 s como recuperacion ante bloqueo. */

#if WDT_PRUEBA_ACTIVA == WDT_PRUEBA_DESPERTADOR
// PRUEBA 1 ACTIVA: WDT SOLO INTERRUPCION, INTERVALO NOMINAL DE 8 S.

static void wdt_configurar_interrupcion_8s(void) {                       // Configura WDIE=1/WDE=0 con la secuencia protegida del WDT.
    uint8_t sreg = SREG;                                                 // Reserva: copia de SREG para restaurar el estado previo de interrupciones.
    uint8_t permiso = _BV(WDCE) | _BV(WDE);                              // Reserva: WDCE/WDE para abrir la ventana protegida de WDT.
    uint8_t ajuste = _BV(WDIF) | _BV(WDIE) | _BV(WDP3) | _BV(WDP0);      // Reserva: modo LED o configuracion WDT que se aplicara.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    wdt_reset();                                                         // Ejecuta WDR: reinicia el contador WDT; no reinicia la CPU.
    MCUSR &= (uint8_t)~_BV(WDRF);                                        // Borra WDRF antes de cambiar WDT; este flag fuerza su habilitacion tras reset.
    // Dos STS consecutivos garantizan la ventana protegida de cuatro ciclos.
    __asm__ __volatile__(                                                // Dos escrituras consecutivas cumplen la ventana protegida de cuatro ciclos.
        "sts %[registro], %[permiso]\n\t"                                // Primera STS: escribe WDCE/WDE para permitir el cambio.
        "sts %[registro], %[ajuste]\n\t"                                 // Segunda STS: aplica WDIE/prescaler con WDE=0.
        :                                                                // Esta instruccion ensamblador no devuelve operandos a C.
        : [registro] "n" (_SFR_MEM_ADDR(WDTCSR)),                        // Direccion de WDTCSR como constante para la instruccion STS.
          [permiso] "r" (permiso), [ajuste] "r" (ajuste)                 // Entrega ambos valores en registros del CPU antes de ejecutar las STS.
        : "memory"                                                       // Indica al compilador efectos sobre memoria; evita reordenar accesos alrededor.
    );
    // WDIF se limpia escribiendo 1; WDE queda en 0, WDIE en 1.
    wdt_armado = 1U;                                                     // Recuerda si la estrategia despertador ya tiene su plazo corriendo.
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
}

static void wdt_iniciar_prueba(void) {                                   // Inicializa la estrategia WDT seleccionada mediante comentarios.
    wdt_disable();                                                       // Deshabilita WDT mediante la secuencia protegida de avr-libc.
    wdt_armado = 0U;                                                     // Recuerda si la estrategia despertador ya tiene su plazo corriendo.
    bandera_wdt = 0U;                                                    // Consume/limpia: despertar WDT pendiente de atender.
    (void)uart_enviar_texto_P(PSTR("PRUEBA1:IRQ_8S\r\n"));               // Encola aviso como EVENTO con checksum; no transmite esperando en main.
    // Se arma al entrar realmente en Power-Down, no durante ACTIVO.
}

static void wdt_antes_powerdown(void) {                                  // Coordina el WDT con la entrada intencional al sueño profundo.
    if (!wdt_armado) {                                                   // Arma WDT solo una vez; no reinicia su cuenta en cada intento de dormir.
        wdt_configurar_interrupcion_8s();                                // Arma la primera prueba para despertar sin provocar reset.
    }
    // No reiniciar el contador en cada vuelta: el intervalo sigue corriendo.
}

static void wdt_despues_powerdown(void) {                                // Mantiene o reanuda la estrategia WDT tras despertar.
    /* WDIE fue borrado por hardware al ejecutar ISR(WDT_vect).
       El rearme se realiza en main al procesar el evento. */
}

static void wdt_al_volver_a_ejecucion(void) {                            // Coordina el regreso por usuario sin mezclar las dos pruebas WDT.
    wdt_disable();                                                       // Deshabilita WDT mediante la secuencia protegida de avr-libc.
    wdt_armado = 0U;                                                     // Recuerda si la estrategia despertador ya tiene su plazo corriendo.
    bandera_wdt = 0U;                                                    // Consume/limpia: despertar WDT pendiente de atender.
    PORTB &= (uint8_t)~_BV(WDT_PRUEBA_PIN);                              // Termina/baja la marca observable D12.
}

static void wdt_procesar_evento(void) {                                  // Atiende el evento WDT desde main, nunca dentro de la ISR.
    uint8_t pendiente, sreg = SREG;                                      // Reserva: copia local del evento WDT; copia de SREG para restaurar el estado previo de interrupciones.
    uint32_t numero_evento;                                              // Reserva: copia atomica del contador WDT de 32 bits.
    char mensaje[64];                                                    // Reserva: buffer RAM para construir una notificacion.
    cli();                                                               // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
    pendiente = bandera_wdt;                                             // Actualiza copia local del evento WDT.
    bandera_wdt = 0U;                                                    // Consume/limpia: despertar WDT pendiente de atender.
    numero_evento = eventos_wdt;                                         // Actualiza copia atomica del contador WDT de 32 bits.
    SREG = sreg;                                                         // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
    if (!pendiente) return;                                              // Sin evento WDT, vuelve inmediatamente a las demas tareas.
    wdt_armado = 0U;                                                     // ISR(WDT_vect) borro WDIE; ya no esta armado.
    if (estado != ESTADO_APAGADO) {                                      // Una pulsacion o emergencia tiene prioridad sobre la lectura periodica.
        PORTB &= (uint8_t)~_BV(WDT_PRUEBA_PIN);                          // Termina/baja la marca observable D12.
        return;                    // La pulsacion/emergencia tiene prioridad.
    }
    // La CPU YA desperto: habilitar PRADC/ADEN antes de adquirir A0.
    adc_habilitar();                                                     // Devuelve reloj al ADC, selecciona AVCC/A0 y habilita prescaler 128.
    valor_adc_actual = adc_leer_a0();                                    // Adquiere o actualiza A0; el dato queda disponible para LCD/protocolo.
    adc_deshabilitar();                                                  // Borra ADEN solamente si el ADC tiene reloj y no esta convirtiendo.
    (void)prr_deshabilitar(PRR_ADC);                                     // Solicita quitar reloj, pasando antes por la validacion de recursos ocupados.
    texto_copiar_P(mensaje, PSTR("WDT,evento="));                              // Copia texto desde Flash; no reserva una copia permanente en SRAM.
    texto_agregar_u32(mensaje, numero_evento);                           // Anexa el numero decimal usando el formateador compartido.
    texto_anexar_P(mensaje, PSTR(",ADC="));                                    // Anexa el campo/texto de Flash al mensaje RAM.
    texto_agregar_u32(mensaje, valor_adc_actual);                        // Anexa el numero decimal usando el formateador compartido.
    texto_anexar_P(mensaje, PSTR(",nominal_ms=8000\r\n"));                     // Anexa el campo/texto de Flash al mensaje RAM.
    (void)uart_enviar_texto(mensaje);                                    // Encola aviso como EVENTO con checksum; no transmite esperando en main.
    wdt_configurar_interrupcion_8s();                                    // Rearma WDIE fuera de la ISR para el proximo intervalo.
    // No cambiar estado, ultima_actividad ni inventar tiempo transcurrido.
    // main termina TX y vuelve a Power-Down mediante gestionar_sueno().
}

static void wdt_supervisar_ciclo(void) {                                 // En prueba 2 atiende el WDT solo despues de una vuelta completa de main.
    // La prueba 1 NO supervisa bloqueos y no reinicia el contador en main.
}

static uint8_t wdt_solicitar_bloqueo(void) {                             // Rechaza el ensayo porque esta activa la prueba despertador.
    return 0U;
}
#else

// PRUEBA_2_RESET_INICIO

// PRUEBA 2: WDT COMO RECUPERACION, RESET NOMINAL DE 2 SEGUNDOS.
// Se selecciona definiendo WDT_PRUEBA_ACTIVA=WDT_PRUEBA_RESET.
// La peticion BLOQUEO provoca el bucle infinito SOLO cuando se recibe.

static void wdt_iniciar_prueba(void) {                                   // Inicializa la estrategia WDT seleccionada mediante comentarios.
    wdt_armado = 0U;                                                     // Recuerda si la estrategia despertador ya tiene su plazo corriendo.
    bandera_wdt = 0U;                                                    // Consume/limpia: despertar WDT pendiente de atender.
    wdt_reset();                                                         // Ejecuta WDR: reinicia el contador WDT; no reinicia la CPU.
    wdt_enable(WDTO_2S);            // WDE=1, WDIE=0: reset, sin interrupcion.
    (void)uart_enviar_texto_P(PSTR("PRUEBA2:RESET_2S\r\n"));             // Encola aviso como EVENTO con checksum; no transmite esperando en main.
}

static void wdt_antes_powerdown(void) {                                  // Coordina el WDT con la entrada intencional al sueño profundo.
    // Un sueno intencional no debe confundirse con el bloqueo del programa.
    // La prueba de recuperacion supervisa ACTIVO, IDLE y SEGURO.
    wdt_disable();                                                       // Deshabilita WDT mediante la secuencia protegida de avr-libc.
}

static void wdt_despues_powerdown(void) {                                // Mantiene o reanuda la estrategia WDT tras despertar.
    wdt_reset();                                                         // Ejecuta WDR: reinicia el contador WDT; no reinicia la CPU.
    wdt_enable(WDTO_2S);            // Reanuda supervision al volver del sueno.
}

static void wdt_al_volver_a_ejecucion(void) {                            // Coordina el regreso por usuario sin mezclar las dos pruebas WDT.
    // Se conserva la supervision: no se atiende el WDT desde ninguna ISR.
}

static void wdt_procesar_evento(void) {                                  // Atiende el evento WDT desde main, nunca dentro de la ISR.
    // En modo reset no se espera ISR(WDT_vect).
    bandera_wdt = 0U;                                                    // Consume/limpia: despertar WDT pendiente de atender.
}

static void wdt_supervisar_ciclo(void) {                                 // En prueba 2 atiende el WDT solo despues de una vuelta completa de main.
    // Solo main lo atiende, DESPUES de completar la iteracion normal.
    wdt_reset();                                                         // Ejecuta WDR: reinicia el contador WDT; no reinicia la CPU.
}

static uint8_t wdt_solicitar_bloqueo(void) {                             // Provoca el bloqueo solamente en la prueba de reset.
    protocolo_actividad(0U);                                             // Solo aqui la orden aceptada renueva actividad; puede recuperar ACTIVO.
    (void)uart_enviar_texto_P(PSTR("PRUEBA2:BLOQUEO_SIN_WDR\r\n"));      // Encola aviso como EVENTO con checksum; no transmite esperando en main.
    marca_bloqueo = MARCA_BLOQUEO_WDT;                                   // Guarda la firma antes del bloqueo intencional.
    marca_bloqueo_inversa = ~MARCA_BLOQUEO_WDT;                          // Guarda su complemento para detectar una marca accidental/invalida.
    PORTB |= _BV(WDT_PRUEBA_PIN);   // Marca el comienzo del bloqueo de ensayo.
    wdt_reset();                   // Comienza un plazo completo de 2 s.
    for (;;) {                                                           // Bloqueo intencional sin WDR: permite comprobar el reset del WDT.
        // BLOQUEO INTENCIONAL: las ISR pueden seguir, pero main no supervisa.
        // No colocar wdt_reset() aqui ni en las interrupciones.
    }
    return 1U;                     // Inalcanzable; satisface tambien compiladores antiguos.
}
// PRUEBA_2_RESET_FIN

#endif

/* PROGRAMA PRINCIPAL */

int main(void) {                                                         // Inicializa el sistema y ejecuta la planificacion cooperativa en un lazo infinito.
    uint32_t ahora;                                                      // Reserva: copia local del reloj logico.
    gpio_init();                                                         // Configura direcciones, pull-ups, INT0 y salidas inicialmente apagadas.
    adc_init();                                                          // Configura A0 como entrada analogica y prepara el conversor.
    timer2_init();                                                       // Prepara Timer2 normal para DHT y lo deja sin reloj hasta la captura.
    twi_init();                                                          // Devuelve reloj y configura TWI a 100 kHz antes de escribir la LCD.
    (void)lcd_init();                                                    // Prueba 0x27/0x3F y realiza la secuencia de inicializacion en cuatro bits.
    if (lcd_disponible) {                                                // Solo se escribe la pantalla si el expansor fue detectado.
        char mensaje1[17];                                               // Reserva: primera fila del aviso de arranque.
        char mensaje2[17];                                               // Reserva: segunda fila del aviso de arranque.
        linea_limpiar(mensaje1);                                         // Prepara la fila con espacios y terminador antes de formatearla.
        linea_limpiar(mensaje2);                                         // Prepara la fila con espacios y terminador antes de formatearla.
        linea_texto_P(mensaje1, 0U, PSTR("LAB 5 - B UART"));             // Coloca texto fijo desde Flash en las columnas indicadas.
        linea_texto_P(mensaje2, 0U, PSTR("INICIANDO..."));               // Coloca texto fijo desde Flash en las columnas indicadas.
        lcd_linea(0U, mensaje1);                                         // Transfiere la fila completa a la memoria de caracteres de la LCD.
        lcd_linea(1U, mensaje2);                                         // Transfiere la fila completa a la memoria de caracteres de la LCD.
        _delay_ms(800);                                                  // Aviso de arranque; ocurre antes de habilitar las IRQ y los timers de tareas.
    }
    timer0_init();                                                       // Configura CTC de 1 ms: 16 MHz / 64 / 250.
    timer1_init();                                                       // Configura CTC de 100 ms: 16 MHz / 64 / 25000.
    valor_adc_actual = adc_leer_a0();                                    // Adquiere o actualiza A0; el dato queda disponible para LCD/protocolo.
    referencia_adc_actividad = valor_adc_actual;                         // Establece la nueva referencia para detectar movimientos de A0.
    referencia_adc_valida = 1U;                                          // Evita interpretar la primera adquisicion como actividad.
    ultima_actividad = 0UL;                                              // Guarda una accion valida para los umbrales de 10/30 s.
    uart_init();                                                         // Devuelve reloj a USART y configura 9600 bps, 8N1 y recepcion por interrupcion.
    (void)uart_enviar_texto_P(PSTR("LAB5:B4\r\n"));                      // Encola aviso como EVENTO con checksum; no transmite esperando en main.
    if ((causa_reinicio & _BV(WDRF)) != 0U ||                            // Al arrancar, WDRF o una marca valida obligan a recuperar en SEGURO.
        (((causa_reinicio & _BV(PORF)) == 0U) &&
         marca_bloqueo == MARCA_BLOQUEO_WDT &&                           // Actualiza firma del ensayo de bloqueo en RAM .noinit.
         marca_bloqueo_inversa == ~MARCA_BLOQUEO_WDT)) {                 // El complemento debe coincidir para aceptar la marca de ensayo.
        entrar_seguro((causa_reinicio & _BV(WDRF)) != 0U ?               // Entra en emergencia y registra su causa; su salida requiere boton local.
                      ALARMA_WDT : ALARMA_TRAS_BLOQUEO);                 // Elige causa identificada WDT o recuperacion por marca sin flag disponible.
        if ((causa_reinicio & _BV(WDRF)) != 0U) {                        // Al arrancar, WDRF o una marca valida obligan a recuperar en SEGURO.
            (void)uart_enviar_texto_P(PSTR("RESET:WDT\r\n"));            // Encola aviso como EVENTO con checksum; no transmite esperando en main.
        } else {
            /* No atribuir causa si el cargador de arranque borro MCUSR. */
            (void)uart_enviar_texto_P(PSTR("RESET:TRAS_BLOQUEO_SIN_WDRF\r\n")); // Encola aviso como EVENTO con checksum; no transmite esperando en main.
        }
        (void)uart_enviar_texto_P(PSTR("RECUPERACION:SEGURO\r\n"));      // Encola aviso como EVENTO con checksum; no transmite esperando en main.
    } else {
        mostrar_activo();                                                // Prepara ADC y DHT, o el error correspondiente, para ambas filas.
        (void)prr_fijar_perfil(PRR_ACTIVO);                              // Aplica el perfil de reloj de este estado despues de preparar los recursos.
        (void)uart_enviar_texto_P(PSTR("EV:ACTIVO\r\n"));                // Encola aviso como EVENTO con checksum; no transmite esperando en main.
        prr_informar(PSTR("ACTIVO"));                                    // Publica el PRR leido para comprobar el perfil aplicado.
    }
    marca_bloqueo = 0UL;           /* Evita repetir la recuperacion en otro reset. */
    marca_bloqueo_inversa = 0UL;                                         // Invalida tambien el complemento de la marca consumida.
    sei();                                                               // Habilita las ISR despues de completar la inicializacion del sistema.
    wdt_iniciar_prueba();                                                // Activa la estrategia seleccionada despues de inicializar el resto.
    while (1) {                                                          // Lazo cooperativo permanente: reparte trabajo segun eventos y estado.
        /* Consumir tick y copiar tiempo en una sola seccion atomica. */
        uint8_t sreg = SREG;                                             // Reserva: copia de SREG para restaurar el estado previo de interrupciones.
        cli();                                                           // Bloquea interrupciones brevemente para que la operacion compartida sea atomica.
        ahora = tiempo_ms;                                               // Actualiza copia local del reloj logico.
        bandera_reloj = 0U;                                              // Consume/limpia: tick de STANDBY pendiente.
        SREG = sreg;                                                     // Restaura el estado previo: no habilita IRQ si antes estaban deshabilitadas.
        /* Respaldo si el boton ya estaba bajo al habilitar una fuente. */
        if (!boton_presionado && (PIND & _BV(BOTON_PIN)) == 0U) {        // Atiende tambien un boton que ya estaba bajo al configurar la fuente.
            bandera_boton = 1U;                                          // Solicita: pulsacion pendiente de validar.
        }
        procesar_boton(ahora);                                           // Valida evento y antirrebote; permite despertar o reconocer SEGURO localmente.
        procesar_uart(leer_tiempo());                                    // Procesa hasta 16 bytes o una orden por vuelta, con errores y timeout.
        ahora = leer_tiempo();                                           // Actualiza copia local del reloj logico.
        revisar_boton_mantenido(ahora);                                  // Detecta liberacion y pulsacion de 2 s para emergencia local.
        if (estado != ESTADO_SEGURO) {                                   // La emergencia no se abandona por timeout de inactividad.
            revisar_inactividad(ahora);                                  // Aplica 10 s y 30 s desde la MISMA ultima actividad valida.
        }
        if (estado == ESTADO_ACTIVO) {                                   // Solo ACTIVO ejecuta las tareas periodicas de sensores y LCD.
            ejecutar_tareas(ahora);                                      // Consume banderas de ACTIVO y ejecuta trabajo fuera de las ISR.
        } else {
            descartar_tareas();                                          // Descarta tareas de sensores/LCD fuera de ACTIVO; conserva LED de alarma.
        }
        protocolo_notificar_sensores();                                  // Envia el resultado diferido cuando esta listo y hay espacio TX.
        wdt_procesar_evento();     /* Lectura/registro fuera de la ISR. */
        wdt_supervisar_ciclo();    /* Solo la prueba 2 habilitada ejecuta WDR aqui. */
        gestionar_sueno();         /* Solo duerme en STANDBY o APAGADO. */
    }
}
