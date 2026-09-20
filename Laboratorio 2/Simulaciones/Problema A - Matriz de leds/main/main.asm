; Laboratorio 2, problema A
; Usamos una matriz de LEDs con el MAX7219 y la controlamos por UART
; Microcontrolador: ATmega328P
; Frecuencia: 16 MHz
;
; El programa muestra el mensaje "VIAJE ANTES QUE DESTINO" moviendose
; de derecha a izquierda. Desde la terminal se puede elegir el mensaje
; o una de las seis imagenes. Tambien se puede cambiar de opcion con
; los dos pulsadores.
;
; Conexiones de la matriz FC16/MAX7219
; DIN va a PB3, que es el pin D11 del Arduino
; CS va a PB2, que es el pin D10 del Arduino
; CLK va a PB5, que es el pin D13 del Arduino
;
; Los pulsadores usan los pull-up internos
; INT0 va a PD2, pin D2 del Arduino
; INT1 va a PD3, pin D3 del Arduino
;
; Para la UART usamos 9600 baudios y formato 8N1
; RXD va a PD0, pin D0 del Arduino
; TXD va a PD1, pin D1 del Arduino

.include "m328pdef.inc"

; Valores que usamos en el programa
.equ VELOCIDAD_SCROLL_MS       = 30
.equ INTENSIDAD_MATRIZ         = 3
.equ MENSAJE_TOTAL_COLUMNAS    = 148
.equ MENSAJE_POSICIONES        = MENSAJE_TOTAL_COLUMNAS-7

; Direcciones de los registros del MAX7219
.equ MAX_DECODE_MODE           = 0x09
.equ MAX_INTENSITY             = 0x0A
.equ MAX_SCAN_LIMIT            = 0x0B
.equ MAX_SHUTDOWN              = 0x0C
.equ MAX_DISPLAY_TEST          = 0x0F

; Reservamos estas variables en la SRAM
.dseg
modo_actual:        .byte 1     ; 0 es el mensaje y del 1 al 6 son las imagenes
desplazamiento:     .byte 1     ; indica desde que columna se muestra el mensaje
tiempo_scroll_L:    .byte 1     ; parte baja del contador de tiempo
tiempo_scroll_H:    .byte 1     ; parte alta del contador de tiempo
actualizar_matriz:  .byte 1     ; vale 1 cuando hay que refrescar la matriz

; A partir de aca empieza el programa
.cseg

; Vectores de interrupcion
.org 0x0000
    rjmp RESET

.org INT0addr
    rjmp ISR_INT0

.org INT1addr
    rjmp ISR_INT1

.org OC0Aaddr
    rjmp ISR_TIMER0_COMPA

.org INT_VECTORS_SIZE

; Configuracion inicial de todo lo que usa el programa
RESET:
    cli

    ; Primero configuramos la pila porque despues la usan las subrutinas
    ldi r16, high(RAMEND)
    out SPH, r16
    ldi r16, low(RAMEND)
    out SPL, r16

    ; Estos tres pines son las salidas que necesita el MAX7219
    ldi r16, (1<<DDB2) | (1<<DDB3) | (1<<DDB5)
    out DDRB, r16

    ; Dejamos CS en 1 para que la matriz no reciba datos todavia
    ldi r16, (1<<PB2)
    out PORTB, r16

    ; PD2 y PD3 quedan como entradas con los pull-up activados
    ; PD0 y PD1 se usan despues para la UART
    clr r16
    out DDRD, r16
    ldi r16, (1<<PD2) | (1<<PD3)
    out PORTD, r16

    ; Al encender empieza mostrando el mensaje desde el principio
    clr r16
    sts modo_actual, r16
    sts desplazamiento, r16
    ldi r16, low(VELOCIDAD_SCROLL_MS)
    sts tiempo_scroll_L, r16
    ldi r16, high(VELOCIDAD_SCROLL_MS)
    sts tiempo_scroll_H, r16
    ldi r16, 1
    sts actualizar_matriz, r16

    ; Configuramos el SPI como maestro y con una frecuencia de 1 MHz
    ldi r16, (1<<SPE) | (1<<MSTR) | (1<<SPR0)
    out SPCR, r16
    clr r16
    out SPSR, r16

    rcall INICIALIZAR_MAX7219
    rcall INICIALIZAR_USART

    ; Las interrupciones se activan al apretar los pulsadores
    ldi r16, (1<<ISC01) | (1<<ISC11)
    sts EICRA, r16
    ldi r16, (1<<INTF0) | (1<<INTF1)
    out EIFR, r16
    ldi r16, (1<<INT0) | (1<<INT1)
    out EIMSK, r16

    ; El Timer0 genera una interrupcion cada 1 ms
    ldi r16, (1<<WGM01)
    out TCCR0A, r16
    ldi r16, 249
    out OCR0A, r16
    clr r16
    out TCNT0, r16
    ldi r16, (1<<OCF0A)
    out TIFR0, r16
    ldi r16, (1<<OCIE0A)
    sts TIMSK0, r16
    ldi r16, (1<<CS01) | (1<<CS00)
    out TCCR0B, r16

    sei

    ; Mandamos el saludo y el menu a la terminal
    ldi r30, low(TEXTO_BIENVENIDA*2)
    ldi r31, high(TEXTO_BIENVENIDA*2)
    rcall USART_ENVIAR_CADENA
    rcall USART_MOSTRAR_MENU

; El programa queda revisando la UART y actualiza la matriz cuando hace falta
PRINCIPAL:
    rcall USART_REVISAR_ENTRADA

    lds r16, actualizar_matriz
    tst r16
    breq PRINCIPAL

    clr r16
    sts actualizar_matriz, r16
    rcall MOSTRAR_SELECCION
    rjmp PRINCIPAL

; Con INT0 pasamos a la opcion siguiente
ISR_INT0:
    push r16
    in   r16, SREG
    push r16
    push r17

    lds  r17, modo_actual
    inc  r17
    cpi  r17, 7
    brlo INT0_GUARDAR
    clr  r17

INT0_GUARDAR:
    sts  modo_actual, r17
    rjmp ISR_REINICIAR_SELECCION

; Con INT1 volvemos a la opcion anterior
ISR_INT1:
    push r16
    in   r16, SREG
    push r16
    push r17

    lds  r17, modo_actual
    tst  r17
    breq INT1_ENVOLVER
    dec  r17
    rjmp INT1_GUARDAR

INT1_ENVOLVER:
    ldi  r17, 6

INT1_GUARDAR:
    sts  modo_actual, r17

; Cada vez que cambiamos de opcion la mostramos desde el principio
ISR_REINICIAR_SELECCION:
    clr  r17
    sts  desplazamiento, r17
    ldi  r17, low(VELOCIDAD_SCROLL_MS)
    sts  tiempo_scroll_L, r17
    ldi  r17, high(VELOCIDAD_SCROLL_MS)
    sts  tiempo_scroll_H, r17
    ldi  r17, 1
    sts  actualizar_matriz, r17

    pop  r17
    pop  r16
    out  SREG, r16
    pop  r16
    reti

; Esta interrupcion se ejecuta cada 1 ms y controla el movimiento del texto
ISR_TIMER0_COMPA:
    push r16
    in   r16, SREG
    push r16
    push r17

    ; Si hay una imagen fija no necesitamos mover nada
    lds  r16, modo_actual
    tst  r16
    brne TIMER0_FIN

    ; Bajamos el contador de tiempo un milisegundo
    lds  r16, tiempo_scroll_L
    lds  r17, tiempo_scroll_H
    subi r16, 1
    sbci r17, 0
    sts  tiempo_scroll_L, r16
    sts  tiempo_scroll_H, r17

    ; Si el contador todavia no llego a cero esperamos otro tick
    tst  r16
    brne TIMER0_FIN
    tst  r17
    brne TIMER0_FIN

    ; Movemos el mensaje una columna y al final volvemos al comienzo
    lds  r16, desplazamiento
    inc  r16
    cpi  r16, MENSAJE_POSICIONES
    brlo TIMER0_GUARDAR_POSICION
    clr  r16

TIMER0_GUARDAR_POSICION:
    sts  desplazamiento, r16
    ldi  r16, 1
    sts  actualizar_matriz, r16

    ; Volvemos a cargar el tiempo para el proximo movimiento
    ldi  r16, low(VELOCIDAD_SCROLL_MS)
    sts  tiempo_scroll_L, r16
    ldi  r16, high(VELOCIDAD_SCROLL_MS)
    sts  tiempo_scroll_H, r16

TIMER0_FIN:
    pop  r17
    pop  r16
    out  SREG, r16
    pop  r16
    reti

; Revisamos si llego algun caracter desde la terminal
USART_REVISAR_ENTRADA:
    lds  r17, UCSR0A
    sbrs r17, RXC0
    ret

    lds  r16, UDR0

    cpi  r16, 'M'
    breq USART_PEDIR_MENU
    cpi  r16, 'm'
    breq USART_PEDIR_MENU

    ; Ignoramos Enter para que no aparezca como una entrada incorrecta
    cpi  r16, 13
    breq USART_ENTRADA_FIN
    cpi  r16, 10
    breq USART_ENTRADA_FIN

    ; Solo aceptamos numeros del 1 al 7
    cpi  r16, '1'
    brlo USART_ENTRADA_INVALIDA
    cpi  r16, '8'
    brsh USART_ENTRADA_INVALIDA

    ; Restamos el valor de '1' para obtener un numero entre 0 y 6
    subi r16, '1'
    rcall SELECCIONAR_MODO

    ldi  r30, low(TEXTO_OK*2)
    ldi  r31, high(TEXTO_OK*2)
    rcall USART_ENVIAR_CADENA
    ret

USART_PEDIR_MENU:
    rcall USART_MOSTRAR_MENU
    ret

USART_ENTRADA_INVALIDA:
    ldi  r30, low(TEXTO_ERROR*2)
    ldi  r31, high(TEXTO_ERROR*2)
    rcall USART_ENVIAR_CADENA

USART_ENTRADA_FIN:
    ret

; Guardamos el modo nuevo y reiniciamos el movimiento del mensaje
SELECCIONAR_MODO:
    cli
    sts  modo_actual, r16
    clr  r17
    sts  desplazamiento, r17
    ldi  r17, low(VELOCIDAD_SCROLL_MS)
    sts  tiempo_scroll_L, r17
    ldi  r17, high(VELOCIDAD_SCROLL_MS)
    sts  tiempo_scroll_H, r17
    ldi  r17, 1
    sts  actualizar_matriz, r17
    sei
    ret

; Configuracion de la comunicacion serie
INICIALIZAR_USART:
    clr r16
    sts UCSR0A, r16
    sts UBRR0H, r16
    ldi r16, 103
    sts UBRR0L, r16

    ; Activamos la recepcion y la transmision
    ldi r16, (1<<RXEN0) | (1<<TXEN0)
    sts UCSR0B, r16

    ; Usamos 8 bits de datos, sin paridad y un bit de parada
    ldi r16, (1<<UCSZ01) | (1<<UCSZ00)
    sts UCSR0C, r16
    ret

; Esperamos a que la UART este libre y mandamos el byte de r16
USART_ENVIAR_BYTE:
    lds  r17, UCSR0A
    sbrs r17, UDRE0
    rjmp USART_ENVIAR_BYTE
    sts  UDR0, r16
    ret

; Mandamos una cadena hasta encontrar el cero del final
USART_ENVIAR_CADENA:
    lpm  r16, Z+
    tst  r16
    breq USART_CADENA_FIN
    rcall USART_ENVIAR_BYTE
    rjmp USART_ENVIAR_CADENA

USART_CADENA_FIN:
    ret

; Mostramos el menu completo una linea por vez
USART_MOSTRAR_MENU:
    ldi r30, low(TEXTO_MENU_1*2)
    ldi r31, high(TEXTO_MENU_1*2)
    rcall USART_ENVIAR_CADENA
    ldi r30, low(TEXTO_MENU_2*2)
    ldi r31, high(TEXTO_MENU_2*2)
    rcall USART_ENVIAR_CADENA
    ldi r30, low(TEXTO_MENU_3*2)
    ldi r31, high(TEXTO_MENU_3*2)
    rcall USART_ENVIAR_CADENA
    ldi r30, low(TEXTO_MENU_4*2)
    ldi r31, high(TEXTO_MENU_4*2)
    rcall USART_ENVIAR_CADENA
    ldi r30, low(TEXTO_MENU_5*2)
    ldi r31, high(TEXTO_MENU_5*2)
    rcall USART_ENVIAR_CADENA
    ldi r30, low(TEXTO_MENU_6*2)
    ldi r31, high(TEXTO_MENU_6*2)
    rcall USART_ENVIAR_CADENA
    ldi r30, low(TEXTO_MENU_7*2)
    ldi r31, high(TEXTO_MENU_7*2)
    rcall USART_ENVIAR_CADENA
    ldi r30, low(TEXTO_MENU_FIN*2)
    ldi r31, high(TEXTO_MENU_FIN*2)
    rcall USART_ENVIAR_CADENA
    ret

; Preparamos el MAX7219 para trabajar con la matriz de 8 por 8
INICIALIZAR_MAX7219:
    ldi r16, MAX_DISPLAY_TEST
    clr r17
    rcall MAX_ESCRIBIR

    ; Lo dejamos apagado mientras terminamos de configurarlo
    ldi r16, MAX_SHUTDOWN
    clr r17
    rcall MAX_ESCRIBIR

    ldi r16, MAX_DECODE_MODE
    clr r17
    rcall MAX_ESCRIBIR

    ; Habilitamos las ocho filas de la matriz
    ldi r16, MAX_SCAN_LIMIT
    ldi r17, 7
    rcall MAX_ESCRIBIR

    ; Elegimos el brillo que definimos al principio
    ldi r16, MAX_INTENSITY
    ldi r17, INTENSIDAD_MATRIZ
    rcall MAX_ESCRIBIR

    ; Antes de encender borramos cualquier dato que haya quedado
    ldi r19, 1
    clr r17

MAX_LIMPIAR:
    mov r16, r19
    rcall MAX_ESCRIBIR
    inc r19
    cpi r19, 9
    brlo MAX_LIMPIAR

    ldi r16, MAX_SHUTDOWN
    ldi r17, 1
    rcall MAX_ESCRIBIR
    ret

; r16 tiene el registro del MAX7219 y r17 tiene el dato que mandamos
MAX_ESCRIBIR:
    cbi PORTB, PB2
    rcall SPI_ENVIAR
    mov r16, r17
    rcall SPI_ENVIAR
    sbi PORTB, PB2
    ret

; Enviamos por SPI el byte de r16 y esperamos a que termine
SPI_ENVIAR:
    out SPDR, r16

SPI_ESPERAR:
    in   r18, SPSR
    sbrs r18, SPIF
    rjmp SPI_ESPERAR
    in   r18, SPDR
    ret

; Segun el modo elegido mostramos el texto o una imagen
MOSTRAR_SELECCION:
    lds  r16, modo_actual
    tst  r16
    breq MOSTRAR_MENSAJE_SCROLL
    rcall MOSTRAR_IMAGEN
    ret

MOSTRAR_MENSAJE_SCROLL:
    rcall MOSTRAR_CUADRO_MENSAJE
    ret

; Buscamos en la tabla la imagen elegida y mandamos sus ocho filas
MOSTRAR_IMAGEN:
    push r16
    push r17
    push r18
    push r19
    push r30
    push r31

    ldi  r30, low(TABLA_IMAGENES*2)
    ldi  r31, high(TABLA_IMAGENES*2)
    clr  r18

    ; Cada imagen ocupa ocho bytes, por eso multiplicamos por ocho
    lds  r16, modo_actual
    dec  r16
    lsl  r16
    lsl  r16
    lsl  r16
    add  r30, r16
    adc  r31, r18

    ldi  r19, 1

MOSTRAR_IMAGEN_FILA:
    lpm  r17, Z+
    mov  r16, r19
    rcall MAX_ESCRIBIR
    inc  r19
    cpi  r19, 9
    brlo MOSTRAR_IMAGEN_FILA

    pop  r31
    pop  r30
    pop  r19
    pop  r18
    pop  r17
    pop  r16
    ret

; Armamos las filas usando las ocho columnas visibles del mensaje
MOSTRAR_CUADRO_MENSAJE:
    push r16
    push r17
    push r18
    push r19
    push r20
    push r21
    push r22
    push r23
    push r24
    push r25
    push r30
    push r31

    ldi  r19, 1                   ; empezamos por la primera fila
    ldi  r20, 0x01                ; esta mascara marca la fila que estamos leyendo

MENSAJE_BUCLE_FILAS:
    ; Para cada fila volvemos al comienzo de la parte visible
    ldi  r30, low(MENSAJE_COLUMNAS*2)
    ldi  r31, high(MENSAJE_COLUMNAS*2)
    lds  r24, desplazamiento
    clr  r25
    add  r30, r24
    adc  r31, r25

    clr  r21                      ; aca vamos armando el byte de la fila
    ldi  r22, 8                   ; la matriz siempre muestra ocho columnas

MENSAJE_BUCLE_COLUMNAS:
    lsl  r21                      ; dejamos lugar para el proximo LED
    lpm  r23, Z+
    and  r23, r20
    breq MENSAJE_LED_APAGADO
    ori  r21, 1

MENSAJE_LED_APAGADO:
    dec  r22
    brne MENSAJE_BUCLE_COLUMNAS

    mov  r16, r19
    mov  r17, r21
    rcall MAX_ESCRIBIR

    inc  r19
    lsl  r20
    cpi  r19, 9
    brlo MENSAJE_BUCLE_FILAS

    pop  r31
    pop  r30
    pop  r25
    pop  r24
    pop  r23
    pop  r22
    pop  r21
    pop  r20
    pop  r19
    pop  r18
    pop  r17
    pop  r16
    ret

; Textos que se mandan a la terminal
TEXTO_BIENVENIDA:
    .db 13,10,"=== LABORATORIO 2 - PROBLEMA A ===",13,10,"Matriz FC16/MAX7219 - UART 9600 8N1",13,10,0
TEXTO_MENU_1:
    .db 13,10,"1 - Mensaje: VIAJE ANTES QUE DESTINO",13,10,0,0
TEXTO_MENU_2:
    .db "2 - Carita sonriendo",13,10,0,0
TEXTO_MENU_3:
    .db "3 - Carita guinando",13,10,0
TEXTO_MENU_4:
    .db "4 - Corazon",13,10,0
TEXTO_MENU_5:
    .db "5 - :3",13,10,0,0
TEXTO_MENU_6:
    .db "6 - Asterisco",13,10,0
TEXTO_MENU_7:
    .db "7 - XD",13,10,0,0
TEXTO_MENU_FIN:
    .db "M - Repetir menu",13,10,"> ",0,0
TEXTO_OK:
    .db 13,10,"Opcion aplicada. Ingrese 1..7 o M.",13,10,"> ",0,0
TEXTO_ERROR:
    .db 13,10,"Entrada invalida. Ingrese 1..7 o M.",13,10,"> ",0

; Cada imagen esta guardada como ocho filas de ocho bits
TABLA_IMAGENES:

; Carita sonriendo
    .db 0b00111100, 0b01000010, 0b10100101, 0b10000001
    .db 0b10100001, 0b10011101, 0b01000010, 0b00111100

; Carita guinando
    .db 0b00111100, 0b01000010, 0b10100001, 0b10001101
    .db 0b10100001, 0b10011101, 0b01000010, 0b00111100

; Corazon
    .db 0b00000000, 0b01100110, 0b11111111, 0b11111111
    .db 0b01111110, 0b00111100, 0b00011000, 0b0000000

; Carita :3
    .db 0b00000000, 0b00011100, 0b01000010, 0b00011100
    .db 0b01000010, 0b00011100, 0b00000000, 0b00000000

; Asterisco
    .db 0b00011000, 0b00011000, 0b00111100, 0b11111111
    .db 0b01111110, 0b00111100, 0b01100110, 0b01000010

; XD
    .db 0b00000000, 0b10101110, 0b10101001, 0b01001001
    .db 0b10101001, 0b10101110, 0b00000000, 0b00000000

; El mensaje esta armado por columnas con una fuente de 5 por 7
; El bit 0 es la fila de arriba y el bit 6 es la fila de abajo
MENSAJE_COLUMNAS:
    ; Dejamos ocho columnas vacias para que el texto entre desde afuera
    .db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00

    ; Palabra VIAJE
    .db 0x1F,0x20,0x40,0x20,0x1F,0x00
    .db 0x00,0x41,0x7F,0x41,0x00,0x00
    .db 0x7E,0x11,0x11,0x11,0x7E,0x00
    .db 0x20,0x40,0x41,0x3F,0x01,0x00
    .db 0x7F,0x49,0x49,0x49,0x41,0x00

    ; Espacio entre las palabras
    .db 0x00,0x00,0x00,0x00

    ; Palabra ANTES
    .db 0x7E,0x11,0x11,0x11,0x7E,0x00
    .db 0x7F,0x02,0x04,0x08,0x7F,0x00
    .db 0x01,0x01,0x7F,0x01,0x01,0x00
    .db 0x7F,0x49,0x49,0x49,0x41,0x00
    .db 0x46,0x49,0x49,0x49,0x31,0x00

    ; Espacio entre las palabras
    .db 0x00,0x00,0x00,0x00

    ; Palabra QUE
    .db 0x3E,0x41,0x51,0x21,0x5E,0x00
    .db 0x3F,0x40,0x40,0x40,0x3F,0x00
    .db 0x7F,0x49,0x49,0x49,0x41,0x00

    ; Espacio entre las palabras
    .db 0x00,0x00,0x00,0x00

    ; Palabra DESTINO
    .db 0x7F,0x41,0x41,0x22,0x1C,0x00
    .db 0x7F,0x49,0x49,0x49,0x41,0x00
    .db 0x46,0x49,0x49,0x49,0x31,0x00
    .db 0x01,0x01,0x7F,0x01,0x01,0x00
    .db 0x00,0x41,0x7F,0x41,0x00,0x00
    .db 0x7F,0x02,0x04,0x08,0x7F,0x00
    .db 0x3E,0x41,0x41,0x41,0x3E,0x00

    ; Estas columnas vacias hacen que el texto termine de salir de la matriz
    .db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
