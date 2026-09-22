.include "m328pdef.inc"
.org 0x0000
    rjmp inicio

inicio: 
    ; Configurar salidas para LEDs: PB0-PB5 y PC0-PC1
    ldi r16, 0b00111111
    out DDRB, r16
    ldi r16, 0b00000011
    out DDRC, r16

    ; Apagar todos los LEDs inicialmente
    clr r16
    out PORTB, r16
    out PORTC, r16

    ; Configurar USART a 9600 baudios (fosc = 16 MHz)
    ldi r16, 103
    sts UBRR0L, r16
    ldi r16, 0
    sts UBRR0H, r16

    ; Habilitar Receptor (RXEN0)
    ldi r16, (1<<RXEN0)
    sts UCSR0B, r16

    ; Formato: 8 bits de datos, 1 bit de parada, sin paridad
    ldi r16, (1<<UCSZ01)|(1<<UCSZ00)
    sts UCSR0C, r16

esperar_rx:
    ; Esperar hasta recibir un dato (RXC0 = 1)
    lds r18, UCSR0A
    sbrs r18, RXC0
    rjmp esperar_rx

    ; Leer dato recibido del registro UDR0
    lds r17, UDR0

    ; Limpiar salidas antes de encender el LED activo
    clr r16
    out PORTB, r16
    out PORTC, r16

    ; Decodificador de 3 bits (0 a 7)
    cpi r17, 0
    breq led0
    cpi r17, 1
    breq led1
    cpi r17, 2
    breq led2
    cpi r17, 3
    breq led3
    cpi r17, 4
    breq led4
    cpi r17, 5
    breq led5
    cpi r17, 6
    breq led6
    cpi r17, 7
    breq led7

    rjmp esperar_rx        ; Si llega un dato fuera de rango ( > 7 )

led0:
    ldi r16, 0b00000001
    out PORTB, r16
    rjmp esperar_rx

led1:
    ldi r16, 0b00000010
    out PORTB, r16
    rjmp esperar_rx

led2:
    ldi r16, 0b00000100
    out PORTB, r16
    rjmp esperar_rx

led3:
    ldi r16, 0b00001000
    out PORTB, r16
    rjmp esperar_rx

led4:
    ldi r16, 0b00010000
    out PORTB, r16
    rjmp esperar_rx

led5:
    ldi r16, 0b00100000
    out PORTB, r16
    rjmp esperar_rx

led6:
    ldi r16, 0b00000001
    out PORTC, r16
    rjmp esperar_rx

led7:
    ldi r16, 0b00000010
    out PORTC, r16
    rjmp esperar_rx