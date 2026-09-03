
.include "m328pdef.inc"

.equ ORDEN_NINGUNA = 0
.equ ORDEN_ABRIR   = 1
.equ ORDEN_CERRAR  = 2

.equ EV_ABRIENDO   = 0
.equ EV_ABIERTA    = 1
.equ EV_CERRANDO   = 2
.equ EV_CERRADA    = 3
.equ EV_OBSTACULO  = 4
.equ EV_DETENIDO   = 5

.equ TIEMPO_L      = 0x88
.equ TIEMPO_H      = 0x13
.equ TIEMPO_REBOTE = 50

.equ UBRR_VALOR    = 103

.dseg
contador_L: .byte 1
contador_H: .byte 1
orden:      .byte 1
obstaculo:  .byte 1
antirrebote:.byte 1
eventos:    .byte 1

.cseg
.org 0x0000
    rjmp RESET
.org INT0addr
    rjmp ISR_BOTON_ABRIR
.org INT1addr
    rjmp ISR_BOTON_CERRAR
.org PCI2addr
    rjmp ISR_BOTON_OBSTACULO
.org OC0Aaddr
    rjmp ISR_TIMER0_COMPA

.org 0x0034

RESET:
    ldi r16, high(RAMEND)
    out SPH, r16
    ldi r16, low(RAMEND)
    out SPL, r16
    clr r1

    clr r16
    sts contador_L, r16
    sts contador_H, r16
    sts orden, r16
    sts obstaculo, r16
    sts antirrebote, r16
    sts eventos, r16

    rcall CONFIGURAR_PUERTOS
    rcall CONFIGURAR_USART
    rcall CONFIGURAR_INTERRUPCIONES
    rcall CONFIGURAR_TIMER0

    ldi r16, (1 << EV_CERRADA)
    sts eventos, r16
    sei

BUCLE_PRINCIPAL:
    cli
    lds r18, eventos
    clr r16
    sts eventos, r16
    sei

    tst r18
    breq BUCLE_PRINCIPAL
    rcall TRANSMITIR_EVENTOS
    rjmp BUCLE_PRINCIPAL

CONFIGURAR_PUERTOS:
    ldi r16, 0b00011111
    out DDRB, r16
    sbi DDRC, DDC0
    cbi DDRD, DDD2
    cbi DDRD, DDD3
    cbi DDRD, DDD4
    sbi PORTD, PORTD2
    sbi PORTD, PORTD3
    sbi PORTD, PORTD4

    cbi PORTB, PORTB0
    sbi PORTB, PORTB1
    cbi PORTB, PORTB2
    cbi PORTB, PORTB3
    cbi PORTB, PORTB4
    cbi PORTC, PORTC0
    ret

CONFIGURAR_USART:
    clr r16
    sts UBRR0H, r16
    ldi r16, low(UBRR_VALOR)
    sts UBRR0L, r16
    ldi r16, (1 << TXEN0)
    sts UCSR0B, r16
    ldi r16, (1 << UCSZ01) | (1 << UCSZ00)
    sts UCSR0C, r16
    ret

CONFIGURAR_INTERRUPCIONES:
    ldi r16, (1 << ISC01) | (1 << ISC11)
    sts EICRA, r16
    ldi r16, (1 << INTF0) | (1 << INTF1)
    out EIFR, r16
    ldi r16, (1 << INT0) | (1 << INT1)
    out EIMSK, r16

    ldi r16, (1 << PCIE2)
    sts PCICR, r16
    ldi r16, (1 << PCINT20)
    sts PCMSK2, r16
    ldi r16, (1 << PCIF2)
    out PCIFR, r16
    ret

CONFIGURAR_TIMER0:
    clr r16
    out TCNT0, r16
    ldi r16, (1 << WGM01)
    out TCCR0A, r16
    ldi r16, 249
    out OCR0A, r16
    ldi r16, (1 << OCF0A)
    out TIFR0, r16
    ldi r16, (1 << OCIE0A)
    sts TIMSK0, r16
    ldi r16, (1 << CS01) | (1 << CS00)
    out TCCR0B, r16
    ret

USART_TX_CARACTER:
USART_ESPERAR:
    lds r17, UCSR0A
    sbrs r17, UDRE0
    rjmp USART_ESPERAR
    sts UDR0, r16
    ret

USART_TX_CADENA:
    lpm r16, Z+
    tst r16
    breq USART_CADENA_FIN
    rcall USART_TX_CARACTER
    rjmp USART_TX_CADENA
USART_CADENA_FIN:
    ret

TRANSMITIR_EVENTOS:
    sbrs r18, EV_ABRIENDO
    rjmp TX_REVISAR_ABIERTA
    ldi ZH, high(2*MSG_ABRIENDO)
    ldi ZL, low(2*MSG_ABRIENDO)
    rcall USART_TX_CADENA
TX_REVISAR_ABIERTA:
    sbrs r18, EV_ABIERTA
    rjmp TX_REVISAR_CERRANDO
    ldi ZH, high(2*MSG_ABIERTA)
    ldi ZL, low(2*MSG_ABIERTA)
    rcall USART_TX_CADENA
TX_REVISAR_CERRANDO:
    sbrs r18, EV_CERRANDO
    rjmp TX_REVISAR_CERRADA
    ldi ZH, high(2*MSG_CERRANDO)
    ldi ZL, low(2*MSG_CERRANDO)
    rcall USART_TX_CADENA
TX_REVISAR_CERRADA:
    sbrs r18, EV_CERRADA
    rjmp TX_REVISAR_OBSTACULO
    ldi ZH, high(2*MSG_CERRADA)
    ldi ZL, low(2*MSG_CERRADA)
    rcall USART_TX_CADENA
TX_REVISAR_OBSTACULO:
    sbrs r18, EV_OBSTACULO
    rjmp TX_REVISAR_DETENIDO
    ldi ZH, high(2*MSG_OBSTACULO)
    ldi ZL, low(2*MSG_OBSTACULO)
    rcall USART_TX_CADENA
TX_REVISAR_DETENIDO:
    sbrs r18, EV_DETENIDO
    rjmp TX_FIN
    ldi ZH, high(2*MSG_DETENIDO)
    ldi ZL, low(2*MSG_DETENIDO)
    rcall USART_TX_CADENA
TX_FIN:
    ret

INICIAR_APERTURA:
    cbi PORTB, PORTB0
    cbi PORTB, PORTB1
    sbi PORTB, PORTB2
    cbi PORTB, PORTB3
    lds r16, eventos
    ori r16, (1 << EV_ABRIENDO)
    sts eventos, r16
    ret

INICIAR_CIERRE:
    cbi PORTB, PORTB0
    cbi PORTB, PORTB1
    cbi PORTB, PORTB2
    sbi PORTB, PORTB3
    lds r16, eventos
    ori r16, (1 << EV_CERRANDO)
    sts eventos, r16
    ret

DETENER_MOTORES:
    cbi PORTB, PORTB2
    cbi PORTB, PORTB3
    cbi PORTC, PORTC0
    ret

FINALIZAR_APERTURA:
    rcall DETENER_MOTORES
    sbi PORTB, PORTB0
    cbi PORTB, PORTB1
    lds r16, eventos
    ori r16, (1 << EV_ABIERTA)
    sts eventos, r16
    ret

FINALIZAR_CIERRE:
    rcall DETENER_MOTORES
    cbi PORTB, PORTB0
    sbi PORTB, PORTB1
    lds r16, eventos
    ori r16, (1 << EV_CERRADA)
    sts eventos, r16
    ret

ISR_BOTON_ABRIR:
    push r16
    in r16, SREG
    push r16

    lds r16, orden
    cpi r16, ORDEN_ABRIR
    breq SALIR_BOTON_ABRIR

    lds r16, antirrebote
    tst r16
    brne SALIR_BOTON_ABRIR
    ldi r16, TIEMPO_REBOTE
    sts antirrebote, r16

    ldi r16, ORDEN_ABRIR
    sts orden, r16
    ldi r16, TIEMPO_L
    sts contador_L, r16
    ldi r16, TIEMPO_H
    sts contador_H, r16

    lds r16, obstaculo
    tst r16
    brne ABRIR_BLOQUEADO
    rcall INICIAR_APERTURA
    rjmp SALIR_BOTON_ABRIR
ABRIR_BLOQUEADO:
    rcall DETENER_MOTORES
    cbi PORTB, PORTB0
    cbi PORTB, PORTB1
SALIR_BOTON_ABRIR:
    pop r16
    out SREG, r16
    pop r16
    reti

ISR_BOTON_CERRAR:
    push r16
    in r16, SREG
    push r16

    lds r16, orden
    cpi r16, ORDEN_CERRAR
    breq SALIR_BOTON_CERRAR

    lds r16, antirrebote
    tst r16
    brne SALIR_BOTON_CERRAR
    ldi r16, TIEMPO_REBOTE
    sts antirrebote, r16

    ldi r16, ORDEN_CERRAR
    sts orden, r16
    ldi r16, TIEMPO_L
    sts contador_L, r16
    ldi r16, TIEMPO_H
    sts contador_H, r16

    lds r16, obstaculo
    tst r16
    brne CERRAR_BLOQUEADO
    rcall INICIAR_CIERRE
    rjmp SALIR_BOTON_CERRAR
CERRAR_BLOQUEADO:
    rcall DETENER_MOTORES
    cbi PORTB, PORTB0
    cbi PORTB, PORTB1
SALIR_BOTON_CERRAR:
    pop r16
    out SREG, r16
    pop r16
    reti

ISR_BOTON_OBSTACULO:
    push r16
    in r16, SREG
    push r16

    sbic PIND, PIND4
    rjmp SALIR_BOTON_OBSTACULO
    lds r16, antirrebote
    tst r16
    brne SALIR_BOTON_OBSTACULO
    ldi r16, TIEMPO_REBOTE
    sts antirrebote, r16

    lds r16, obstaculo
    tst r16
    breq ACTIVAR_OBSTACULO

DESACTIVAR_OBSTACULO:
    clr r16
    sts obstaculo, r16
    cbi PORTB, PORTB4
    lds r16, orden
    cpi r16, ORDEN_ABRIR
    breq REANUDAR_APERTURA
    cpi r16, ORDEN_CERRAR
    breq REANUDAR_CIERRE
    rjmp SALIR_BOTON_OBSTACULO
REANUDAR_APERTURA:
    rcall INICIAR_APERTURA
    rjmp SALIR_BOTON_OBSTACULO
REANUDAR_CIERRE:
    rcall INICIAR_CIERRE
    rjmp SALIR_BOTON_OBSTACULO

ACTIVAR_OBSTACULO:
    ldi r16, 1
    sts obstaculo, r16
    sbi PORTB, PORTB4

    lds r16, eventos
    ori r16, (1 << EV_OBSTACULO)

    sbic PORTB, PORTB2
    ori r16, (1 << EV_DETENIDO)
    sbic PORTB, PORTB3
    ori r16, (1 << EV_DETENIDO)
    sts eventos, r16
    rcall DETENER_MOTORES

SALIR_BOTON_OBSTACULO:
    pop r16
    out SREG, r16
    pop r16
    reti

ISR_TIMER0_COMPA:
    push r16
    push r17
    in r16, SREG
    push r16

    lds r16, antirrebote
    tst r16
    breq CONTROLAR_BUZZER
    dec r16
    sts antirrebote, r16

CONTROLAR_BUZZER:
    in r16, PORTB
    andi r16, (1 << PORTB2) | (1 << PORTB3)
    breq APAGAR_BUZZER
    sbi PINC, PINC0
    rjmp REVISAR_OBSTACULO
APAGAR_BUZZER:
    cbi PORTC, PORTC0

REVISAR_OBSTACULO:
    lds r16, obstaculo
    tst r16
    brne SALIR_TIMER

    lds r16, contador_L
    lds r17, contador_H
    tst r16
    brne DECREMENTAR_CONTADOR
    tst r17
    breq SALIR_TIMER

DECREMENTAR_CONTADOR:
    subi r16, 1
    sbci r17, 0
    sts contador_L, r16
    sts contador_H, r17
    tst r16
    brne SALIR_TIMER
    tst r17
    brne SALIR_TIMER

    lds r16, orden
    cpi r16, ORDEN_ABRIR
    breq COMPLETAR_APERTURA
    cpi r16, ORDEN_CERRAR
    breq COMPLETAR_CIERRE
    rjmp BORRAR_ORDEN
COMPLETAR_APERTURA:
    rcall FINALIZAR_APERTURA
    rjmp BORRAR_ORDEN
COMPLETAR_CIERRE:
    rcall FINALIZAR_CIERRE
BORRAR_ORDEN:
    clr r16
    sts orden, r16

SALIR_TIMER:
    pop r16
    out SREG, r16
    pop r17
    pop r16
    reti

MSG_ABRIENDO:
    .db "Puerta abriendo", 13, 10, 0
MSG_ABIERTA:
    .db "Puerta abierta", 13, 10, 0, 0
MSG_CERRANDO:
    .db "Puerta cerrando", 13, 10, 0
MSG_CERRADA:
    .db "Puerta cerrada", 13, 10, 0, 0
MSG_OBSTACULO:
    .db "Obstaculo detectado", 13, 10, 0
MSG_DETENIDO:
    .db "Movimiento detenido por seguridad", 13, 10, 0
