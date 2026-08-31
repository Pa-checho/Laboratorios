.include "m328pdef.inc"

.cseg
.org 0x0000
    rjmp RESET

RESET:
    ldi r16, high(RAMEND)
    out SPH, r16
    ldi r16, low(RAMEND)
    out SPL, r16
    ldi r16, 0xFF
    out DDRD, r16
    ldi r16, 0x00
    out PORTD, r16
    ldi r16, 0x00
    out DDRB, r16
    ldi r16, 0x07
    out PORTB, r16
    ldi r17, 1
    ldi r22, 0

MAIN:
    rcall LEER_BOTONES
    cp r17, r22
    breq DESPACHO
    mov r22, r17
    rcall INICIAR_SECUENCIA

DESPACHO:
    cpi r17, 1
    brne REVISAR_2
    rjmp SECUENCIA_1

REVISAR_2:
    cpi r17, 2
    brne REVISAR_3
    rjmp SECUENCIA_2

REVISAR_3:
    cpi r17, 3
    brne REVISAR_4
    rjmp SECUENCIA_3

REVISAR_4:
    cpi r17, 4
    brne REVISAR_5
    rjmp SECUENCIA_4

REVISAR_5:
    cpi r17, 5
    brne REVISAR_6
    rjmp SECUENCIA_5

REVISAR_6:
    cpi r17, 6
    brne REVISAR_7
    rjmp SECUENCIA_6

REVISAR_7:
    cpi r17, 7
    brne REVISAR_8
    rjmp SECUENCIA_7

REVISAR_8:
    rjmp SECUENCIA_8

INICIAR_SECUENCIA:
    cpi r17, 1
    brne INICIAR_2
    ldi r21, 0x01
    ret

INICIAR_2:
    cpi r17, 2
    brne INICIAR_3
    ldi r21, 0x80
    ret

INICIAR_3:
    cpi r17, 3
    brne INICIAR_4
    ldi r21, 0xAA
    ret

INICIAR_4:
    cpi r17, 4
    brne INICIAR_5
    ldi r21, 0x01
    ret

INICIAR_5:
    cpi r17, 5
    brne INICIAR_6
    ldi r21, 0xFF
    ret

INICIAR_6:
    cpi r17, 6
    brne INICIAR_7
    ldi r21, 0x03
    ret

INICIAR_7:
    cpi r17, 7
    brne INICIAR_8
    ldi r21, 0
    ret

INICIAR_8:
    ldi r21, 0x00
    ret

SECUENCIA_1:
    out PORTD, r21
    lsl r21
    brne SEC1_PAUSA
    ldi r21, 0x01

SEC1_PAUSA:
    rcall PAUSA
    rjmp MAIN

SECUENCIA_2:
    out PORTD, r21
    lsr r21
    brne SEC2_PAUSA
    ldi r21, 0x80

SEC2_PAUSA:
    rcall PAUSA
    rjmp MAIN

SECUENCIA_3:
    out PORTD, r21
    com r21
    rcall PAUSA
    rjmp MAIN

SECUENCIA_4:
    out PORTD, r21
    cpi r21, 0xFF
    breq REINICIAR_4
    lsl r21
    ori r21, 0x01
    rjmp SEC4_PAUSA

REINICIAR_4:
    ldi r21, 0x01

SEC4_PAUSA:
    rcall PAUSA
    rjmp MAIN

SECUENCIA_5:
    out PORTD, r21
    lsr r21
    brne SEC5_PAUSA
    ldi r21, 0xFF

SEC5_PAUSA:
    rcall PAUSA
    rjmp MAIN

SECUENCIA_6:
    out PORTD, r21
    lsl r21
    lsl r21
    brne SEC6_PAUSA
    ldi r21, 0x03

SEC6_PAUSA:
    rcall PAUSA
    rjmp MAIN

SECUENCIA_7:
    cpi r21, 0
    brne SEC7_PASO_1
    ldi r16, 0x18
    out PORTD, r16
    rjmp SEC7_SIGUIENTE

SEC7_PASO_1:
    cpi r21, 1
    brne SEC7_PASO_2
    ldi r16, 0x24
    out PORTD, r16
    rjmp SEC7_SIGUIENTE

SEC7_PASO_2:
    cpi r21, 2
    brne SEC7_PASO_3
    ldi r16, 0x42
    out PORTD, r16
    rjmp SEC7_SIGUIENTE

SEC7_PASO_3:
    ldi r16, 0x81
    out PORTD, r16

SEC7_SIGUIENTE:
    inc r21
    cpi r21, 4
    brlo SEC7_PAUSA
    ldi r21, 0

SEC7_PAUSA:
    rcall PAUSA
    rjmp MAIN

SECUENCIA_8:
    out PORTD, r21
    com r21
    rcall PAUSA
    rjmp MAIN

LEER_BOTONES:
    sbis PINB, PB0
    rjmp BOTON_SIGUIENTE
    sbis PINB, PB1
    rjmp BOTON_ANTERIOR
    sbis PINB, PB2
    rjmp BOTON_REINICIO
    ret

BOTON_SIGUIENTE:
    inc r17
    cpi r17, 9
    brlo ESPERAR_PB0
    ldi r17, 1

ESPERAR_PB0:
    sbis PINB, PB0
    rjmp ESPERAR_PB0
    ret

BOTON_ANTERIOR:
    cpi r17, 1
    breq VOLVER_A_OCHO
    dec r17
    rjmp ESPERAR_PB1

VOLVER_A_OCHO:
    ldi r17, 8

ESPERAR_PB1:
    sbis PINB, PB1
    rjmp ESPERAR_PB1
    ret

BOTON_REINICIO:
    ldi r17, 1

ESPERAR_PB2:
    sbis PINB, PB2
    rjmp ESPERAR_PB2
    ret

PAUSA:
    ldi r23, 12

BUCLE_PAUSA:
    rcall RETARDO_CORTO
    rcall LEER_BOTONES
    dec r23
    brne BUCLE_PAUSA
    ret

RETARDO_CORTO:
    ldi r18, 2

RETARDO_1:
    ldi r19, 255

RETARDO_2:
    ldi r20, 255

RETARDO_3:
    dec r20
    brne RETARDO_3
    dec r19
    brne RETARDO_2
    dec r18
    brne RETARDO_1
    ret