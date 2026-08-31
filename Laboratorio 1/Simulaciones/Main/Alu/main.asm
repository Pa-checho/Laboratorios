.include "m328pdef.inc"

;Registros
.def numA = r17        ;Operando A (4 bits, D8-D11)
.def numB = r18        ;Operando B (4 bits, A0-A3)
.def selector = r19    ;Selector S (3 bits, S0=A4, S1=A5, S2=D12)
.def resultado = r20   ;Resultado F y Banderas
.def temp = r21        ;Registro auxiliar para banderas
.def lectura = r22     ;Registro auxiliar de lectura

.org 0x0000
rjmp setup

;Configuración de Puertos
setup:
    clr temp
    out DDRB, temp        ;Puerto B: Entradas (D8-D11 = A)
    out DDRC, temp        ;Puerto C: Entradas (A0-A3 = B)
    
    ser temp
    out DDRD, temp        ;Puerto D: Salidas (D0-D3 = F)

;Bucle Principal (Lectura)
loop:
    ;Leer Operando A
    in numA, PINB
    andi numA, 0x0F

    ;Leer Operando B (PC0-PC3 / A0-A3)
    in numB, PINC
    andi numB, 0x0F

    ;Leer Selector (PC4=S0, PC5=S1, PB4=S2)
    in temp, PINC
    lsr temp
    lsr temp
    lsr temp
    lsr temp
    andi temp, 0x03       ;temp tiene S0 y S1

    in lectura, PINB
    sbrc lectura, 4       ;Si D12 (PB4) es 1
    sbr temp, 0x04        ;Activa el bit S2
    
    mov selector, temp    ;Selector completo (0 a 7)

;3. Decodificador
    cpi selector, 0
    breq op_clear
    cpi selector, 1
    breq op_sub
    cpi selector, 2
    breq op_add
    cpi selector, 3
    breq op_xor
    cpi selector, 4
    breq op_and
    cpi selector, 5
    breq op_or
    cpi selector, 6
    breq op_shl
    cpi selector, 7
    breq op_inc
    rjmp loop

;4. Operaciones ALU
op_clear:
    clr resultado
    rjmp evaluar_flags

op_sub:
    mov resultado, numA
    sub resultado, numB
    rjmp evaluar_flags

op_add:
    mov resultado, numA
    add resultado, numB
    rjmp evaluar_flags

op_xor:
    mov resultado, numA
    eor resultado, numB
    rjmp evaluar_flags

op_and:
    mov resultado, numA
    and resultado, numB
    rjmp evaluar_flags

op_or:
    mov resultado, numA
    or resultado, numB
    rjmp evaluar_flags

op_shl:
    mov resultado, numA
    lsl resultado
    rjmp evaluar_flags

op_inc:
    mov resultado, numA
    subi resultado, -1
    rjmp evaluar_flags

;5. Manejo de Salidas y Banderas (Puerto D)
evaluar_flags:
    clr temp              ;Limpia el registro auxiliar de banderas

    ;1. Evaluar Signo REAL (Bit 7 del resultado de 8 bits)
    sbrc resultado, 7
    sbr temp, (1<<6)      ;Activa D6 (Signo) SOLO si el resultado es negativo

    ;2. Evaluar Carry (Bit 4 del resultado de 8 bits -> D4)
    sbrc resultado, 4
    sbr temp, (1<<4)      ;Activa D4 (Carry)

    ;3. Recortar resultado a 4 bits para los LEDs de salida (D0-D3)
    andi resultado, 0x0F

    ;4. Evaluar Cero (sobre los 4 bits del resultado -> D5)
    brne no_es_cero
    sbr temp, (1<<5)      ; Activa D5 (Cero)
no_es_cero:

    ;5. Unir Resultado (D0-D3) y Banderas (D4-D6) en el Puerto D
    or resultado, temp
    out PORTD, resultado

    rjmp loop