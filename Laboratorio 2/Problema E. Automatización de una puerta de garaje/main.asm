; ETAPA 2: SE AGREGAN LOS LED DEL MOTOR Y MOVIMIENTO DE 5 S
; D8=abierta, D9=cerrada, D10=motor abriendo, D11=motor cerrando
.include "m328pdef.inc"
.equ NINGUNA=0
.equ ABRIR=1
.equ CERRAR=2
.equ T_L=0x88                 ; 5000 = 0x1388
.equ T_H=0x13
.dseg
contador_L:.byte 1
contador_H:.byte 1
orden:.byte 1
.cseg
.org 0
 rjmp RESET
.org INT0addr
 rjmp ISR_ABRIR
.org INT1addr
 rjmp ISR_CERRAR
.org OC0Aaddr
 rjmp ISR_TIMER0
.org 0x0034
RESET:
 ldi r16,high(RAMEND)
 out SPH,r16
 ldi r16,low(RAMEND)
 out SPL,r16
 clr r16
 sts contador_L,r16
 sts contador_H,r16
 sts orden,r16
 ldi r16,0b00001111          ; PB0..PB3 son salidas
 out DDRB,r16
 cbi PORTB,PORTB0
 sbi PORTB,PORTB1            ; al iniciar se considera cerrada
 cbi DDRD,DDD2
 cbi DDRD,DDD3
 sbi PORTD,PORTD2
 sbi PORTD,PORTD3
 ldi r16,(1<<ISC01)|(1<<ISC11)
 sts EICRA,r16
 ldi r16,(1<<INT0)|(1<<INT1)
 out EIMSK,r16
 ldi r16,(1<<WGM01)
 out TCCR0A,r16
 ldi r16,249
 out OCR0A,r16
 ldi r16,(1<<OCIE0A)
 sts TIMSK0,r16
 ldi r16,(1<<CS01)|(1<<CS00)
 out TCCR0B,r16
 sei
PRINCIPAL:
 rjmp PRINCIPAL

INICIAR_APERTURA:
 cbi PORTB,PORTB0
 cbi PORTB,PORTB1
 sbi PORTB,PORTB2            ; encender motor abriendo
 cbi PORTB,PORTB3
 ret
INICIAR_CIERRE:
 cbi PORTB,PORTB0
 cbi PORTB,PORTB1
 cbi PORTB,PORTB2
 sbi PORTB,PORTB3            ; encender motor cerrando
 ret
DETENER:
 cbi PORTB,PORTB2
 cbi PORTB,PORTB3
 ret

ISR_ABRIR:
 push r16
 in r16,SREG
 push r16
 rcall INICIAR_APERTURA
 ldi r16,ABRIR
 sts orden,r16
 ldi r16,T_L
 sts contador_L,r16
 ldi r16,T_H
 sts contador_H,r16
 pop r16
 out SREG,r16
 pop r16
 reti
ISR_CERRAR:
 push r16
 in r16,SREG
 push r16
 rcall INICIAR_CIERRE
 ldi r16,CERRAR
 sts orden,r16
 ldi r16,T_L
 sts contador_L,r16
 ldi r16,T_H
 sts contador_H,r16
 pop r16
 out SREG,r16
 pop r16
 reti

ISR_TIMER0:
 push r16
 push r17
 in r16,SREG
 push r16
 lds r16,contador_L
 lds r17,contador_H
 tst r16
 brne RESTAR
 tst r17
 breq FIN_TIMER
RESTAR:
 subi r16,1
 sbci r17,0
 sts contador_L,r16
 sts contador_H,r17
 tst r16
 brne FIN_TIMER
 tst r17
 brne FIN_TIMER
 rcall DETENER
 lds r16,orden
 cpi r16,ABRIR
 breq TERMINAR_ABRIR
 cpi r16,CERRAR
 breq TERMINAR_CERRAR
 rjmp BORRAR
TERMINAR_ABRIR:
 sbi PORTB,PORTB0
 cbi PORTB,PORTB1
 rjmp BORRAR
TERMINAR_CERRAR:
 cbi PORTB,PORTB0
 sbi PORTB,PORTB1
BORRAR:
 clr r16
 sts orden,r16
FIN_TIMER:
 pop r16
 out SREG,r16
 pop r17
 pop r16
 reti
