; ETAPA 1: CONTROL BASICO DE PUERTA
; ATmega328P, 16 MHz
; D2=abrir, D3=cerrar, D8=abierta, D9=cerrada

.include "m328pdef.inc"
.equ ORDEN_NINGUNA=0
.equ ORDEN_ABRIR=1
.equ ORDEN_CERRAR=2
.equ TIEMPO_L=0xB8
.equ TIEMPO_H=0x0B
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
 ldi r16,(1<<DDB0)|(1<<DDB1)
 out DDRB,r16
 cbi PORTB,PORTB0
 sbi PORTB,PORTB1
 cbi DDRD,DDD2
 cbi DDRD,DDD3
 sbi PORTD,PORTD2
 sbi PORTD,PORTD3
 ldi r16,(1<<ISC01)|(1<<ISC11)
 sts EICRA,r16
 ldi r16,(1<<INT0)|(1<<INT1)
 out EIMSK,r16
 ; Timer0 CTC produce una interrupcion cada 1 ms.
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
ISR_ABRIR:
 push r16
 in r16,SREG
 push r16
 cbi PORTB,PORTB0
 cbi PORTB,PORTB1
 ldi r16,ORDEN_ABRIR
 sts orden,r16
 ldi r16,TIEMPO_L
 sts contador_L,r16
 ldi r16,TIEMPO_H
 sts contador_H,r16
 pop r16
 out SREG,r16
 pop r16
 reti
ISR_CERRAR:
 push r16
 in r16,SREG
 push r16
 cbi PORTB,PORTB0
 cbi PORTB,PORTB1
 ldi r16,ORDEN_CERRAR
 sts orden,r16
 ldi r16,TIEMPO_L
 sts contador_L,r16
 ldi r16,TIEMPO_H
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
 lds r16,orden
 cpi r16,ORDEN_ABRIR
 breq MOSTRAR_ABIERTA
 cpi r16,ORDEN_CERRAR
 breq MOSTRAR_CERRADA
 rjmp BORRAR_ORDEN
MOSTRAR_ABIERTA:
 sbi PORTB,PORTB0
 cbi PORTB,PORTB1
 rjmp BORRAR_ORDEN
MOSTRAR_CERRADA:
 cbi PORTB,PORTB0
 sbi PORTB,PORTB1
BORRAR_ORDEN:
 clr r16
 sts orden,r16
FIN_TIMER:
 pop r16
 out SREG,r16
 pop r17
 pop r16
 reti
