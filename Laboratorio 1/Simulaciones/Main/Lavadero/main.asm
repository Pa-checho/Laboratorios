

.include "m328pdef.inc"



.def temp   = r16
.def carga  = r17
.def ciclos = r18
.def tiempo = r19



.org 0x0000
    rjmp RESET


; INT0 -> D2 -> SW3

.org INT0addr
    rjmp ISR_SEGURIDAD


; INT1 -> D3 -> SW4

.org INT1addr
    rjmp ISR_SEGURIDAD


.org 0x0034




RESET:

    cli




    ldi temp, high(RAMEND)
    out SPH, temp

    ldi temp, low(RAMEND)
    out SPL, temp



    ldi temp, 0b11110000
    out DDRD, temp


    

    ldi temp, 0b00111111
    out DDRB, temp


   

    ldi temp, 0
    out PORTD, temp

    out PORTB, temp


    

    ldi temp, (1<<ISC01) | (1<<ISC00) | (1<<ISC11) | (1<<ISC10)

    sts EICRA, temp


    ; Inicialmente deshabilitadas

    ldi temp, 0
    out EIMSK, temp


    ; Limpiar banderas

    ldi temp, (1<<INTF0) | (1<<INTF1)
    out EIFR, temp


    sei

  

    ldi carga, 3


    rjmp ESTADO_LISTO




ESTADO_LISTO:

    rcall APAGAR_LEDS


    ; LED1 = D4

    sbi PORTD, 4


    rcall MOSTRAR_CARGA





ESPERAR_LISTO:

    

    sbic PIND, 1

    rjmp REVISAR_SW1


    ; Antirrebote

    rcall DELAY_REBOTE


    ; Confirmar que sigue en LOW

    sbis PIND, 1

    rjmp SELECCIONAR_CARGA


    rjmp ESPERAR_LISTO




REVISAR_SW1:

  

    sbic PIND, 0

    rjmp ESPERAR_LISTO


   

    cpi carga, 3

    breq ESPERAR_LISTO


    rcall DELAY_REBOTE


    ; Confirmar SW1

    sbis PIND, 0

    rjmp INICIAR_PROCESO


    rjmp ESPERAR_LISTO




INICIAR_PROCESO:

    ; Esperar que SW1 vuelva a HIGH

    rcall ESPERAR_SOLTAR_SW1


    ; Verificar puerta y agua

    rcall VERIFICAR_CONDICIONES


    

    ldi temp, (1<<INTF0) | (1<<INTF1)
    out EIFR, temp


    

    ldi temp, (1<<INT0) | (1<<INT1)
    out EIMSK, temp



    rcall ESTADO_LAVADO

    rcall ESTADO_CENTRIFUGADO

    rcall ESTADO_SECADO


    

    ldi temp, 0
    out EIMSK, temp


    rcall ESTADO_FIN


    rjmp ESTADO_LISTO




SELECCIONAR_CARGA:

    

    inc carga


    ; Si supera 2 -> volver a 0

    cpi carga, 3

    brlo CARGA_VALIDA


    ldi carga, 0



CARGA_VALIDA:

    rcall MOSTRAR_CARGA





SELECCION_CARGA_LOOP:

    ; Si SW2 pasa a HIGH, terminar

    sbic PIND, 1

    rjmp CARGA_SELECCIONADA


    ; Esperar aproximadamente 1 segundo

    ldi tiempo, 1

    rcall DELAY_CARGA


    ; Revisar nuevamente

    sbic PIND, 1

    rjmp CARGA_SELECCIONADA


    ; Cambiar carga

    inc carga


    cpi carga, 3

    brlo CARGA_VALIDA_CONT


    ldi carga, 0



CARGA_VALIDA_CONT:

    rcall MOSTRAR_CARGA

    rjmp SELECCION_CARGA_LOOP





CARGA_SELECCIONADA:

    rcall DELAY_REBOTE

    rjmp ESTADO_LISTO





DELAY_CARGA:


DELAY_CARGA_LOOP:

    rcall DELAY_1S

    dec tiempo

    brne DELAY_CARGA_LOOP

    ret





ESPERAR_SOLTAR_SW1:


SOLTAR_SW1:



    sbis PIND, 0

    rjmp SOLTAR_SW1


    ret





VERIFICAR_CONDICIONES:




ESPERAR_PUERTA:


    sbic PIND, 0

    rjmp COMPROBAR_PUERTA


    rjmp REINICIAR



COMPROBAR_PUERTA:

  

    sbic PIND, 2

    rjmp ESPERAR_PUERTA





ESPERAR_AGUA:

    ; SW1 permite cancelar

    sbic PIND, 0

    rjmp COMPROBAR_AGUA


    rjmp REINICIAR



COMPROBAR_AGUA:

    ; PD3 debe estar LOW

    sbic PIND, 3

    rjmp ESPERAR_AGUA


    ret





;
; INT0 = SW3
; INT1 = SW4
;


ISR_SEGURIDAD:

  

    push temp



    in temp, SREG

    push temp


   

    in temp, PORTB

    push temp


  

    cbi PORTB, 0

    cbi PORTB, 1




ESPERAR_SEGURIDAD:

    
    sbic PIND, 2

    rjmp ESPERAR_SEGURIDAD


    

    sbic PIND, 3

    rjmp ESPERAR_SEGURIDAD


   

    pop temp

    out PORTB, temp


    ; Recuperar SREG

    pop temp

    out SREG, temp


    ; Recuperar r16

    pop temp


    reti




ESTADO_LAVADO:

    rcall APAGAR_LEDS


    ; LED2 = D5

    sbi PORTD, 5


    ; 5 ciclos

    ldi ciclos, 5



CICLO_LAVADO:

    

    sbi PORTB, 0


    rcall TIEMPO_LAVADO_GIRO


    cbi PORTB, 0


    ; Pausa

    rcall TIEMPO_LAVADO_PAUSA


    dec ciclos

    brne CICLO_LAVADO


    ret





TIEMPO_LAVADO_GIRO:

    cpi carga, 0

    breq GIRO_CHICA


    cpi carga, 1

    breq GIRO_MEDIANA


    ldi tiempo, 5

    rcall EJECUTAR_SEGUNDOS

    ret



GIRO_CHICA:

    ldi tiempo, 3

    rcall EJECUTAR_SEGUNDOS

    ret



GIRO_MEDIANA:

    ldi tiempo, 4

    rcall EJECUTAR_SEGUNDOS

    ret





TIEMPO_LAVADO_PAUSA:

    cpi carga, 0

    breq PAUSA_CHICA


    cpi carga, 1

    breq PAUSA_MEDIANA


    ldi tiempo, 4

    rcall EJECUTAR_SEGUNDOS

    ret



PAUSA_CHICA:

    ldi tiempo, 2

    rcall EJECUTAR_SEGUNDOS

    ret



PAUSA_MEDIANA:

    ldi tiempo, 3

    rcall EJECUTAR_SEGUNDOS

    ret




ESTADO_CENTRIFUGADO:

    rcall APAGAR_LEDS


    ; LED3

    sbi PORTD, 6


    ; Motor

    sbi PORTB, 0


    cpi carga, 0

    breq CENT_CHICA


    cpi carga, 1

    breq CENT_MEDIANA


    ; Grande

    ldi tiempo, 24

    rcall EJECUTAR_SEGUNDOS

    rjmp FIN_CENTRIFUGADO



CENT_CHICA:

    ldi tiempo, 18

    rcall EJECUTAR_SEGUNDOS

    rjmp FIN_CENTRIFUGADO



CENT_MEDIANA:

    ldi tiempo, 21

    rcall EJECUTAR_SEGUNDOS



FIN_CENTRIFUGADO:

    cbi PORTB, 0

    ret





ESTADO_SECADO:

    rcall APAGAR_LEDS


    ; LED4

    sbi PORTD, 7


    

    sbi PORTB, 1


    rcall TIEMPO_SECADO_GIRO


    cbi PORTB, 1


   

    ldi tiempo, 3

    rcall EJECUTAR_SEGUNDOS


   
    sbi PORTB, 0


    rcall TIEMPO_SECADO_GIRO


    cbi PORTB, 0


    ret




TIEMPO_SECADO_GIRO:

    cpi carga, 0

    breq SEC_CHICA


    cpi carga, 1

    breq SEC_MEDIANA


    ldi tiempo, 11

    rcall EJECUTAR_SEGUNDOS

    ret



SEC_CHICA:

    ldi tiempo, 7

    rcall EJECUTAR_SEGUNDOS

    ret



SEC_MEDIANA:

    ldi tiempo, 9

    rcall EJECUTAR_SEGUNDOS

    ret




ESTADO_FIN:

    rcall APAGAR_LEDS


    ; LED10

    sbi PORTB, 5



ESPERAR_FIN:

    

    sbic PIND, 0

    rjmp ESPERAR_FIN


    rcall DELAY_REBOTE


    ; Confirmar

    sbis PIND, 0

    rjmp FIN_CONFIRMADO


    rjmp ESPERAR_FIN



FIN_CONFIRMADO:

    rcall ESPERAR_SOLTAR_SW1


    ret





MOSTRAR_CARGA:

    ; Apagar carga anterior

    cbi PORTB, 2

    cbi PORTB, 3

    cbi PORTB, 4


    cpi carga, 0

    breq CARGA_CHICA


    cpi carga, 1

    breq CARGA_MEDIANA


    cpi carga, 2

    breq CARGA_GRANDE


    ret



CARGA_CHICA:

    sbi PORTB, 2

    ret



CARGA_MEDIANA:

    sbi PORTB, 3

    ret



CARGA_GRANDE:

    sbi PORTB, 4

    ret





APAGAR_LEDS:

    

    ldi temp, 0

    out PORTD, temp


    ; D8-D13

    out PORTB, temp


    ret





EJECUTAR_SEGUNDOS:


BUCLE_SEGUNDOS:

    

    sbic PIND, 0

    rjmp CONTINUAR_DELAY


    rjmp REINICIAR



CONTINUAR_DELAY:

    rcall DELAY_1S


    dec tiempo


    brne BUCLE_SEGUNDOS


    ret




DELAY_1S:

    ldi r20, 64


DELAY_1:

    ldi r21, 250


DELAY_2:

    ldi r22, 250


DELAY_3:

    dec r22

    brne DELAY_3


    dec r21

    brne DELAY_2


    dec r20

    brne DELAY_1


    ret





DELAY_REBOTE:

    ldi r20, 10


REBOTE_1:

    ldi r21, 255


REBOTE_2:

    dec r21

    brne REBOTE_2


    dec r20

    brne REBOTE_1


    ret





REINICIAR:

   

    ldi temp, 0

    out EIMSK, temp


  

    out PORTD, temp

    out PORTB, temp


   
    ldi temp, high(RAMEND)

    out SPH, temp


    ldi temp, low(RAMEND)

    out SPL, temp


  

    ldi carga, 3


    

    ldi temp, (1<<INTF0) | (1<<INTF1)

    out EIFR, temp


    rjmp ESTADO_LISTO