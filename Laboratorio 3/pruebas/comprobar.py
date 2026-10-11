"""Pruebas de SOFTWARE con datos artificiales, no ensayos fisicos.
Ejecuta funciones actuales del firmware en el simulador de instrucciones
de avr-gdb. Sustituye exclusivamente las interfaces UART/LCD/EEPROM.
Requiere AVR-GCC y avr-gdb (Studio); pySerial/Matplotlib para la grafica.
"""
import argparse
import importlib.util
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE / 'Python Graficas Código'))
from graficas_comunes import interpretar_a, interpretar_b, LineasSerie


def reemplazar_funcion(texto, nombre, cuerpo):
    match = re.search(r'^(?:static )?[\w *]+\b' + nombre + r'\s*\([^;]*?\)\s*\{', texto, re.M)
    if not match: raise ValueError(nombre)
    inicio = match.end()
    fin = inicio
    profundidad = 1
    while profundidad:
        if texto[fin] == '{': profundidad += 1
        elif texto[fin] == '}': profundidad -= 1
        fin += 1
    return texto[:inicio] + cuerpo + texto[fin-1:]


PREAMBULO = '''
#include <string.h>
volatile unsigned int comprobaciones;
volatile unsigned int fallos;
char trama[200];
static void comprobar(unsigned char bien) { comprobaciones++; if (!bien) fallos++; }
void fin_pruebas(void) __attribute__((noinline));
void fin_pruebas(void) { __asm__ volatile("nop"); }
'''

PRUEBAS_A = '''
static void comando_a(const char *entrada) {
    while (*entrada) { UCSR0A=_BV(RXC0); UDR0=*entrada++; atender_menu(); }
}
int main(void) {
    const int16_t temperaturas[] = {0,15,16,25,26,35,36,45,46,50};
    const uint8_t esperados[] = {CALOR,CALOR,REPOSO,REPOSO,BAJO,BAJO,MEDIO,MEDIO,ALTO,ALTO};
    punto_medio=22;
    for (uint8_t i=0;i<10;i++) {
        estado_control=SIN_LECTURA; temperatura=temperaturas[i]; trama[0]=0;
        controlar_temperatura(); comprobar(estado_control==esperados[i]);
        comprobar(!(nivel_ventilador && (PORTB & _BV(CALEFACTOR_PIN))));
        comprobar(pwm_ventilador==85*nivel_ventilador);
    }
    for(int16_t pm=PUNTO_MINIMO; pm<=PUNTO_MAXIMO; pm++) {
        punto_medio=pm;
        uint8_t encontrados=0;
        estado_control=SIN_LECTURA;
        for(temperatura=0;temperatura<=50;temperatura++) {
            trama[0]=0; controlar_temperatura(); encontrados |= 1<<estado_control;
            comprobar(!(nivel_ventilador && (PORTB & _BV(CALEFACTOR_PIN))));
        }
        comprobar(encontrados==31); /* Los cinco estados en rampa ascendente. */
        encontrados=0;
        for(temperatura=50;temperatura>=0;temperatura--) {
            trama[0]=0; controlar_temperatura(); encontrados |= 1<<estado_control;
        }
        comprobar(encontrados==31);
    }
    punto_medio=22; estado_control=REPOSO;
    for(uint8_t i=0;i<20;i++) { temperatura=i%2?25:26; trama[0]=0; controlar_temperatura(); comprobar(estado_control==REPOSO); }
    temperatura=27; trama[0]=0; controlar_temperatura(); comprobar(estado_control==BAJO);
    temperatura=25; trama[0]=0; controlar_temperatura(); comprobar(estado_control==BAJO);
    temperatura=24; trama[0]=0; controlar_temperatura(); comprobar(estado_control==REPOSO);
    temperatura=50; trama[0]=0; controlar_temperatura(); comprobar(estado_control==ALTO);
    temperatura=-1; trama[0]=0; controlar_temperatura(); comprobar(estado_control==SIN_LECTURA && pwm_ventilador==0 && !(PORTB&_BV(CALEFACTOR_PIN)));
    temperatura=51; trama[0]=0; controlar_temperatura(); comprobar(estado_control==SIN_LECTURA && pwm_ventilador==0);
    temperatura=24; trama[0]=0;
    comando_a("7\\n"); comprobar(punto_medio==22);
    comando_a("26\\n"); comprobar(punto_medio==22);
    comando_a("-10\\n"); comprobar(punto_medio==22);
    comando_a("123\\n"); comprobar(punto_medio==22);
    comando_a("8\\r\\n"); comprobar(punto_medio==8);
    comando_a("25\\n"); comprobar(punto_medio==25);
    comando_a("r"); comprobar(punto_medio==22);
    temperatura=24; reloj_segundos=5; estado_control=SIN_LECTURA; trama[0]=0;
    controlar_temperatura();
    comprobar(strcmp(trama,"A1,MS=5000,T=24,PM=22,LI=16,LS=25,CAL=0,VENT=0,PWM=0,OK=1\\r\\n")==0);
    fin_pruebas(); for(;;) {}
}
'''

PRUEBAS_B = '''
static void mover_dos_veces(void) { controlar_luz(); reloj_ms+=50; controlar_luz(); }
int main(void) {
    setpoint=512; lectura_ldr=512; posicion_persiana=900;
    mover_dos_veces(); comprobar(pwm_aplicado==0);
    lectura_ldr=480; mover_dos_veces(); comprobar(strcmp(sentido_actual,"ABRIR")==0);
    lectura_ldr=510; mover_dos_veces(); comprobar(pwm_aplicado==0);
    lectura_ldr=490; mover_dos_veces(); comprobar(pwm_aplicado==0); /* |e|22: retiene reposo. */
    lectura_ldr=480; mover_dos_veces(); comprobar(pwm_aplicado>=PWM_MINIMO && pwm_aplicado<=PWM_MAXIMO);
    lectura_ldr=700; posicion_persiana=500; mover_dos_veces(); comprobar(strcmp(sentido_actual,"CERRAR")==0);
    posicion_persiana=POSICION_MINIMA; mover_dos_veces(); comprobar(strcmp(sentido_actual,"TOPE")==0 && pwm_aplicado==0);
    lectura_ldr=300; mover_dos_veces(); comprobar(strcmp(sentido_actual,"ABRIR")==0); /* Puede alejarse del tope. */
    posicion_persiana=POSICION_MAXIMA; mover_dos_veces(); comprobar(strcmp(sentido_actual,"TOPE")==0 && pwm_aplicado==0);
    lectura_ldr=700; mover_dos_veces(); comprobar(strcmp(sentido_actual,"CERRAR")==0);
    posicion_persiana=500;
    for(setpoint=0;setpoint<=1023;setpoint+=31) {
        lectura_ldr=1023; mover_dos_veces(); comprobar(pwm_aplicado<=PWM_MAXIMO);
        lectura_ldr=0; mover_dos_veces(); comprobar(pwm_aplicado<=PWM_MAXIMO);
    }
    setpoint=512; uart_procesar_caracter('1'); uart_procesar_caracter('0'); uart_procesar_caracter('2'); uart_procesar_caracter('3'); uart_procesar_caracter('\\n'); comprobar(setpoint==1023);
    uart_procesar_caracter('9'); uart_procesar_caracter('9'); uart_procesar_caracter('9'); uart_procesar_caracter('9'); uart_procesar_caracter('9'); uart_procesar_caracter('\\n'); comprobar(setpoint==1023);
    uart_procesar_caracter('-'); uart_procesar_caracter('1'); uart_procesar_caracter('\\n'); comprobar(setpoint==1023);
    uart_procesar_caracter('0'); uart_procesar_caracter('\\r'); uart_procesar_caracter('\\n'); comprobar(setpoint==0);
    setpoint=512; lectura_ldr=480; posicion_persiana=620; pwm_aplicado=124; sentido_actual="ABRIR"; trama[0]=0;
    enviar_diagnostico();
    comprobar(strcmp(trama,"SP=512,LDR=480,POS=620,PWM=124,DIR=ABRIR\\r\\n")==0);
    fin_pruebas(); for(;;) {}
}
'''

PRUEBAS_C = '''
int main(void) {
    /* Referencias ARTIFICIALES para ejercitar la resta y la tolerancia. */
    for(uint8_t patron=0;patron<4;patron++)
        for(uint8_t canal=0;canal<3;canal++) calibracion.referencia[patron][canal]=100+patron*200;
    uint8_t cercano=0; uint16_t valor[3]={100,100,100};
    comprobar(clasificar(valor,&cercano)==0);
    valor[0]=51; comprobar(clasificar(valor,&cercano)==0);
    valor[0]=50; comprobar(clasificar(valor,&cercano)==SIN_MUESTRA);
    valor[0]=149; comprobar(clasificar(valor,&cercano)==0);
    valor[0]=150; comprobar(clasificar(valor,&cercano)==SIN_MUESTRA);
    for(uint8_t canal=0;canal<3;canal++) valor[canal]=700;
    comprobar(clasificar(valor,&cercano)==3 && cercano==3);
    resultado=3; mas_cercano=3; angulo_actual=180;
    for(uint8_t canal=0;canal<3;canal++) { lectura[canal]=0; calibracion.referencia[3][canal]=1023; }
    pagina_resultado=0; mostrar_resultado(); comprobar(strcmp(lcd1,"AMARILLO")==0 && strcmp(lcd2,"Angulo: 180")==0);
    for(pagina_resultado=1;pagina_resultado<4;pagina_resultado++) {
        mostrar_resultado(); comprobar(strlen(lcd1)<=16 && strlen(lcd2)<=16);
        comprobar(strstr(lcd1,"L:0 P:1023")!=0 && strcmp(lcd2,"D:-1023 AMARILLO")==0);
    }
    pagina_resultado=0; modo=2; resultado_disponible=true; siguiente_pagina=2000;
    reloj_ms=1999; actualizar_paginas(); comprobar(pagina_resultado==0);
    reloj_ms=2000; actualizar_paginas(); comprobar(pagina_resultado==1);
    modo=0; reloj_ms=4000; actualizar_paginas(); comprobar(pagina_resultado==1);
    fin_pruebas(); for(;;) {}
}
'''


def comprobar_avr(toolchain, carpeta):
    nombres = {'A':'Problema A. Calefactor_automatizado.c',
               'B':'Problema B. Control de iluminación con setpoint.c',
               'C':'Problema C. Sistema Clasificador de Color'}
    for problema, pruebas in [('A',PRUEBAS_A), ('B',PRUEBAS_B), ('C',PRUEBAS_C)]:
        fuente = (BASE/nombres[problema]).read_text(encoding='utf-8')
        fuente = fuente.replace('int main(void)', 'int firmware_main(void)')
        if problema == 'A':
            fuente = reemplazar_funcion(fuente,'serie_texto','strncat(trama,texto,sizeof(trama)-strlen(trama)-1);')
        elif problema == 'B':
            fuente = reemplazar_funcion(fuente,'uart_enviar_caracter','size_t n=strlen(trama); if(n<sizeof(trama)-1) {trama[n]=caracter; trama[n+1]=0;}')
            fuente = reemplazar_funcion(fuente,'setpoint_guardar_eeprom','/* EEPROM no se simula en estas pruebas de logica. */')
        else:
            fuente = reemplazar_funcion(fuente,'pantalla','strncpy(lcd1,linea1,16); lcd1[16]=0; strncpy(lcd2,linea2,16); lcd2[16]=0;')
            fuente = 'char lcd1[17],lcd2[17];\n' + fuente
        archivo = carpeta/f'prueba_{problema}.c'
        archivo.write_text(PREAMBULO+fuente+'\n'+pruebas,encoding='utf-8')
        elf = archivo.with_suffix('.elf')
        subprocess.run([str(toolchain/'avr-gcc.exe'),'-mmcu=atmega328p','-std=gnu99',
                        '-Os','-g','-ffunction-sections','-fdata-sections','-Wl,--gc-sections',
                        str(archivo),'-o',str(elf)],check=True)
        # El simulador de instrucciones no emula el arranque Flash->SRAM.
        # Restaurar la .data inicial del ELF, sin alterar la logica probada.
        iniciales = archivo.with_suffix('.data')
        subprocess.run([str(toolchain/'avr-objcopy.exe'),'-O','binary','-j','.data',
                        str(elf),str(iniciales)],check=True)
        resultado = subprocess.run([str(toolchain/'avr-gdb.exe'),'--batch',str(elf),
                '-ex','target sim','-ex','load','-ex','break main','-ex','run',
                '-ex',f'restore {iniciales.name} binary 0x800100',
                '-ex','break fin_pruebas','-ex','continue',
                '-ex','print /d comprobaciones','-ex','print /d fallos',
                '-ex','print trama'],cwd=carpeta,capture_output=True,text=True,timeout=40,check=True)
        valores = re.findall(r'\$\d+ = (\d+)',resultado.stdout)
        if len(valores)<2 or int(valores[1]):
            raise AssertionError(resultado.stdout+resultado.stderr)
        print(f'{problema}: {valores[0]} comprobaciones de software, cero fallos')
        # Verifica en Python la trama generada por el C ejecutado en AVR.
        trama = re.search(r'\$3 = "(.*?)"',resultado.stdout)
        if problema in ('A','B'):
            texto = bytes(trama[1],'ascii').decode('unicode_escape').split('\0',1)[0].strip()
            dato = (interpretar_a if problema=='A' else interpretar_b)(texto)
            assert dato is not None
            print(f'{problema}: trama C interpretada correctamente en Python')


def comprobar_python(carpeta):
    from graficas_comunes import leer_disponible
    class PuertoFragmentado:
        def __init__(self, texto): self.datos = texto
        @property
        def in_waiting(self): return len(self.datos)
        def read(self, cantidad):
            cantidad = min(cantidad, 3)
            parte, self.datos = self.datos[:cantidad], self.datos[cantidad:]
            return parte
    tramas = b''.join(
        f'SP=512,LDR={i*100},POS=620,PWM=124,DIR=CERRAR\r\n'.encode('ascii')
        for i in range(11))
    puerto = PuertoFragmentado(tramas)
    completas = LineasSerie().recibir(leer_disponible(puerto))
    assert [interpretar_b(linea)['ldr'] for linea in completas] == list(range(0,1001,100))
    assert puerto.in_waiting == 0
    assert leer_disponible(puerto) == b''
    linea = b'A1,MS=5000,T=24,PM=22,LI=16,LS=25,CAL=0,VENT=0,PWM=0,OK=1\r\n'
    for corte in range(len(linea)+1):
        separador = LineasSerie()
        completas = separador.recibir(linea[:corte])+separador.recibir(linea[corte:])
        assert len(completas)==1 and interpretar_a(completas[0])['temperatura']==24
    assert interpretar_a('Temperatura: 24 C | sin setpoint') is None
    assert interpretar_b('SP=512,LDR=1000,POS=620,PWM=124,DIR=CERRAR')['pwm_firmado']==-124
    assert interpretar_b('SP=512,LDR=0,POS=620,PWM=124,DIR=ABRIR')['ldr']==0
    for linea_mala in ['SP=512,LDR=1200,POS=620,PWM=124,DIR=ABRIR',
                       'SP=512,LDR=500,POS=620,PWM=124,DIR=PARADO']:
        try: interpretar_b(linea_mala)
        except ValueError: pass
        else: raise AssertionError('Se acepto una trama invalida')
    os.environ['MPLBACKEND']='Agg'
    from graficas_comunes import crear_grafica, actualizar_grafica
    import matplotlib.pyplot as plt
    for problema in ('A','B'):
        figura,ejes,lineas,titulo=crear_grafica(problema)
        datos=[]
        for i in range(8):
            if problema=='A':
                temperatura=[10,17,27,37,47,50,24,-1][i]
                nivel=[0,0,1,2,3,3,0,0][i]
                dato=interpretar_a(f'A1,MS={i*5000},T={temperatura},PM=22,LI=16,LS=25,CAL={int(i==0)},VENT={nivel},PWM={85*nivel},OK={int(i!=7)}')
            else:
                dato=interpretar_b(f'SP=512,LDR={500 if i<4 else 1000},POS=620,PWM=124,DIR={"ABRIR" if i<4 else "CERRAR"}')
            datos.append(dict(tiempo_s=i*5,**dato))
        actualizar_grafica(ejes,lineas,titulo,datos,problema)
        if problema=='A':
            assert list(lineas['inferior'].get_ydata())==[16]*8
            assert list(lineas['superior'].get_ydata())==[25]*8
            assert list(lineas['ventilador'].get_ydata())==[0,0,1,2,3,3,0,0]
            assert lineas['calefactor'].get_ydata()[0]==1
        else:
            assert lineas['ldr'].get_ydata()[-1]==1000
            assert lineas['pwm_firmado'].get_ydata()[-1]==-124
        titulo.set_text('PRUEBA DE SOFTWARE: DATOS ARTIFICIALES, NO MEDICIONES')
        figura.savefig(carpeta/f'prueba_grafica_{problema}.png',dpi=130)
        plt.close(figura)
    print('Python: tramas fragmentadas, extremos LDR, signo PWM y ambos limites: correcto')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--toolchain',type=Path,default=Path(r'C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin'))
    parser.add_argument('--salida',type=Path,default=Path(tempfile.gettempdir())/'Lab3_pruebas_software')
    args=parser.parse_args()
    args.salida.mkdir(parents=True,exist_ok=True)
    comprobar_avr(args.toolchain,args.salida)
    comprobar_python(args.salida)
    print('No se probo ningun periferico ni montaje fisico en estas comprobaciones.')
