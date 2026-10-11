"""Recepcion serie, registro y graficas en vivo de los problemas A y B."""

import argparse
from collections import deque
import csv
from datetime import datetime
import math
from pathlib import Path
import re
import time

PATRON_A = re.compile(
    r"A1,MS=(\d+),T=(-?\d+),PM=(\d+),LI=(-?\d+),LS=(-?\d+),"
    r"CAL=([01]),VENT=([0-3]),PWM=(\d+),OK=([01])"
)
PATRON_B = re.compile(
    r"SP=(\d+),LDR=(\d+),POS=(\d+),PWM=(\d+),DIR=(ABRIR|CERRAR|PARADO|TOPE)"
)


def interpretar_a(linea):
    """Cada trama A1 es independiente del menu y de las tramas anteriores."""
    resultado = PATRON_A.fullmatch(linea.strip())
    if resultado is None:
        return None
    ms, t, pm, inferior, superior, calor, nivel, pwm, ok = map(int, resultado.groups())
    if not (8 <= pm <= 25 and inferior == pm - 6 and superior == pm + 3):
        raise ValueError("Configuracion de A fuera del protocolo revisado")
    if pwm != nivel * 85 or (calor and nivel):
        raise ValueError("Estados de actuadores incompatibles")
    if ok and not 0 <= t <= 50:
        raise ValueError("Temperatura fuera del rango del DHT11")
    if not ok and (calor or nivel or pwm):
        raise ValueError("Salidas activas con sensor invalido")
    return dict(ms=ms, temperatura=t if ok else math.nan, setpoint=pm,
                inferior=inferior, superior=superior, calefactor=calor,
                ventilador=nivel, pwm=pwm, valido=ok)


def interpretar_b(linea):
    resultado = PATRON_B.fullmatch(linea.strip())
    if resultado is None:
        return None
    sp, ldr, pos, pwm = map(int, resultado.groups()[:4])
    direccion = resultado.group(5)
    if max(sp, ldr, pos) > 1023 or pwm > 255:
        raise ValueError("Valores fuera del ADC/PWM")
    if direccion in ("PARADO", "TOPE") and pwm:
        raise ValueError("PWM activo con motor detenido")
    firmado = pwm if direccion == "ABRIR" else -pwm if direccion == "CERRAR" else 0
    return dict(setpoint=sp, ldr=ldr, posicion=pos, pwm=pwm,
                direccion=direccion, pwm_firmado=firmado)


class LineasSerie:
    """Conserva las tramas partidas entre lecturas USB. No usa readline bloqueante."""
    def __init__(self):
        self.pendiente = b""

    def recibir(self, bloque):
        self.pendiente += bloque
        completas = self.pendiente.split(b"\n")
        self.pendiente = completas.pop()
        if len(self.pendiente) > 8192:
            self.pendiente = b""
            raise ValueError("Linea serie demasiado larga; se descarta")
        return [parte.decode("ascii", errors="replace").strip() for parte in completas]


def leer_disponible(conexion):
    """Consume el lote pendiente aunque una lectura entregue pocos bytes."""
    restante = min(conexion.in_waiting, 65536)
    partes = []
    while restante:
        parte = conexion.read(min(restante, 4096))
        if not parte:
            break
        partes.append(parte)
        restante -= len(parte)
    return b"".join(partes)


def crear_grafica(problema):
    import matplotlib.pyplot as plt
    figura, ejes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    lineas = {}
    if problema == "A":
        configuracion = [
            (0, "temperatura", "Temperatura", "#c45516", "-"),
            (0, "setpoint", "Punto medio", "red", "--"),
            (0, "inferior", "Limite ideal inferior", "#287d47", "--"),
            (0, "superior", "Limite ideal superior", "#167da0", "--"),
            (1, "calefactor", "Calefactor", "#ae3e35", "-"),
            (2, "ventilador", "Nivel solicitado", "#266da4", "-"),
        ]
        titulos = ["Temperatura y ambos limites del rango ideal",
                   "Orden del calefactor", "Orden del ventilador (PWM en D3)"]
        etiquetas = ["Temperatura (C)", "Calefactor", "Nivel"]
        ejes[0].set_ylim(-1, 51)
        ejes[1].set_yticks([0, 1], ["Apagado", "Encendido"])
        ejes[1].set_ylim(-0.2, 1.2)
        ejes[2].set_yticks([0, 1, 2, 3], ["Apagado", "Bajo", "Medio", "Alto"])
        ejes[2].set_ylim(-0.2, 3.2)
    else:
        configuracion = [
            (0, "setpoint", "Consigna de luz", "red", "--"),
            (0, "ldr", "Iluminacion LDR", "orange", "-"),
            (1, "posicion", "Posicion del potenciometro", "blue", "-"),
            (2, "pwm_firmado", "Orden del puente H", "green", "-"),
        ]
        titulos = ["Consigna e iluminacion", "Posicion: magnitud independiente",
                   "PWM con signo: cerrar (-) / abrir (+)"]
        etiquetas = ["Luz (ADC)", "Posicion (ADC)", "PWM"]
        ejes[0].set_ylim(0, 1023)
        ejes[1].set_ylim(0, 1023)
        ejes[2].set_ylim(-255, 255)
        ejes[2].axhline(0, color="black", linewidth=0.8)
    for eje, clave, etiqueta, color, estilo in configuracion:
        lineas[clave], = ejes[eje].plot([], [], color=color, linestyle=estilo,
                                      label=etiqueta, drawstyle="steps-post")
    for eje, titulo, etiqueta in zip(ejes, titulos, etiquetas):
        eje.set_title(titulo, fontsize=11)
        eje.set_ylabel(etiqueta)
        eje.set_xlim(0, 60)
        eje.grid(alpha=0.25)
    ejes[0].legend(loc="upper right", fontsize=8)
    ejes[-1].set_xlabel("Tiempo desde el inicio del registro (s)")
    figura.subplots_adjust(left=0.09, right=0.98, bottom=0.15, top=0.88, hspace=0.4)
    titulo = figura.suptitle(f"Problema {problema}: esperando telemetria", fontsize=11, y=0.97)
    return figura, ejes, lineas, titulo


def actualizar_grafica(ejes, lineas, titulo, datos, problema):
    tiempos = [dato["tiempo_s"] for dato in datos]
    for clave, linea in lineas.items():
        linea.set_data(tiempos, [dato[clave] for dato in datos])
    derecha = max(60, tiempos[-1])
    for eje in ejes:
        eje.set_xlim(max(0, derecha - 60), derecha)
    ultimo = datos[-1]
    if problema == "A":
        titulo.set_text(f"T={ultimo['temperatura']} C  PM={ultimo['setpoint']} C  "
                        f"Ideal={ultimo['inferior']}..{ultimo['superior']} C  "
                        f"CAL={ultimo['calefactor']} VENT={ultimo['ventilador']} "
                        f"PWM={ultimo['pwm']} OK={ultimo['valido']}")
    else:
        titulo.set_text(f"SP={ultimo['setpoint']} LDR={ultimo['ldr']} "
                        f"POS={ultimo['posicion']} PWM={ultimo['pwm']} "
                        f"DIR={ultimo['direccion']}")


def ejecutar(problema, puerto_predeterminado):
    import matplotlib.pyplot as plt
    from matplotlib.widgets import TextBox
    import serial

    opciones = argparse.ArgumentParser(description=f"Registro en vivo del Problema {problema}")
    opciones.add_argument("--puerto", default=puerto_predeterminado)
    opciones.add_argument("--setpoint", type=int, help="Enviar al iniciar; si se omite no se cambia")
    opciones.add_argument("--duracion", type=float, default=0, help="Segundos; 0 hasta cerrar")
    args = opciones.parse_args()
    minimo, maximo = (8, 25) if problema == "A" else (0, 1023)
    if args.duracion < 0 or (args.setpoint is not None and not minimo <= args.setpoint <= maximo):
        opciones.error(f"Duracion >=0; setpoint entre {minimo} y {maximo}")
    carpeta = Path(__file__).parent / "mediciones"
    carpeta.mkdir(exist_ok=True)
    base = carpeta / (problema + datetime.now().strftime("_%Y%m%d_%H%M%S_%f"))
    figura, ejes, lineas, titulo = crear_grafica(problema)
    entrada = TextBox(figura.add_axes([0.3, 0.03, 0.16, 0.04]), "Nuevo PM:" if problema == "A" else "Nueva luz:")
    # Solo la vista tiene limite; el CSV conserva TODAS las muestras recibidas.
    datos = deque(maxlen=2000)
    separador = LineasSerie()
    interpretar = interpretar_a if problema == "A" else interpretar_b
    conexion = None
    total = 0
    inicio = time.monotonic()
    siguiente_dibujo = 0

    def enviar(texto):
        if conexion is None or not conexion.is_open:
            return
        try:
            valor = int(texto)
            if not minimo <= valor <= maximo:
                raise ValueError
        except ValueError:
            print(f"Ingrese un entero de {minimo} a {maximo}")
            return
        conexion.write(f"{valor}\n".encode("ascii"))
        print(f"Solicitud enviada: {valor}. La grafica usara el valor confirmado por el micro.")

    entrada.on_submit(enviar)
    plt.show(block=False)
    try:
        with serial.Serial(args.puerto, 9600, timeout=0) as conexion, \
                base.with_suffix(".csv").open("x", newline="", encoding="utf-8") as csv_file, \
                Path(str(base) + "_mensajes.txt").open("x", encoding="utf-8") as mensajes:
            print(f"Conectado a {args.puerto}. Cierre Monitor Serie y Serial Plotter.")
            # Abrir USB puede reiniciar el Uno; esperar sin congelar la ventana.
            espera = time.monotonic() + 2
            while time.monotonic() < espera and plt.fignum_exists(figura.number):
                plt.pause(0.02)
            if not plt.fignum_exists(figura.number):
                return
            if args.setpoint is not None:
                enviar(str(args.setpoint))
            escritor = None
            inicio = time.monotonic()
            while plt.fignum_exists(figura.number) and (
                    args.duracion == 0 or time.monotonic() - inicio < args.duracion):
                ahora = time.monotonic()
                bloque = leer_disponible(conexion)
                if bloque:
                    for linea in separador.recibir(bloque):
                        if not linea:
                            continue
                        mensajes.write(f"{ahora - inicio:.3f}\t{linea}\n")
                        try:
                            dato = interpretar(linea)
                        except ValueError as error:
                            print("Trama rechazada:", error, linea)
                            continue
                        if dato is None:
                            print(linea)
                            continue
                        dato = dict(tiempo_s=round(ahora - inicio, 3), **dato)
                        if escritor is None:
                            escritor = csv.DictWriter(csv_file, fieldnames=list(dato))
                            escritor.writeheader()
                        escritor.writerow(dato)
                        datos.append(dato)
                        total += 1
                    csv_file.flush()
                    mensajes.flush()
                # Redibujar una sola vez tras consumir el bloque de muestras.
                if datos and ahora >= siguiente_dibujo:
                    actualizar_grafica(ejes, lineas, titulo, datos, problema)
                    figura.canvas.draw_idle()
                    siguiente_dibujo = ahora + 0.1
                plt.pause(0.01)
    except (serial.SerialException, KeyboardInterrupt, ValueError) as error:
        print("Registro terminado:", error)
    finally:
        if total:
            figura.savefig(base.with_suffix(".png"), dpi=160)
        plt.close(figura)
        print(f"{total} mediciones recibidas. Archivos en {carpeta}")

