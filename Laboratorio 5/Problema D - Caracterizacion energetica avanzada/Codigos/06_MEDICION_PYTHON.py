"""
LABORATORIO 5 - PROBLEMA D

Recibe muestras de un segundo Arduino, calcula corriente, dibuja una grafica
en vivo y guarda CSV, resumen y grafica.

Correcciones para comparaciones PRR:
  1. Permite medir y guardar un offset, o reutilizar exactamente el mismo.
  2. Usa el reloj monotonic de la PC para el eje temporal.
  3. Rechaza valores micros() malformados.
  4. Identifica cada captura con un nombre de configuracion.

Teclas dentro de la grafica:
    1 -> ACTIVO
    2 -> STANDBY
    3 -> APAGADO
    4 -> EMERGENCIA
    Q -> terminar y guardar
"""

from __future__ import annotations

import csv
import queue
import re
import statistics
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation
from serial.tools import list_ports


plt.rcParams["keymap.save"] = []
plt.rcParams["keymap.pan"] = []


# ----------------------- DATOS DEL MONTAJE -----------------------

BAUDIOS = 115200
VENTANA_GRAFICA_S = 60.0
VREF_ADC_V = 5.000
FACTOR_DIVISOR = 2.0
R_SHUNT_OHM = 5.0
GANANCIA = 11.0
TENSION_ALIMENTACION_V = 9.0
CALIBRACION_S = 3.0

CARPETA_BASE = Path(__file__).resolve().parent
ARCHIVO_OFFSET = CARPETA_BASE / "offset_lm358.txt"


def elegir_puerto() -> str:
    """Muestra los puertos disponibles y permite elegir uno."""
    puertos = list(list_ports.comports())

    if not puertos:
        raise RuntimeError("No se encontro ningun puerto serie.")

    print("\nPuertos disponibles:")
    for numero, puerto in enumerate(puertos, start=1):
        print(f"  {numero}. {puerto.device} - {puerto.description}")

    if len(puertos) == 1:
        print(f"\nSe usara {puertos[0].device}.")
        return puertos[0].device

    while True:
        try:
            opcion = int(input("Elegir numero de puerto: "))
            if 1 <= opcion <= len(puertos):
                return puertos[opcion - 1].device
        except ValueError:
            pass
        print("Opcion invalida.")


def pedir_configuracion() -> str:
    """Pide una etiqueta que se guardara en archivos y CSV."""
    texto = input(
        "\nNombre de la configuracion "
        "(ejemplo TIMER2_0U o TIMER2_1U): "
    ).strip().upper()

    if not texto:
        texto = "SIN_NOMBRE"

    # Mantiene solo caracteres seguros para nombres de archivo.
    texto = re.sub(r"[^A-Z0-9_-]+", "_", texto)
    return texto[:50]


def leer_linea(ser: serial.Serial) -> tuple[int, int] | None:
    """Lee una linea con formato micros,adc y descarta datos malformados."""
    try:
        texto = ser.readline().decode("ascii", errors="ignore").strip()
        if not texto:
            return None

        partes = texto.split(",")
        if len(partes) != 2:
            return None

        micros_arduino = int(partes[0])
        adc = int(partes[1])

        if not 0 <= micros_arduino <= 0xFFFFFFFF:
            return None
        if not 0 <= adc <= 1023:
            return None

        return micros_arduino, adc
    except (ValueError, UnicodeError):
        return None


def calibrar_offset(ser: serial.Serial) -> float:
    """Calcula el offset del amplificador con el shunt puenteado."""
    print("\nCALIBRACION DEL OFFSET")
    print("1. Mantener el sistema encendido.")
    print("2. Puentear momentaneamente las dos patas del shunt.")
    input("3. Presionar ENTER cuando el puente este colocado...")

    ser.reset_input_buffer()
    muestras_v: list[float] = []
    fin = time.monotonic() + CALIBRACION_S

    while time.monotonic() < fin:
        dato = leer_linea(ser)
        if dato is None:
            continue

        _, adc = dato
        voltaje_a0 = adc * VREF_ADC_V / 1023.0
        muestras_v.append(voltaje_a0 * FACTOR_DIVISOR)

    if len(muestras_v) < 20:
        raise RuntimeError("No se recibieron suficientes muestras para calibrar.")

    offset = statistics.fmean(muestras_v)
    ruido = statistics.pstdev(muestras_v)

    print(f"Offset medido: {offset:.6f} V")
    print(f"Desviacion durante calibracion: {ruido:.6f} V")
    input("Retirar el puente del shunt y presionar ENTER para comenzar...")
    ser.reset_input_buffer()

    return offset


def guardar_offset(offset: float) -> None:
    """Guarda el offset para reutilizarlo en la configuracion siguiente."""
    ARCHIVO_OFFSET.write_text(f"{offset:.9f}\n", encoding="utf-8")
    print(f"Offset guardado en: {ARCHIVO_OFFSET}")


def leer_offset_guardado() -> float | None:
    """Devuelve el offset guardado, si existe y es valido."""
    if not ARCHIVO_OFFSET.exists():
        return None

    try:
        offset = float(ARCHIVO_OFFSET.read_text(encoding="utf-8").strip())
        if 0.0 <= offset <= VREF_ADC_V * FACTOR_DIVISOR:
            return offset
    except (OSError, ValueError):
        pass

    return None


def obtener_offset(ser: serial.Serial) -> tuple[float, str]:
    """Permite calibrar una vez o reutilizar el mismo offset."""
    guardado = leer_offset_guardado()

    if guardado is None:
        print("\nNo existe un offset guardado. Se realizara una calibracion nueva.")
        offset = calibrar_offset(ser)
        guardar_offset(offset)
        return offset, "MEDIDO"

    print(f"\nOffset guardado disponible: {guardado:.6f} V")
    while True:
        opcion = input(
            "Escribir M para MEDIR uno nuevo o R para REUTILIZAR el guardado: "
        ).strip().upper()

        if opcion == "R":
            print(f"Se reutilizara el offset: {guardado:.6f} V")
            ser.reset_input_buffer()
            return guardado, "REUTILIZADO"

        if opcion == "M":
            offset = calibrar_offset(ser)
            guardar_offset(offset)
            return offset, "MEDIDO"

        print("Opcion invalida. Elegir M o R.")


def crear_archivos(configuracion: str) -> tuple[Path, Path, Path]:
    marca = datetime.now().strftime("%Y%m%d_%H%M%S")
    carpeta = CARPETA_BASE / "mediciones"
    carpeta.mkdir(parents=True, exist_ok=True)

    base = f"{configuracion}_{marca}"
    return (
        carpeta / f"medicion_{base}.csv",
        carpeta / f"resumen_{base}.csv",
        carpeta / f"grafica_{base}.png",
    )


def calcular_resumen(
    tiempos: list[float],
    corrientes: list[float],
    estados: list[str],
    configuracion: str,
    offset_v: float,
    origen_offset: str,
) -> list[dict[str, float | int | str]]:
    """Calcula estadisticas y carga integrada para cada estado."""
    nombres = ["TOTAL", "ACTIVO", "STANDBY", "APAGADO", "EMERGENCIA"]
    resumen: list[dict[str, float | int | str]] = []

    for nombre in nombres:
        if nombre == "TOTAL":
            indices = list(range(len(tiempos)))
        else:
            indices = [i for i, estado in enumerate(estados) if estado == nombre]

        if not indices:
            continue

        valores = [corrientes[i] for i in indices]
        duracion = 0.0
        carga_mAh = 0.0

        for i in range(1, len(tiempos)):
            mismo_estado = (
                nombre == "TOTAL"
                or (estados[i - 1] == nombre and estados[i] == nombre)
            )
            if not mismo_estado:
                continue

            dt_s = tiempos[i] - tiempos[i - 1]
            if dt_s <= 0.0 or dt_s > 1.0:
                continue

            corriente_media_intervalo = (
                corrientes[i - 1] + corrientes[i]
            ) / 2.0
            carga_mAh += corriente_media_intervalo * dt_s / 3600.0
            duracion += dt_s

        corriente_media = statistics.fmean(valores)

        resumen.append(
            {
                "configuracion": configuracion,
                "offset_lm358_V": offset_v,
                "origen_offset": origen_offset,
                "estado": nombre,
                "muestras": len(valores),
                "duracion_s": duracion,
                "corriente_min_mA": min(valores),
                "corriente_max_mA": max(valores),
                "corriente_media_mA": corriente_media,
                "mediana_mA": statistics.median(valores),
                "desviacion_mA": statistics.pstdev(valores),
                "carga_mAh": carga_mAh,
                "potencia_media_mW": TENSION_ALIMENTACION_V * corriente_media,
                "energia_mWh": TENSION_ALIMENTACION_V * carga_mAh,
            }
        )

    return resumen


def main() -> None:
    configuracion = pedir_configuracion()
    puerto = elegir_puerto()
    print(f"\nAbriendo {puerto} a {BAUDIOS} baudios...")

    ser = serial.Serial(puerto, BAUDIOS, timeout=0.2)
    time.sleep(2.0)
    ser.reset_input_buffer()

    offset_v, origen_offset = obtener_offset(ser)
    archivo_csv, archivo_resumen, archivo_png = crear_archivos(configuracion)

    print(f"\nConfiguracion: {configuracion}")
    print(f"Offset: {offset_v:.6f} V ({origen_offset})")
    print(f"CSV: {archivo_csv}")
    print("Teclas: 1=ACTIVO, 2=STANDBY, 3=APAGADO, 4=EMERGENCIA, Q=salir")

    cola: queue.Queue[tuple[float, int, int]] = queue.Queue()
    ejecutando = threading.Event()
    ejecutando.set()

    def lector_serial() -> None:
        while ejecutando.is_set():
            dato = leer_linea(ser)
            if dato is not None:
                micros_arduino, adc = dato
                # Se registra el instante al recibir la linea, antes de encolarla.
                cola.put((time.monotonic(), micros_arduino, adc))

    hilo = threading.Thread(target=lector_serial, daemon=True)
    hilo.start()

    todos_tiempos: list[float] = []
    todos_adc: list[int] = []
    todos_vout: list[float] = []
    todos_vshunt: list[float] = []
    todas_corrientes: list[float] = []
    todos_estados: list[str] = []

    graf_t: deque[float] = deque()
    graf_i: deque[float] = deque()

    estado_actual = "ACTIVO"
    tiempo_host_inicial: float | None = None
    finalizado = False

    csv_f = archivo_csv.open("w", newline="", encoding="utf-8")
    escritor = csv.writer(csv_f)
    escritor.writerow(
        [
            "configuracion",
            "offset_lm358_V",
            "origen_offset",
            "tiempo_s",
            "micros_arduino",
            "adc_raw",
            "vout_amplificador_V",
            "vshunt_V",
            "corriente_mA",
            "estado",
        ]
    )

    fig, ax = plt.subplots(figsize=(12, 6))
    fig.canvas.manager.set_window_title(
        f"LAB 5 - Problema D - {configuracion}"
    )

    linea_cruda, = ax.plot(
        [], [], color="0.75", linewidth=0.7, label="Corriente"
    )
    linea_suave, = ax.plot(
        [], [], color="#0066CC", linewidth=2.0, label="Promedio movil"
    )
    texto_info = ax.text(
        0.02,
        0.96,
        "",
        transform=ax.transAxes,
        va="top",
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "0.75"},
    )

    ax.set_title(f"Corriente del sistema - {configuracion}")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Corriente (mA)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    def marcar_estado(nuevo_estado: str) -> None:
        nonlocal estado_actual
        estado_actual = nuevo_estado
        tiempo_actual = todos_tiempos[-1] if todos_tiempos else 0.0
        ax.axvline(tiempo_actual, color="black", linestyle="--", alpha=0.45)
        ax.text(
            tiempo_actual,
            ax.get_ylim()[1],
            nuevo_estado,
            rotation=90,
            va="top",
            ha="right",
            fontsize=8,
        )
        print(f"Estado marcado: {nuevo_estado} en t={tiempo_actual:.3f} s")

    def al_presionar_tecla(event) -> None:
        tecla = (event.key or "").lower()
        mapa = {
            "1": "ACTIVO",
            "2": "STANDBY",
            "3": "APAGADO",
            "4": "EMERGENCIA",
        }

        if tecla in mapa:
            marcar_estado(mapa[tecla])
        elif tecla == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", al_presionar_tecla)

    def procesar_muestra(
        instante_host: float,
        micros_arduino: int,
        adc: int,
    ) -> None:
        nonlocal tiempo_host_inicial

        if tiempo_host_inicial is None:
            tiempo_host_inicial = instante_host

        tiempo_s = instante_host - tiempo_host_inicial
        voltaje_a0 = adc * VREF_ADC_V / 1023.0
        vout = voltaje_a0 * FACTOR_DIVISOR
        vshunt = (vout - offset_v) / GANANCIA
        corriente_mA = vshunt / R_SHUNT_OHM * 1000.0

        todos_tiempos.append(tiempo_s)
        todos_adc.append(adc)
        todos_vout.append(vout)
        todos_vshunt.append(vshunt)
        todas_corrientes.append(corriente_mA)
        todos_estados.append(estado_actual)

        escritor.writerow(
            [
                configuracion,
                f"{offset_v:.9f}",
                origen_offset,
                f"{tiempo_s:.6f}",
                micros_arduino,
                adc,
                f"{vout:.6f}",
                f"{vshunt:.9f}",
                f"{corriente_mA:.6f}",
                estado_actual,
            ]
        )

        graf_t.append(tiempo_s)
        graf_i.append(corriente_mA)

        while graf_t and tiempo_s - graf_t[0] > VENTANA_GRAFICA_S:
            graf_t.popleft()
            graf_i.popleft()

    def actualizar(_):
        cantidad = 0
        while cantidad < 1000:
            try:
                instante_host, micros_arduino, adc = cola.get_nowait()
            except queue.Empty:
                break

            procesar_muestra(instante_host, micros_arduino, adc)
            cantidad += 1

        if not graf_t:
            return linea_cruda, linea_suave, texto_info

        t = np.asarray(graf_t, dtype=float)
        corriente = np.asarray(graf_i, dtype=float)

        puntos = min(50, len(corriente))
        if puntos >= 3:
            kernel = np.ones(puntos) / puntos
            suave = np.convolve(corriente, kernel, mode="valid")
            t_suave = t[puntos - 1 :]
        else:
            suave = corriente
            t_suave = t

        linea_cruda.set_data(t, corriente)
        linea_suave.set_data(t_suave, suave)

        izquierda = max(0.0, t[-1] - VENTANA_GRAFICA_S)
        derecha = max(VENTANA_GRAFICA_S, t[-1] + 1.0)
        ax.set_xlim(izquierda, derecha)

        minimo = float(np.min(corriente))
        maximo = float(np.max(corriente))
        margen = max(2.0, (maximo - minimo) * 0.15)
        ax.set_ylim(min(minimo - margen, 0.0), maximo + margen)

        media_corta = float(np.mean(corriente[-min(200, len(corriente)) :]))
        voltaje_a0_actual = todos_adc[-1] * VREF_ADC_V / 1023.0
        aviso = ""
        if voltaje_a0_actual >= 4.75:
            aviso = "\nADVERTENCIA: A0 cerca de 5 V"

        texto_info.set_text(
            f"Configuracion: {configuracion}\n"
            f"Estado: {estado_actual}\n"
            f"Corriente: {corriente[-1]:.2f} mA\n"
            f"Media 1 s: {media_corta:.2f} mA\n"
            f"A0: {voltaje_a0_actual:.3f} V\n"
            f"Offset LM358: {offset_v:.4f} V ({origen_offset})"
            f"{aviso}"
        )

        if len(todos_tiempos) % 200 == 0:
            csv_f.flush()

        return linea_cruda, linea_suave, texto_info

    animacion = FuncAnimation(
        fig,
        actualizar,
        interval=100,
        blit=False,
        cache_frame_data=False,
    )
    _ = animacion

    def finalizar() -> None:
        nonlocal finalizado
        if finalizado:
            return
        finalizado = True

        ejecutando.clear()
        hilo.join(timeout=1.0)
        ser.close()
        csv_f.flush()
        csv_f.close()

        if not todos_tiempos:
            print("No se guardaron muestras.")
            return

        resumen = calcular_resumen(
            todos_tiempos,
            todas_corrientes,
            todos_estados,
            configuracion,
            offset_v,
            origen_offset,
        )

        with archivo_resumen.open("w", newline="", encoding="utf-8") as f:
            campos = list(resumen[0].keys())
            writer = csv.DictWriter(f, fieldnames=campos)
            writer.writeheader()
            writer.writerows(resumen)

        fig.savefig(archivo_png, dpi=180, bbox_inches="tight")

        print("\nArchivos guardados:")
        print(f"  Datos:   {archivo_csv}")
        print(f"  Resumen: {archivo_resumen}")
        print(f"  Grafica: {archivo_png}")

        print("\nResumen:")
        for fila in resumen:
            print(
                f"  {fila['estado']:<10} "
                f"media={fila['corriente_media_mA']:.2f} mA, "
                f"mediana={fila['mediana_mA']:.2f} mA, "
                f"min={fila['corriente_min_mA']:.2f} mA, "
                f"max={fila['corriente_max_mA']:.2f} mA"
            )

    fig.canvas.mpl_connect("close_event", lambda _event: finalizar())

    try:
        plt.tight_layout()
        plt.show()
    finally:
        finalizar()


if __name__ == "__main__":
    main()
