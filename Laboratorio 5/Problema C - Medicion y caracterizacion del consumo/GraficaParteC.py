"""
LABORATORIO 5 - PROBLEMA C

Recibe muestras de un segundo Arduino, calcula corriente, dibuja una grafica
en vivo y guarda los resultados para el informe.

Teclas dentro de la ventana de la grafica:
    1 -> ACTIVO
    2 -> STANDBY
    3 -> APAGADO
    4 -> EMERGENCIA
    Q -> terminar y guardar
"""

from __future__ import annotations

import csv
import queue
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

# Se anulan atajos de Matplotlib que interfieren con el marcado de estados.
plt.rcParams["keymap.save"] = []
plt.rcParams["keymap.pan"] = []


# ----------------------- DATOS DEL MONTAJE -----------------------

BAUDIOS = 115200
VENTANA_GRAFICA_S = 60.0

# Medir con multimetro el pin 5 V del SEGUNDO Arduino y corregir este valor.
VREF_ADC_V = 5.000

# Divisor 10k/10k entre la salida del LM358 y A0.
# A0 recibe la mitad de la salida, por eso Python la multiplica por 2.
FACTOR_DIVISOR = 2.0

R_SHUNT_OHM = 5.0

# Ganancia medida/configurada en el amplificador no inversor.
# Por ejemplo: G = 1 + 22k/2.2k = 11.
GANANCIA = 11.0

# Tension que alimenta el sistema medido por VIN.
TENSION_ALIMENTACION_V = 9.0

# Duracion de la calibracion inicial del offset.
CALIBRACION_S = 3.0


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


def leer_linea(ser: serial.Serial) -> tuple[int, int] | None:
    """Lee una linea con formato micros,adc."""
    try:
        texto = ser.readline().decode("ascii", errors="ignore").strip()
        if not texto:
            return None

        tiempo_us, adc = texto.split(",", maxsplit=1)
        adc_entero = int(adc)

        if not 0 <= adc_entero <= 1023:
            return None

        return int(tiempo_us), adc_entero
    except (ValueError, UnicodeError):
        return None


def calibrar_offset(ser: serial.Serial) -> float:
    """Calcula el offset del amplificador con el shunt puenteado."""
    print("\nCALIBRACION DEL OFFSET")
    print("1. Mantener el sistema encendido.")
    print("2. Puentear momentaneamente las dos patas del shunt.")
    input("3. Presionar ENTER cuando el puente este colocado...")

    ser.reset_input_buffer()
    muestras_v = []
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


def crear_archivos() -> tuple[Path, Path, Path]:
    marca = datetime.now().strftime("%Y%m%d_%H%M%S")
    carpeta = Path(__file__).resolve().parent / "mediciones"
    carpeta.mkdir(parents=True, exist_ok=True)

    return (
        carpeta / f"medicion_{marca}.csv",
        carpeta / f"resumen_{marca}.csv",
        carpeta / f"grafica_{marca}.png",
    )


def calcular_resumen(
    tiempos: list[float],
    corrientes: list[float],
    estados: list[str],
) -> list[dict[str, float | int | str]]:
    """Calcula estadisticas y carga integrada para cada estado."""
    nombres = ["TOTAL", "ACTIVO", "STANDBY", "APAGADO", "EMERGENCIA"]
    resumen = []

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

            corriente_media_intervalo = (corrientes[i - 1] + corrientes[i]) / 2.0
            carga_mAh += corriente_media_intervalo * dt_s / 3600.0
            duracion += dt_s

        corriente_media = statistics.fmean(valores)

        resumen.append(
            {
                "estado": nombre,
                "muestras": len(valores),
                "duracion_s": duracion,
                "corriente_min_mA": min(valores),
                "corriente_max_mA": max(valores),
                "corriente_media_mA": corriente_media,
                "desviacion_mA": statistics.pstdev(valores),
                "carga_mAh": carga_mAh,
                "potencia_media_mW": TENSION_ALIMENTACION_V * corriente_media,
                "energia_mWh": TENSION_ALIMENTACION_V * carga_mAh,
            }
        )

    return resumen


def main() -> None:
    puerto = elegir_puerto()
    print(f"\nAbriendo {puerto} a {BAUDIOS} baudios...")

    ser = serial.Serial(puerto, BAUDIOS, timeout=0.2)
    time.sleep(2.0)  # El segundo Arduino se reinicia al abrir el puerto.
    ser.reset_input_buffer()

    offset_v = calibrar_offset(ser)
    archivo_csv, archivo_resumen, archivo_png = crear_archivos()

    print(f"\nCSV: {archivo_csv}")
    print("Teclas: 1=ACTIVO, 2=STANDBY, 3=APAGADO, 4=EMERGENCIA, Q=salir")

    cola: queue.Queue[tuple[int, int]] = queue.Queue()
    ejecutando = threading.Event()
    ejecutando.set()

    def lector_serial() -> None:
        while ejecutando.is_set():
            dato = leer_linea(ser)
            if dato is not None:
                cola.put(dato)

    hilo = threading.Thread(target=lector_serial, daemon=True)
    hilo.start()

    # Datos completos para CSV y resumen.
    todos_tiempos: list[float] = []
    todos_adc: list[int] = []
    todos_vout: list[float] = []
    todos_vshunt: list[float] = []
    todas_corrientes: list[float] = []
    todos_estados: list[str] = []

    # Ventana movil para que la grafica siga siendo rapida.
    graf_t = deque()
    graf_i = deque()

    estado_actual = "ACTIVO"
    cambios_estado: list[tuple[float, str]] = [(0.0, estado_actual)]

    primer_us: int | None = None
    ultimo_us: int | None = None
    vueltas_micros = 0
    finalizado = False

    csv_f = archivo_csv.open("w", newline="", encoding="utf-8")
    escritor = csv.writer(csv_f)
    escritor.writerow(
        [
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
    fig.canvas.manager.set_window_title("LAB 5 - Consumo de corriente")

    linea_cruda, = ax.plot([], [], color="0.75", linewidth=0.7, label="Corriente")
    linea_suave, = ax.plot([], [], color="#0066CC", linewidth=2.0, label="Promedio movil")
    texto_info = ax.text(
        0.02,
        0.96,
        "",
        transform=ax.transAxes,
        va="top",
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "0.75"},
    )

    ax.set_title("Corriente del sistema en funcion del tiempo")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Corriente (mA)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    def marcar_estado(nuevo_estado: str) -> None:
        nonlocal estado_actual
        estado_actual = nuevo_estado
        tiempo = todos_tiempos[-1] if todos_tiempos else 0.0
        cambios_estado.append((tiempo, nuevo_estado))
        ax.axvline(tiempo, color="black", linestyle="--", alpha=0.45)
        ax.text(
            tiempo,
            ax.get_ylim()[1],
            nuevo_estado,
            rotation=90,
            va="top",
            ha="right",
            fontsize=8,
        )
        print(f"Estado marcado: {nuevo_estado} en t={tiempo:.3f} s")

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

    def procesar_muestra(us_crudo: int, adc: int) -> None:
        nonlocal primer_us, ultimo_us, vueltas_micros

        if ultimo_us is not None and us_crudo < ultimo_us:
            vueltas_micros += 2**32
        ultimo_us = us_crudo

        us_extendido = us_crudo + vueltas_micros
        if primer_us is None:
            primer_us = us_extendido

        tiempo_s = (us_extendido - primer_us) / 1_000_000.0
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
                f"{tiempo_s:.6f}",
                us_crudo,
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
                us_crudo, adc = cola.get_nowait()
            except queue.Empty:
                break
            procesar_muestra(us_crudo, adc)
            cantidad += 1

        if not graf_t:
            return linea_cruda, linea_suave, texto_info

        t = np.asarray(graf_t, dtype=float)
        corriente = np.asarray(graf_i, dtype=float)

        # Promedio movil de 0,25 s para visualizar sin modificar el CSV.
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
            f"Estado: {estado_actual}\n"
            f"Corriente: {corriente[-1]:.2f} mA\n"
            f"Media 1 s: {media_corta:.2f} mA\n"
            f"A0: {voltaje_a0_actual:.3f} V\n"
            f"Offset LM358: {offset_v:.4f} V"
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

        resumen = calcular_resumen(todos_tiempos, todas_corrientes, todos_estados)

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
