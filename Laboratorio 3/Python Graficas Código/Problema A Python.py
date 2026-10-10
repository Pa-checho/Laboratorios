"""Registro y graficas del Problema A para ejecutar desde Visual Studio."""

import csv
import re
import time
import unicodedata
from datetime import datetime
from pathlib import Path

import matplotlib

# Abre la grafica en una ventana externa de Windows.
matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
import serial


PUERTO = "COM19"
VELOCIDAD = 9600

DURACION_SEGUNDOS = 0
# 0: funciona hasta cerrar la ventana o usar Ctrl+C.

VENTANA_GRAFICA_SEGUNDOS = 60


def interpretar(linea, rango):
    """
    Extrae:
    - rango medio
    - temperatura
    - setpoint
    - calefactor
    - ventilador
    """

    texto = unicodedata.normalize(
        "NFKD",
        linea.lower()
    )

    texto = "".join(
        c
        for c in texto
        if not unicodedata.combining(c)
    )

    numero = r"(-?\d+(?:[.,]\d+)?)"


    # --------------------------------------------------
    # RANGO MEDIO
    # --------------------------------------------------

    limites = re.search(
        r"rango medio\s*:\s*" + numero + r"\s+a\s+" + numero,
        texto
    )

    if limites:

        rango = tuple(
            float(x.replace(",", "."))
            for x in limites.groups()
        )


    # --------------------------------------------------
    # TEMPERATURA
    # --------------------------------------------------

    temperatura = re.search(
        r"temperatura\s*:\s*" + numero,
        texto
    )

    if not temperatura:

        return rango, None


    # --------------------------------------------------
    # SETPOINT
    # --------------------------------------------------

    setpoint = re.search(
        r"setpoint\s*:\s*" + numero,
        texto
    )


    # Si por alguna razon llega una medicion
    # sin setpoint, no se inventa ningun valor.
    if not setpoint:

        return rango, None


    # --------------------------------------------------
    # CALEFACTOR
    # --------------------------------------------------

    calefactor = re.search(
        r"calefactor\s*:?\s*(apagado|encendido)",
        texto
    )


    # --------------------------------------------------
    # VENTILADOR
    # --------------------------------------------------

    ventilador = re.search(
        r"ventilador\s*:?\s*(?:a\s+)?(?:velocidad\s+)?"
        r"(apagado|baj[oa]|medi[oa]|alt[oa])\b",
        texto
    )


    if not calefactor or not ventilador:

        raise ValueError(
            "No reconozco los estados de esta medicion: "
            + linea
        )


    niveles = {
        "apagado": 0,
        "bajo": 1,
        "baja": 1,
        "medio": 2,
        "media": 2,
        "alto": 3,
        "alta": 3,
    }


    inferior, superior = (
        rango
        if rango is not None
        else (None, None)
    )


    medicion = [

        float(
            temperatura[1].replace(",", ".")
        ),

        float(
            setpoint[1].replace(",", ".")
        ),

        int(
            calefactor[1] == "encendido"
        ),

        niveles[
            ventilador[1]
        ],

        inferior,

        superior,
    ]


    return rango, medicion


def crear_grafica():
    """Crea la ventana que se actualizara durante el registro."""

    figura, ejes = plt.subplots(
        3,
        1,
        sharex=True,
        figsize=(10, 8),
        gridspec_kw={
            "height_ratios": [2, 1, 1]
        },
    )


    figura.suptitle(
        "Problema A - Esperando mediciones..."
    )


    ejes[0].set_ylabel(
        "Temperatura (C)"
    )

    ejes[1].set_ylabel(
        "Calefactor"
    )

    ejes[2].set_ylabel(
        "Ventilador"
    )

    ejes[2].set_xlabel(
        "Tiempo desde el inicio del registro (s)"
    )


    for eje in ejes:

        eje.grid(
            alpha=0.2
        )

        eje.set_xlim(
            0,
            VENTANA_GRAFICA_SEGUNDOS
        )


    figura.tight_layout(
        rect=[0, 0, 1, 0.94]
    )

    plt.ion()

    plt.show(
        block=False
    )


    return figura, ejes


def actualizar_grafica(
    figura,
    ejes,
    datos
):

    """Redibuja la ventana usando todas las mediciones recibidas."""


    (
        tiempos,
        temperaturas,
        setpoints,
        calefactor,
        ventilador,
        inferiores,
        superiores
    ) = zip(*datos)


    # Limpia las tres graficas.
    for eje in ejes:

        eje.clear()


    estado_calefactor = (
        "Encendido"
        if calefactor[-1]
        else "Apagado"
    )


    nombres_ventilador = [
        "Apagado",
        "Bajo",
        "Medio",
        "Alto"
    ]


    # --------------------------------------------------
    # TITULO GENERAL
    # --------------------------------------------------

    figura.suptitle(

        f"Temperatura: {temperaturas[-1]:.1f} C   "
        f"Setpoint: {setpoints[-1]:.1f} C   "
        f"Calefactor: {estado_calefactor}   "
        f"Ventilador: "
        f"{nombres_ventilador[ventilador[-1]]}"

    )


    # ==================================================
    # GRAFICA 1
    # TEMPERATURA + SETPOINT
    # ==================================================

    ejes[0].plot(

        tiempos,

        temperaturas,

        "o-",

        color="#c45516",

        label="Temperatura"
    )


    # --------------------------------------------------
    # SETPOINT COLOCADO POR UART
    # --------------------------------------------------

    ejes[0].step(

        tiempos,

        setpoints,

        where="post",

        color="#c62828",

        linestyle="--",

        linewidth=2,

        label="Setpoint colocado"
    )


    # --------------------------------------------------
    # RANGO IDEAL
    # --------------------------------------------------

    if any(
        x is not None
        for x in inferiores
    ):

        bajos = [
            float("nan")
            if x is None
            else x
            for x in inferiores
        ]


        altos = [
            float("nan")
            if x is None
            else x
            for x in superiores
        ]


        ejes[0].fill_between(

            tiempos,

            bajos,

            altos,

            step="post",

            color="#70b887",

            alpha=0.25,

            label="Rango ideal informado por Arduino"
        )


        ejes[0].step(

            tiempos,

            bajos,

            where="post",

            color="#368052",

            linestyle="--"
        )


        ejes[0].step(

            tiempos,

            altos,

            where="post",

            color="#368052",

            linestyle="--"
        )


    ejes[0].set_ylabel(
        "Temperatura (C)"
    )


    ejes[0].legend(
        loc="best",
        fontsize=9
    )


    # ==================================================
    # GRAFICA 2
    # CALEFACTOR
    # ==================================================

    ejes[1].step(

        tiempos,

        calefactor,

        where="post",

        color="#ae3e35",

        marker="."
    )


    ejes[1].set_yticks(
        [0, 1],
        [
            "Apagado",
            "Encendido"
        ]
    )


    ejes[1].set_ylim(
        -0.2,
        1.2
    )


    ejes[1].set_ylabel(
        "Calefactor"
    )


    # ==================================================
    # GRAFICA 3
    # VENTILADOR
    # ==================================================

    ejes[2].step(

        tiempos,

        ventilador,

        where="post",

        color="#266da4",

        marker="."
    )


    ejes[2].set_yticks(

        [0, 1, 2, 3],

        [
            "Apagado",
            "Bajo",
            "Medio",
            "Alto"
        ]
    )


    ejes[2].set_ylim(
        -0.3,
        3.3
    )


    ejes[2].set_ylabel(
        "Ventilador"
    )


    ejes[2].set_xlabel(
        "Tiempo desde el inicio del registro (s)"
    )


    # --------------------------------------------------
    # VENTANA DE TIEMPO
    # --------------------------------------------------

    for eje in ejes:

        eje.grid(
            alpha=0.2
        )


        limite_derecho = max(
            VENTANA_GRAFICA_SEGUNDOS,
            tiempos[-1]
        )


        limite_izquierdo = max(
            0,
            limite_derecho
            -
            VENTANA_GRAFICA_SEGUNDOS
        )


        eje.set_xlim(
            limite_izquierdo,
            limite_derecho
        )


    figura.tight_layout(
        rect=[0, 0, 1, 0.94]
    )


    figura.canvas.draw()

    figura.canvas.flush_events()

    plt.pause(
        0.01
    )


def main():

    carpeta = (
        Path(__file__).resolve().parent
        /
        "mediciones"
    )


    carpeta.mkdir(
        parents=True,
        exist_ok=True
    )


    nombre = datetime.now().strftime(
        "mediciones_%Y%m%d_%H%M%S_%f"
    )


    archivo_csv = carpeta / (
        nombre + ".csv"
    )


    archivo_txt = carpeta / (
        nombre + "_mensajes.txt"
    )


    datos = []

    rango = None


    figura, ejes = crear_grafica()


    print(
        f"Abriendo {PUERTO} a {VELOCIDAD} baudios..."
    )


    try:

        with serial.Serial(
            PUERTO,
            VELOCIDAD,
            timeout=0.2
        ) as arduino:


            # El Arduino Uno puede reiniciarse
            # cuando se abre el puerto USB.
            time.sleep(2)


            arduino.reset_input_buffer()


            with (
                archivo_csv.open(
                    "x",
                    newline="",
                    encoding="utf-8"
                ) as salida,

                archivo_txt.open(
                    "x",
                    encoding="utf-8"
                ) as mensajes
            ):


                escritor = csv.writer(
                    salida
                )


                # ------------------------------------------
                # ENCABEZADO CSV
                # ------------------------------------------

                escritor.writerow([

                    "tiempo_s",

                    "temperatura_C",

                    "setpoint_C",

                    "calefactor",

                    "nivel_ventilador",

                    "limite_inferior_C",

                    "limite_superior_C",
                ])


                salida.flush()


                if DURACION_SEGUNDOS == 0:

                    print(
                        "Registro continuo. "
                        "Cerra la ventana o presiona "
                        "Ctrl+C para terminar."
                    )

                else:

                    print(
                        f"Registrando durante "
                        f"{DURACION_SEGUNDOS} segundos."
                    )


                print(
                    f"Los datos se guardan en: "
                    f"{archivo_csv}"
                )


                inicio = time.monotonic()

                pendiente = b""


                try:

                    while (

                        plt.fignum_exists(
                            figura.number
                        )

                        and

                        (
                            DURACION_SEGUNDOS == 0

                            or

                            time.monotonic() - inicio
                            <
                            DURACION_SEGUNDOS
                        )
                    ):


                        pendiente += arduino.read(
                            arduino.in_waiting
                            or
                            1
                        )


                        while b"\n" in pendiente:


                            cruda, pendiente = (
                                pendiente.split(
                                    b"\n",
                                    1
                                )
                            )


                            linea = cruda.decode(

                                "utf-8",

                                errors="replace"

                            ).strip()


                            if not linea:

                                continue


                            mensajes.write(
                                linea + "\n"
                            )


                            mensajes.flush()


                            print(
                                linea
                            )


                            try:

                                rango, medicion = interpretar(
                                    linea,
                                    rango
                                )


                            except ValueError as error:

                                print(
                                    "AVISO:",
                                    error
                                )

                                continue


                            if medicion is not None:


                                fila = [

                                    round(
                                        time.monotonic()
                                        -
                                        inicio,
                                        3
                                    )

                                ] + medicion


                                datos.append(
                                    fila
                                )


                                escritor.writerow(
                                    fila
                                )


                                salida.flush()


                                actualizar_grafica(
                                    figura,
                                    ejes,
                                    datos
                                )


                        plt.pause(
                            0.01
                        )


                finally:

                    if pendiente:

                        mensajes.write(

                            pendiente.decode(

                                "utf-8",

                                errors="replace"
                            )
                        )


    except KeyboardInterrupt:

        print(
            "Registro detenido. "
            "Conservando las mediciones recibidas."
        )


    except serial.SerialException as error:

        print(
            "No se pudo abrir o mantener "
            "la conexion:",
            error
        )


        print(
            "Revisa el USB y el puerto; "
            "cierra el Monitor Serie "
            "y el Serial Plotter."
        )


    # ------------------------------------------------------
    # GUARDAR GRAFICA
    # ------------------------------------------------------

    if datos:

        archivo_png = carpeta / (
            nombre + ".png"
        )


        figura.savefig(
            archivo_png,
            dpi=160
        )


        print(
            f"Guardadas {len(datos)} mediciones. "
            f"CSV: {archivo_csv}"
        )


        print(
            f"Grafica guardada: {archivo_png}"
        )


        plt.ioff()


        if plt.fignum_exists(
            figura.number
        ):

            plt.show()


    else:

        plt.close(
            figura
        )


        print(
            "No se recibieron mediciones reconocibles. "
            "Revisa los mensajes de la consola."
        )


    print(
        "Registro terminado. Puerto cerrado."
    )


if __name__ == "__main__":

    main()