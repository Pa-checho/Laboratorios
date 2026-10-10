import re
import time

import matplotlib

# Fuerza una ventana grafica externa que se actualiza en tiempo real.
# Debe configurarse antes de importar pyplot.
matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
import serial


PUERTO = "COM18"
BAUDIOS = 9600

INTERVALO_GRAFICA = 0.2       # Redibuja 5 veces por segundo.
MAXIMO_MUESTRAS = 600         # Ultimos 60 s a 10 muestras por segundo.


# Formato recibido desde el Arduino:
# SP=512,LDR=480,POS=620,PWM=124,DIR=ABRIR
PATRON = re.compile(
    r"SP=(\d+),LDR=(\d+),POS=(\d+),PWM=(\d+),DIR=([A-Z]+)"
)


def main():

    tiempos = []
    setpoints = []
    valores_ldr = []
    posiciones = []
    valores_pwm = []

    # Se crean dos graficas:
    # 1. Setpoint, LDR y posicion del potenciometro.
    # 2. PWM y sentido del motor.
    figura, ejes = plt.subplots(
        2,
        1,
        figsize=(10, 8),
        sharex=True
    )

    grafica_luz, grafica_pwm = ejes


    # ------------------------------------------------------
    # PRIMERA GRAFICA
    # ------------------------------------------------------

    # Las tres variables ADC se muestran juntas.

    linea_setpoint, = grafica_luz.plot(
        [],
        [],
        color="red",
        label="Setpoint"
    )

    linea_ldr, = grafica_luz.plot(
        [],
        [],
        color="orange",
        label="Iluminacion LDR"
    )

    linea_posicion, = grafica_luz.plot(
        [],
        [],
        color="blue",
        label="Posicion potenciometro"
    )


    grafica_luz.set_title(
        "Setpoint, iluminacion y posicion"
    )

    grafica_luz.set_ylabel(
        "Valor ADC"
    )

    grafica_luz.set_ylim(
        0,
        1023
    )

    grafica_luz.legend(
        loc="upper right"
    )


    # ------------------------------------------------------
    # SEGUNDA GRAFICA
    # ------------------------------------------------------

    # El PWM del motor se mantiene en una grafica independiente.

    linea_pwm, = grafica_pwm.plot(
        [],
        [],
        color="green"
    )


    grafica_pwm.set_title(
        "PWM y sentido del motor"
    )

    grafica_pwm.set_xlabel(
        "Tiempo (s)"
    )

    grafica_pwm.set_ylabel(
        "Cerrar <- PWM -> Abrir"
    )

    grafica_pwm.set_ylim(
        -255,
        255
    )

    # Linea horizontal que representa motor detenido.
    grafica_pwm.axhline(
        0,
        color="black",
        linewidth=0.8
    )


    # ------------------------------------------------------
    # CONFIGURACION GENERAL DE LAS GRAFICAS
    # ------------------------------------------------------

    for eje in ejes:

        eje.grid(
            True,
            alpha=0.3
        )

        eje.set_xlim(
            0,
            60
        )


    # Deja espacio arriba para mostrar
    # los valores actuales recibidos.
    figura.subplots_adjust(
        left=0.08,
        right=0.98,
        bottom=0.08,
        top=0.88,
        hspace=0.32
    )


    # Activa el modo interactivo de Matplotlib.
    plt.ion()

    # Muestra la ventana sin bloquear el programa.
    plt.show(
        block=False
    )


    # Permite terminar el programa
    # cerrando la ventana de las graficas.
    estado = {
        "activo": True
    }


    def cerrar_ventana(_evento):

        estado["activo"] = False


    figura.canvas.mpl_connect(
        "close_event",
        cerrar_ventana
    )


    print(
        f"Abriendo {PUERTO} a {BAUDIOS} baudios..."
    )


    try:

        with serial.Serial(
            PUERTO,
            BAUDIOS,
            timeout=0.05
        ) as puerto:


            # El Arduino puede reiniciarse
            # cuando se abre el puerto serie.
            time.sleep(2)


            # Limpia mensajes anteriores
            # que hayan quedado en el buffer.
            puerto.reset_input_buffer()


            # IMPORTANTE:
            # Ya NO se envia un setpoint desde Python.
            # El setpoint sera el que tenga configurado
            # el codigo del Arduino.

            print(
                "Conectado correctamente."
            )

            print(
                "El setpoint se toma directamente del Arduino."
            )

            print(
                "Graficando en tiempo real."
            )

            print(
                "Cerra la ventana para terminar."
            )


            inicio = time.monotonic()

            ultima_actualizacion_grafica = 0.0


            # ------------------------------------------------------
            # BUCLE PRINCIPAL
            # ------------------------------------------------------

            while estado["activo"]:


                linea = (
                    puerto.readline()
                    .decode(
                        "ascii",
                        errors="ignore"
                    )
                    .strip()
                )


                # Si no llegan datos,
                # mantiene activa la ventana.
                if not linea:

                    plt.pause(0.01)

                    continue


                # Comprueba que el mensaje tenga
                # el formato esperado.
                resultado = PATRON.fullmatch(
                    linea
                )


                if resultado is None:

                    # Si Arduino envia algun mensaje
                    # distinto al diagnostico,
                    # simplemente se muestra en consola.
                    print(linea)

                    plt.pause(0.01)

                    continue


                # Tiempo transcurrido desde que
                # comenzo la prueba.
                tiempo_actual = (
                    time.monotonic() - inicio
                )


                # ------------------------------------------------------
                # DATOS RECIBIDOS DEL ARDUINO
                # ------------------------------------------------------

                setpoint = int(
                    resultado.group(1)
                )

                ldr = int(
                    resultado.group(2)
                )

                posicion = int(
                    resultado.group(3)
                )

                pwm = int(
                    resultado.group(4)
                )

                direccion = (
                    resultado.group(5)
                )


                # ------------------------------------------------------
                # SENTIDO DEL MOTOR
                # ------------------------------------------------------

                # Convencion para la grafica:
                #
                # ABRIR  -> PWM positivo
                # CERRAR -> PWM negativo
                # PARADO -> 0

                if direccion == "ABRIR":

                    pwm_con_sentido = pwm


                elif direccion == "CERRAR":

                    pwm_con_sentido = -pwm


                else:

                    pwm_con_sentido = 0


                # ------------------------------------------------------
                # GUARDAR DATOS
                # ------------------------------------------------------

                tiempos.append(
                    tiempo_actual
                )

                setpoints.append(
                    setpoint
                )

                valores_ldr.append(
                    ldr
                )

                posiciones.append(
                    posicion
                )

                valores_pwm.append(
                    pwm_con_sentido
                )


                # ------------------------------------------------------
                # LIMITAR CANTIDAD DE MUESTRAS
                # ------------------------------------------------------

                # Conserva solamente aproximadamente
                # el ultimo minuto de datos.

                if len(tiempos) > MAXIMO_MUESTRAS:

                    tiempos.pop(0)

                    setpoints.pop(0)

                    valores_ldr.pop(0)

                    posiciones.pop(0)

                    valores_pwm.pop(0)


                # ------------------------------------------------------
                # ACTUALIZAR GRAFICAS
                # ------------------------------------------------------

                if (
                    tiempo_actual
                    - ultima_actualizacion_grafica
                    >= INTERVALO_GRAFICA
                ):


                    # Primera grafica:
                    # Setpoint
                    # LDR
                    # Posicion

                    linea_setpoint.set_data(
                        tiempos,
                        setpoints
                    )

                    linea_ldr.set_data(
                        tiempos,
                        valores_ldr
                    )

                    linea_posicion.set_data(
                        tiempos,
                        posiciones
                    )


                    # Segunda grafica:
                    # PWM y direccion.

                    linea_pwm.set_data(
                        tiempos,
                        valores_pwm
                    )


                    # --------------------------------------------------
                    # VENTANA DE LOS ULTIMOS 60 SEGUNDOS
                    # --------------------------------------------------

                    limite_derecho = max(
                        60,
                        tiempo_actual
                    )

                    limite_izquierdo = max(
                        0,
                        limite_derecho - 60
                    )


                    for eje in ejes:

                        eje.set_xlim(
                            limite_izquierdo,
                            limite_derecho
                        )


                    # --------------------------------------------------
                    # MOSTRAR VALORES ACTUALES ARRIBA
                    # --------------------------------------------------

                    figura.suptitle(

                        f"SP={setpoint}  "
                        f"LDR={ldr}  "
                        f"POS={posicion}  "
                        f"PWM={pwm}  "
                        f"DIR={direccion}",

                        fontsize=11,

                        y=0.97
                    )


                    # Redibuja la ventana.

                    figura.canvas.draw()

                    figura.canvas.flush_events()

                    plt.pause(0.001)


                    # Guarda el instante de la ultima
                    # actualizacion grafica.

                    ultima_actualizacion_grafica = (
                        time.monotonic() - inicio
                    )


                # ------------------------------------------------------
                # MOSTRAR DATOS EN CONSOLA
                # ------------------------------------------------------

                print(

                    f"SP={setpoint:4d}  "

                    f"LDR={ldr:4d}  "

                    f"POS={posicion:4d}  "

                    f"PWM={pwm:3d}  "

                    f"DIR={direccion}"
                )


    # ------------------------------------------------------
    # ERRORES
    # ------------------------------------------------------

    except serial.SerialException as error:

        print(
            f"No se pudo utilizar {PUERTO}: {error}"
        )

        print(
            "Cerra el Monitor Serie y comproba "
            "el numero de puerto."
        )


    except KeyboardInterrupt:

        print(
            "\nPrograma detenido."
        )


    finally:

        plt.close(
            "all"
        )


if __name__ == "__main__":

    main()