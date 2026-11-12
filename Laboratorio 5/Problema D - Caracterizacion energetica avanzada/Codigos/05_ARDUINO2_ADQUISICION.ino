/*
 * Arduino 2 - adquisición para Laboratorio 5 Problema D
 * Lee A0 aproximadamente cada 5 ms y transmite: micros,adc
 * Velocidad: 115200 baudios
 */

const unsigned long PERIODO_US = 5000UL;
unsigned long proxima_muestra = 0UL;

void setup()
{
    Serial.begin(115200);
    proxima_muestra = micros();
}

void loop()
{
    unsigned long ahora = micros();

    if ((long)(ahora - proxima_muestra) >= 0L) {
        proxima_muestra += PERIODO_US;

        unsigned int adc = analogRead(A0);

        Serial.print(ahora);
        Serial.print(',');
        Serial.println(adc);
    }
}

