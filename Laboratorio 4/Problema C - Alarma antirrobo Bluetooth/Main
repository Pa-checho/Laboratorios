#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdint.h>
#include <string.h>

#define UART_BAUD 9600UL
#define UART_UBRR ((F_CPU / (16UL * UART_BAUD)) - 1UL)
#define UMBRAL_LDR_ABIERTA 500U
#define LCD_BACKLIGHT 0x08U

#define PIN_MOVIMIENTO PD2
#define LED_VERDE PB0   /* Arduino D8 */
#define LED_ROJO PB1    /* Arduino D9 */
#define BUZZER PB2      /* Arduino D10 */

typedef enum {
	MODO_AUTO,
	MODO_MANUAL,
	MODO_FORZADO
} modo_t;

typedef enum {
	DESARMADA,
	ARMADA_AUTO,
	ARMADA_MANUAL,
	ALARMA_SONANDO,
	ALARMA_SILENCIADA
} estado_t;

static estado_t estado = DESARMADA;
static modo_t modo = MODO_AUTO;
static uint8_t ventana_abierta = 0;
static char zona_alarma[12] = "";
static char ultimo_evento_manual[12] = "";
static uint8_t lcd_i2c_address = 0;
static char lcd_ultima_linea_1[16];
static char lcd_ultima_linea_2[16];
static volatile uint8_t buzzer_habilitado = 0;
static volatile uint8_t contador_barrido = 0;
static volatile uint8_t paso_barrido = 0;
static volatile int8_t direccion_barrido = 1;

/* Barrido de sirena más rápido, aproximadamente de 1500 a 2080 Hz */
ISR(TIMER1_COMPA_vect)
{
	if (buzzer_habilitado) {
		PINB = (1 << BUZZER); /* Alterna D10 para generar el tono */

		/* Actualiza la frecuencia cada 80 semiperiodos del tono */
		if (++contador_barrido >= 80) {
			contador_barrido = 0;

			if (direccion_barrido > 0) {
				if (paso_barrido >= 31) {
					direccion_barrido = -1;
					} else {
					paso_barrido++;
				}
				} else if (paso_barrido == 0) {
				direccion_barrido = 1;
				} else {
				paso_barrido--;
			}

			OCR1A = (uint16_t)(480U - (uint16_t)paso_barrido * 5U);
		}
		} else {
		PORTB &= ~(1 << BUZZER);
	}
}

static void buzzer_timer_init(void)
{
	TCCR1A = 0;
	TCCR1B = (1 << WGM12); /* CTC, aún detenido */
	OCR1A = 480;           /* Comienza cerca de 1500 Hz */
	TIMSK1 |= (1 << OCIE1A);
	TCCR1B |= (1 << CS11); /* Inicia Timer1 con prescaler 8 */
}

static void twi_init(void)
{
	TWSR = 0;
	TWBR = (uint8_t)(((F_CPU / 100000UL) - 16UL) / 2UL); /* I2C a 100 kHz */
	TWCR = (1 << TWEN);
}

static uint8_t twi_start(uint8_t address_byte)
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT))) {
	}

	uint8_t status = TWSR & 0xF8;
	if (status != TW_START && status != TW_REP_START) {
		return 0;
	}

	TWDR = address_byte;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT))) {
	}

	return (TWSR & 0xF8) == TW_MT_SLA_ACK;
}

static uint8_t twi_write(uint8_t value)
{
	TWDR = value;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT))) {
	}

	return (TWSR & 0xF8) == TW_MT_DATA_ACK;
}

static void twi_stop(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	while (TWCR & (1 << TWSTO)) {
	}
}

static uint8_t i2c_write_byte(uint8_t address, uint8_t value)
{
	if (!twi_start((uint8_t)(address << 1)) ||
	!twi_write(value)) {
		twi_stop();
		return 0;
	}
	twi_stop();
	return 1;
}

static uint8_t i2c_probe(uint8_t address)
{
	uint8_t responded = twi_start((uint8_t)(address << 1));
	twi_stop();
	return responded;
}

static uint8_t lcd_find_i2c_address(void)
{
	uint8_t address;

	
	for (address = 0x20; address <= 0x27; address++) {
		if (i2c_probe(address)) {
			return address;
		}
	}
	for (address = 0x38; address <= 0x3F; address++) {
		if (i2c_probe(address)) {
			return address;
		}
	}
	return 0;
}

static void lcd_write_nibble(uint8_t nibble, uint8_t register_select)
{
	uint8_t value = (uint8_t)((nibble & 0xF0) | LCD_BACKLIGHT | register_select);
	i2c_write_byte(lcd_i2c_address, (uint8_t)(value | 0x04));
	_delay_us(1);
	i2c_write_byte(lcd_i2c_address, value);
	_delay_us(50);
}

static void lcd_send(uint8_t value, uint8_t register_select)
{
	lcd_write_nibble(value, register_select);
	lcd_write_nibble((uint8_t)(value << 4), register_select);
}

static void lcd_command(uint8_t command)
{
	lcd_send(command, 0);
	if (command == 0x01 || command == 0x02) {
		_delay_ms(2);
	}
}

static void lcd_data(uint8_t data)
{
	lcd_send(data, 0x01);
}

static void lcd_init(void)
{
	_delay_ms(50);
	lcd_i2c_address = lcd_find_i2c_address();
	if (lcd_i2c_address == 0) {
		return;
	}

	lcd_write_nibble(0x30, 0);
	_delay_ms(5);
	lcd_write_nibble(0x30, 0);
	_delay_us(150);
	lcd_write_nibble(0x30, 0);
	_delay_us(150);
	lcd_write_nibble(0x20, 0);
	_delay_us(150);
	lcd_command(0x28); /* 2 lineas, matriz 5x8 */
	lcd_command(0x0C); /* Pantalla encendida, cursor apagado */
	lcd_command(0x01); /* Limpiar pantalla */
	lcd_command(0x06); /* Avanzar cursor */
}

static void lcd_format_line(char line[16], const char *text)
{
	uint8_t i = 0;
	while (i < 16 && text[i] != '\0') {
		line[i] = text[i];
		i++;
	}
	while (i < 16) {
		line[i++] = ' ';
	}
}

static void lcd_write_line(uint8_t row, const char line[16], char previous[16])
{
	uint8_t i;
	if (lcd_i2c_address == 0) {
		return;
	}
	if (memcmp(line, previous, 16) == 0) {
		return;
	}

	lcd_command((uint8_t)(0x80 | (row == 0 ? 0x00 : 0x40)));
	for (i = 0; i < 16; i++) {
		lcd_data((uint8_t)line[i]);
	}
	memcpy(previous, line, 16);
}

static void lcd_update(void)
{
	char line1[16];
	char line2[16];
	const char *mode_text = modo == MODO_MANUAL ? "ESTADO: M" :
	(modo == MODO_FORZADO && estado != DESARMADA)
	? "ESTADO: F" : "ESTADO: A";
	const char *state_text;

	switch (estado) {
		case DESARMADA:
		state_text = "DESARMADA";
		break;
		case ARMADA_AUTO:
		state_text = "ARMADA";
		break;
		case ARMADA_MANUAL:
		if (strcmp(ultimo_evento_manual, "VENTANA") == 0) {
			state_text = "EVENTO:VENTANA";
			} else if (strcmp(ultimo_evento_manual, "MOVIMIENTO") == 0) {
			state_text = "EVENTO:MOVIM.";
			} else {
			state_text = "ARMADA";
		}
		break;
		case ALARMA_SILENCIADA:
		state_text = "SILENCIADA";
		break;
		case ALARMA_SONANDO:
		if (strcmp(zona_alarma, "VENTANA") == 0) {
			state_text = "ACTIVA:VENTANA";
			} else if (strcmp(zona_alarma, "MOVIMIENTO") == 0) {
			state_text = "ACTIVA:MOVIM.";
			} else {
			state_text = "ACTIVA:FORZADA";
		}
		break;
		default:
		state_text = "DESARMADA";
		break;
	}

	lcd_format_line(line1, mode_text);
	lcd_format_line(line2, state_text);
	lcd_write_line(0, line1, lcd_ultima_linea_1);
	lcd_write_line(1, line2, lcd_ultima_linea_2);
}

static void uart_init(void)
{
	UBRR0H = (uint8_t)(UART_UBRR >> 8);
	UBRR0L = (uint8_t)UART_UBRR;
	UCSR0A = 0;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); /* 8 bits, sin paridad, 1 stop */
}

static void uart_send_char(char value)
{
	while (!(UCSR0A & (1 << UDRE0))) {
	}
	UDR0 = value;
}

static void uart_send_text(const char *text)
{
	while (*text) {
		uart_send_char(*text++);
	}
}

static uint8_t uart_read_char(char *value)
{
	if (!(UCSR0A & (1 << RXC0))) {
		return 0;
	}
	*value = UDR0;
	return 1;
}

static void adc_init(void)
{
	DDRC &= ~(1 << DDC0);
	PORTC &= ~(1 << PORTC0);

	ADMUX = (1 << REFS0); /* Referencia AVcc y canal A0 */
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

static uint16_t adc_read_a0(void)
{
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC)) {
	}
	return ADC;
}

static void outputs_update(void)
{
	uint8_t outputs = PORTB & ~((1 << LED_VERDE) | (1 << LED_ROJO) |
	(1 << BUZZER));

	if (estado == ARMADA_AUTO || estado == ARMADA_MANUAL ||
	estado == ALARMA_SILENCIADA) {
		outputs |= (1 << LED_VERDE);
	}

	if (estado == ALARMA_SONANDO || estado == ALARMA_SILENCIADA) {
		outputs |= (1 << LED_ROJO);
	}

	PORTB = outputs;
	if (estado == ALARMA_SONANDO && !buzzer_habilitado) {
		contador_barrido = 0;
		paso_barrido = 0;
		direccion_barrido = 1;
		OCR1A = 666;
	}
	buzzer_habilitado = (estado == ALARMA_SONANDO);
	if (!buzzer_habilitado) {
		PORTB &= ~(1 << BUZZER);
	}
}

static void send_status(void)
{
	switch (estado) {
		case DESARMADA:
		uart_send_text("ESTADO:DESARMADA\r\n");
		break;
		case ARMADA_AUTO:
		uart_send_text("ESTADO:ARMADA_AUTO\r\n");
		break;
		case ARMADA_MANUAL:
		uart_send_text("ESTADO:ARMADA_MANUAL\r\n");
		break;
		case ALARMA_SONANDO:
		uart_send_text("ESTADO:ALARMA_SONANDO;MODO:");
		uart_send_text(modo == MODO_AUTO ? "AUTO\r\n" :
		modo == MODO_MANUAL ? "MANUAL\r\n" : "FORZADO\r\n");
		break;
		case ALARMA_SILENCIADA:
		uart_send_text("ESTADO:ALARMA_SILENCIADA;MODO:");
		uart_send_text(modo == MODO_AUTO ? "AUTO\r\n" :
		modo == MODO_MANUAL ? "MANUAL\r\n" : "FORZADO\r\n");
		break;
	}
}

static void send_menu(void)
{
	uart_send_text("\r\n=== MENU DE LA ALARMA ===\r\n");
	uart_send_text("1 - ALARMA_AUTO   - Armar en modo automatico\r\n");
	uart_send_text("2 - ALARMA_MANUAL - Armar en modo manual\r\n");
	uart_send_text("3 - DESACTIVAR    - Desarmar la alarma\r\n");
	uart_send_text("4 - SILENCIO      - Silenciar hasta desarmar\r\n");
	uart_send_text("5 - FORZAR        - Activar la sirena manualmente\r\n");
	uart_send_text("6 - ESTADO        - Consultar el estado actual\r\n");
	uart_send_text("Escribe el numero y presiona Enter.\r\n\r\n");
}

static void report_zone(const char *zone)
{
	if (estado == DESARMADA) {
		return;
	}

	if (estado == ALARMA_SILENCIADA) {
		uart_send_text("EVENTO:");
		uart_send_text(zone);
		uart_send_text(";ALARMA:SILENCIADA\r\n");
		return;
	}

	if (estado == ALARMA_SONANDO) {
		strncpy(zona_alarma, zone, sizeof(zona_alarma) - 1);
		zona_alarma[sizeof(zona_alarma) - 1] = '\0';
		uart_send_text("EVENTO:");
		uart_send_text(zone);
		uart_send_text(";ALARMA:SONANDO\r\n");
		return;
	}

	if (modo == MODO_MANUAL) {
		strncpy(ultimo_evento_manual, zone, sizeof(ultimo_evento_manual) - 1);
		ultimo_evento_manual[sizeof(ultimo_evento_manual) - 1] = '\0';
		uart_send_text("EVENTO:");
		uart_send_text(zone);
		uart_send_text(";MODO:MANUAL\r\n");
		return;
	}

	strncpy(zona_alarma, zone, sizeof(zona_alarma) - 1);
	zona_alarma[sizeof(zona_alarma) - 1] = '\0';
	estado = ALARMA_SONANDO;
	outputs_update();
	uart_send_text("ALARMA:SONANDO;ZONA:");
	uart_send_text(zone);
	uart_send_text("\r\n");
}

static void arm_auto(void)
{
	if (estado == ALARMA_SONANDO) {
		uart_send_text("ERROR:ALARMA_SONANDO\r\n");
		return;
	}

	ultimo_evento_manual[0] = '\0';
	modo = MODO_AUTO;
	if (estado != ALARMA_SILENCIADA) {
		estado = ARMADA_AUTO;
	}
	outputs_update();

	if (ventana_abierta) {
		report_zone("VENTANA");
		} else {
		send_status();
	}
}

static void arm_manual(void)
{
	if (estado == ALARMA_SONANDO) {
		uart_send_text("ERROR:ALARMA_SONANDO\r\n");
		return;
	}

	ultimo_evento_manual[0] = '\0';
	modo = MODO_MANUAL;
	if (estado != ALARMA_SILENCIADA) {
		estado = ARMADA_MANUAL;
	}
	outputs_update();

	if (ventana_abierta) {
		strcpy(ultimo_evento_manual, "VENTANA");
		uart_send_text("EVENTO:VENTANA;MODO:MANUAL\r\n");
		} else {
		send_status();
	}
}

static void force_alarm(void)
{
	if (estado == ALARMA_SILENCIADA) {
		uart_send_text("ALARMA:SILENCIADA\r\n");
		return;
	}

	if (estado == DESARMADA) {
		modo = MODO_FORZADO;
	}

	strcpy(zona_alarma, "FORZADA");
	estado = ALARMA_SONANDO;
	outputs_update();
	uart_send_text("ALARMA:SONANDO;ZONA:FORZADA\r\n");
}

static void process_command(const char *command)
{
	if (strcmp(command, "1") == 0) {
		arm_auto();
		} else if (strcmp(command, "2") == 0) {
		arm_manual();
		} else if (strcmp(command, "3") == 0) {
		estado = DESARMADA;
		zona_alarma[0] = '\0';
		ultimo_evento_manual[0] = '\0';
		outputs_update();
		send_status();
		} else if (strcmp(command, "4") == 0) {
		if (estado == ALARMA_SONANDO) {
			estado = ALARMA_SILENCIADA;
			outputs_update();
			uart_send_text("ALARMA:SILENCIADA\r\n");
			} else {
			uart_send_text("ERROR:ALARMA_NO_ACTIVA\r\n");
		}
		} else if (strcmp(command, "5") == 0) {
		force_alarm();
		} else if (strcmp(command, "6") == 0) {
		send_status();
		} else if (command[0] != '\0') {
		uart_send_text("ERROR:COMANDO_DESCONOCIDO\r\n");
	}

	if (command[0] != '\0') {
		send_menu();
	}
}

int main(void)
{
	char command[32];
	uint8_t command_length = 0;
	uint8_t button_history = 0;
	uint8_t button_stable_pressed = 0;
	uint8_t ldr_sample_counter = 0;
	uint8_t lcd_refresh_counter = 0;

	DDRB |= (1 << LED_VERDE) | (1 << LED_ROJO) | (1 << BUZZER);
	outputs_update();
	buzzer_timer_init();
	sei();

	DDRD &= ~(1 << PIN_MOVIMIENTO);
	PORTD |= (1 << PIN_MOVIMIENTO); /* Botón entre D2 y GND. */

	uart_init();
	adc_init();
	twi_init();
	lcd_init();
	ventana_abierta = (adc_read_a0() >= UMBRAL_LDR_ABIERTA);
	lcd_update();
	send_menu();
	send_status();

	while (1) {
		char received;

		if (uart_read_char(&received)) {
			if (received == '\r' || received == '\n') {
				command[command_length] = '\0';
				process_command(command);
				command_length = 0;
				} else if (command_length < sizeof(command) - 1) {
				command[command_length++] = received;
				} else {
				command_length = 0;
				uart_send_text("ERROR:COMANDO_DEMASIADO_LARGO\r\n");
			}
		}

		button_history = (uint8_t)((button_history << 1) |
		((PIND & (1 << PIN_MOVIMIENTO)) == 0));
		if (button_history == 0xFF && !button_stable_pressed) {
			button_stable_pressed = 1;
			report_zone("MOVIMIENTO");
			} else if (button_history == 0x00) {
			button_stable_pressed = 0;
		}

		if (++ldr_sample_counter >= 10) {
			uint8_t current_window_open =
			(adc_read_a0() >= UMBRAL_LDR_ABIERTA);
			ldr_sample_counter = 0;

			if (current_window_open != ventana_abierta) {
				ventana_abierta = current_window_open;
				if (ventana_abierta) {
					report_zone("VENTANA");
				}
			}
		}

		if (++lcd_refresh_counter >= 50) {
			lcd_refresh_counter = 0;
			lcd_update();
		}

		_delay_ms(1);
	}
}
