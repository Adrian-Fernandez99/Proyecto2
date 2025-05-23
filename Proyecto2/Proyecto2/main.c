/*
Universidad del Valle de Guatemala
IE2023 : Programación de Microcontroladores

Proyecto2.c

Created: 5/8/2025 11:29:27 AM
Author : Adrián Fernández

Descripción: Controlador de dos servos haciendo 
uso de dos potenciometros ditstintos.
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

// Prototipos de función
void setup();
void PWM_init();
void ADC_init();
void UART_init();

void write_char(char caracter);
void write_str(char* texto);

void mapeo_servo(uint8_t PWM_select, uint16_t ADC_servo1, uint16_t ADC_servo2, uint16_t* PWM_1, uint16_t* PWM_2);
uint16_t map_servo2(uint16_t ADC_need);
uint16_t ADC_read(uint8_t PIN);

void writeEEPROM(uint16_t direccion, uint8_t dato);
uint8_t read_EEPROM(uint16_t direccion);
void save_(uint8_t servo, uint8_t posicion, uint16_t dato);
uint8_t load_(uint8_t servo, uint8_t posicion, uint8_t punch);

void led_on(uint8_t modo, uint8_t place);

uint16_t ADC_servo1 = 0;
uint16_t ADC_servo2 = 0;
uint16_t ADC_servo3 = 0;
uint16_t ADC_servo4 = 0;

uint16_t PWM_1 = 0;
uint16_t PWM_2 = 0;
uint8_t PWM_3 = 0;
uint8_t PWM_4 = 0;

uint8_t modo = 0;
uint8_t place = 0;
uint8_t habilitar = 0;

uint8_t modisimo = 0;
uint8_t estadisimo = 0;

uint16_t servil1 = 0;
uint16_t servil2 = 0;
uint16_t servil3 = 0;
uint16_t servil4 = 0;

char comando;
char doble_comando;
volatile char buffer[10];
volatile uint8_t indice = 0;
volatile uint8_t new_data = 0;
volatile uint8_t largo_buff = 0;

void serviltime(uint16_t* servil);
void envio_datos(uint16_t dato, char prefijo);

// MAIN LOOP
int main(void)
{
	setup();
	while (1)
	{	
		led_on(modo, place);
		if (modo == 0)
		{
			if (new_data == 1)
			{
				new_data = 0;
				if (buffer[0] == 'M')
				{
					if (buffer[1] == '1')
					{
						modisimo = 1;
					}
				}
				else if (buffer[0] == 'P')
				{
					if (buffer[1] == '1')
					{
						estadisimo = 1;
					}
					else if (buffer[1] == '2')
					{
						if (modo != 1)
						{
							habilitar = 1;
						}
					}
				}
			}
			
			if (modisimo == 1)
			{
				modisimo = 0;
				modo = 1;
			}
			
			if (estadisimo == 1)
			{
				estadisimo = 0;
				place++;
				if (place == 4)
				{
					place = 0;
				}
			}
			
			ADC_servo1 = ADC_read(6);
			ADC_servo2 = ADC_read(7);
			
			mapeo_servo(1, ADC_servo1, ADC_servo2, &PWM_1, &PWM_2);
			mapeo_servo(0, ADC_servo1, ADC_servo2, &PWM_1, &PWM_2);
			
			OCR1A = PWM_1;
			OCR1B = PWM_2;
			
			ADC_servo3 = ADC_read(4);
			PWM_3 = map_servo2(ADC_servo3);
			
			OCR2A = PWM_3; // Setear duty cycle
			
			ADC_servo4 = ADC_read(5);
			PWM_4 = map_servo2(ADC_servo4);
			
			OCR2B = PWM_4; // Setear duty cycle
			
			if (habilitar == 1)
			{
				save_(1, place, ADC_servo1);
				save_(2, place, ADC_servo2);
				save_(3, place, ADC_servo3);
				save_(4, place, ADC_servo4);
				habilitar = 0;
			}

			_delay_ms(20);  // Tiempo entre actualizaciones
		}
		
		else if (modo == 1)
		{
			if (new_data == 1)
			{
				new_data = 0;
				if (buffer[0] == 'M')
				{
					if (buffer[1] == '1')
					{
						modisimo = 1;
					}
				}
				else if (buffer[0] == 'P')
				{
					if (buffer[1] == '1')
					{
						estadisimo = 1;
					}
					else if (buffer[1] == '2')
					{
						if (modo != 1)
						{
							habilitar = 1;
						}
					}
				}
			}
			
			if (modisimo == 1)
			{
				modisimo = 0;
				modo = 2;
			}
			
			if (estadisimo == 1)
			{
				estadisimo = 0;
				place++;
				if (place == 4)
				{
					place = 0;
				}
			}
			
			ADC_servo1 = ((load_(1, place, 0)) << 8) | (load_(1, place, 1));
			ADC_servo2 = ((load_(2, place, 0)) << 8) | (load_(2, place, 1));
			
			mapeo_servo(1, ADC_servo1, ADC_servo2, &PWM_1, &PWM_2);
			mapeo_servo(0, ADC_servo1, ADC_servo2, &PWM_1, &PWM_2);
			OCR1A = PWM_1;
			OCR1B = PWM_2;
			
			ADC_servo3 = ((load_(3, place, 0)) << 8) | (load_(3, place, 1));
			PWM_3 = map_servo2(ADC_servo3);
			OCR2A = PWM_3; // Setear duty cycle
			
			ADC_servo4 = ((load_(4, place, 0)) << 8) | (load_(4, place, 1));
			PWM_4 = map_servo2(ADC_servo4);
			OCR2B = PWM_4; // Setear duty cycle

			_delay_ms(20);  // Tiempo entre actualizaciones
		}
		
		else if (modo == 2)
		{
			if (new_data == 1)
			{
				new_data = 0;
				if (buffer[0] == '1')
				{
					serviltime(&servil1);
					envio_datos(servil1, '1');
				}
				else if (buffer[0] == '2')
				{
					serviltime(&servil2);
					envio_datos(servil2, '2');
				}
				else if (buffer[0] == '3')
				{
					serviltime(&servil3);
					envio_datos(servil3, '3');
				}
				else if (buffer[0] == '4')
				{
					serviltime(&servil4);
					envio_datos(servil4, '4');
				}
				else if (buffer[0] == 'M')
				{
					if (buffer[1] == '1')
					{
						modisimo = 1;
					}
				}
				else if (buffer[0] == 'P')
				{
					if (buffer[1] == '1')
					{
						estadisimo = 1;
					}
					else if (buffer[1] == '2')
					{
						if (modo != 1)
						{
							habilitar = 1;
						}
					}
				}
				else
				{
					write_str("invalido");
				}				
			}
			
			if (modisimo == 1)
			{
				modisimo = 0;
				modo = 0;
			}
			
			if (estadisimo == 1)
			{
				estadisimo = 0;
				place++;
				if (place == 4)
				{
					place = 0;
				}
			}
			
			ADC_servo1 = servil1;
			ADC_servo2 = servil2;
			ADC_servo3 = servil3;
			ADC_servo4 = servil4;
			
			mapeo_servo(1, ADC_servo1, ADC_servo2, &PWM_1, &PWM_2);
			mapeo_servo(0, ADC_servo1, ADC_servo2, &PWM_1, &PWM_2);
			OCR1A = PWM_1;
			OCR1B = PWM_2;
			
			PWM_3 = map_servo2(ADC_servo3);
			OCR2A = PWM_3; // Setear duty cycle
			
			PWM_4 = map_servo2(ADC_servo4);
			OCR2B = PWM_4; // Setear duty cycle
			
			if (habilitar == 1)
			{
				save_(1, place, ADC_servo1);
				save_(2, place, ADC_servo2);
				save_(3, place, ADC_servo3);
				save_(4, place, ADC_servo4);
				habilitar = 0;
			}
			
			_delay_ms(20);  // Tiempo entre actualizaciones

		}

	}
}

// NON-Interrupt subroutines
// Funciones de Seteo
void setup()
{
	cli();
	DDRB = 0xFF;  // Puerto B como salida
	DDRD |= (1 << PIND3);  // PIND3 como salida
	PORTD |= (1 << PIND2) | (1 << PIND4) | (1 << PIND5);
	DDRC = 0x0F;  // Mitad de puerto C como salida
	
	PCMSK2 |= (1 << PIND2) | (1 << PIND4) | (1 << PIND5);
	PCICR |= (1 << PCIE2);
	
	PWM_init();
	ADC_init();
	UART_init();
	
	sei();
}

void PWM_init()
{
	// Timer 1
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // Modo Fast PWM 14, TOP = ICR1 y prescaler = 8

	ICR1 = 20000;  // Setear Top como 20ms
	
	// Timer 2
	TCCR2A = (1 << COM2A1) | (1 << COM0B1) | (1 << WGM21) | (1 << WGM20); // Configurar Fast PWM, no-inverting mode
	TCCR2B = (1 << CS22) | (1 << CS20); // Prescaler de 

}

void ADC_init()
{
	ADMUX = (1 << REFS0);  // 5V de referencia
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  // prescaler = 64
}

void UART_init()
{
	DDRD |= (1 << DDD1);	// Configuración de pines rx y tx
	DDRD &= ~(1 << DDD0);
	UCSR0A = 0;				// Configuración del serial
	UCSR0B |= (1 << RXCIE0) | (1<< RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	
	UBRR0 = 103;	// BAUD RATE a 9600
}

uint16_t ADC_read(uint8_t PIN)
{
	ADMUX = (ADMUX & 0xF0) | (PIN & 0x0F);	// Selecciona pin de lectura
	ADCSRA |= (1 << ADSC);                   // Inicia conversión
	while (ADCSRA & (1 << ADSC));            // Espera a que termine
	return ADC;                              //	Devuelve valor (10 bits)
}

void mapeo_servo(uint8_t PWM_select, uint16_t ADC_servo1, uint16_t ADC_servo2, uint16_t* PWM_1, uint16_t* PWM_2)
{
	if ((ADC_servo1 > 450) & (ADC_servo1 < 560))
	{
		(*PWM_1) = (490 * 4000UL) / 1023 + 1000;
	}
	else if (ADC_servo1 < 450)
	{
		(*PWM_1) = (570 * 4000UL) / 1023 + 1000;
	}
	else if (ADC_servo1 > 560)
	{
		(*PWM_1) = (380 * 4000UL) / 1023 + 1000;
	}
	if ((ADC_servo2 > 450) & (ADC_servo2 < 560))
	{
		(*PWM_2) = (580 * 4000UL) / 1023 + 1000;
	}
	else if (ADC_servo2 < 450)
	{
		(*PWM_2) = (450 * 4000UL) / 1023 + 1000;	
	}
	else if (ADC_servo2 > 560)
	{
		(*PWM_2) = (700 * 4000UL) / 1023 + 1000;
	}
	
}

uint16_t map_servo2(uint16_t ADC_need)
{
	uint16_t valor = ((ADC_need * 200UL / 1023) + 25);
	return valor;
}

void writeEEPROM(uint16_t direccion, uint8_t dato) 
{
	while (EECR & (1 << EEPE));
	EEAR = direccion;
	EEDR = dato;
	EECR |= (1 << EEMPE);
	EECR |= (1 << EEPE);
}

uint8_t read_EEPROM(uint16_t direccion)
{
	while (EECR & (1 << EEPE));
	EEAR = direccion;
	EECR |= (1 << EERE);
	return EEDR;
}

void save_(uint8_t servo, uint8_t posicion, uint16_t dato)
{
	uint8_t temporal = (posicion * 8) + (servo * 2);
	writeEEPROM(temporal, (dato >> 8));
	temporal++;
	writeEEPROM(temporal, (dato & 0xFF));
}

uint8_t load_(uint8_t servo, uint8_t posicion, uint8_t punch)
{
	uint8_t temporal = (posicion * 8) + (servo * 2) + punch;
	return read_EEPROM(temporal);
}

void led_on(uint8_t modo, uint8_t place)
{
	if (modo == 1)
	{
		PORTB |= (1 << PORTB5);
		PORTB &= ~(1 << PORTB4);
	}
	else if (modo == 2)
	{
		PORTB |= (1 << PORTB4);
		PORTB &= ~(1 << PORTB5);
	}
	else
	{
		PORTB &= ~(1 << PORTB5);
		PORTB &= ~(1 << PORTB4);
	}
	
	// Leds para posición
	if (place == 1)
	{
		PORTC |= (1 << PORTC1);
		PORTC &= ~(1 << PORTC0);
		PORTC &= ~(1 << PORTC2);
		PORTC &= ~(1 << PORTC3);
	}
	else if (place == 2)
	{
		PORTC |= (1 << PORTC2);
		PORTC &= ~(1 << PORTC0);
		PORTC &= ~(1 << PORTC1);
		PORTC &= ~(1 << PORTC3);
	}
	else if (place == 3)
	{
		PORTC |= (1 << PORTC3);
		PORTC &= ~(1 << PORTC0);
		PORTC &= ~(1 << PORTC2);
		PORTC &= ~(1 << PORTC1);
	}
	else
	{
		PORTC |= (1 << PORTC0);
		PORTC &= ~(1 << PORTC1);
		PORTC &= ~(1 << PORTC2);
		PORTC &= ~(1 << PORTC3);
	}
}

void write_char(char caracter)
{	
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = caracter;
}

void write_str(char* texto)
{
	for(uint8_t i = 0; *(texto+i) != '\0'; i++)
	{
		write_char(*(texto+i));
	}
}

void serviltime(uint16_t* servil)
{
	*(servil) = 0;
	for (uint8_t i = 1; i < largo_buff; i++)
	{
		*(servil) = *(servil) * 10 + (buffer[i] - '0');
	}
	
	if (*(servil) > 1023)
	*(servil) = 1023;
	else if (*(servil) < 139)
	*(servil) = 139;
}

void envio_datos(uint16_t dato, char prefijo)
{	
	uint8_t cambio[4];  // Arreglo de 4 elementos
	cambio[3] = (dato % 10) + '0';              // unidades
	cambio[2] = ((dato / 10) % 10) + '0';       // decenas
	cambio[1] = ((dato / 100) % 10) + '0';      // centenas
	cambio[0] = ((dato / 1000) % 10) + '0';     // millares
	
	write_char(prefijo);
	write_char(cambio[0]);
	write_char(cambio[1]);
	write_char(cambio[2]);
	write_char(cambio[3]);
}

// Interrupt routines
ISR(PCINT2_vect)
{
	cli();
	
	_delay_ms(10);
	// Si el pin está encendido en el pin 2 incrementa
	if (!(PIND & (1 << PORTD2)))
	{
		modisimo = 1;
	}
	
	// Si el pin está encendido en el pin 4 incrementa
	else if (!(PIND & (1 << PORTD4)))
	{
		estadisimo = 1; 
	}
	
	// Si el pin está encendido en el pin 5 decrementa
	else if (!(PIND & (1 << PORTD5)))
	{
		if (modo != 1)
		{
			habilitar = 1;	
		}
	}
	sei();
}

ISR(USART_RX_vect)
{
	cli();
	
	comando = UDR0;
	
	if (comando == ',') // Si la cadena termina en el caracter de "enter" entra al if
	{
		buffer[indice] = '\0';	// Termina el string
		new_data = 1;					// Enciende la bandera de UART
		largo_buff = indice;
		indice = 0;				// Reinicia el índice del buffer
	}
	else
	{
		if (indice < sizeof(buffer) - 1) // Mientras que el índice del caracter recibido sea menor que el tamaño de la lista (Buffer) entra al if
		{
			buffer[indice++] = comando;	// Guarda en la lista el caracter recibido y suma uno al índice de la lista
		}
	}

	
	sei();
}