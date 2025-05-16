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

// Prototipos de función
void setup();
void PWM_init();
void ADC_init();

void mapeo_servo(uint8_t PWM_select, uint16_t ADC_servo1, uint16_t ADC_servo2, uint16_t* PWM_1, uint16_t* PWM_2);

void writeEEPROM(uint16_t direccion, uint8_t dato);
uint8_t read_EEPROM(uint16_t direccion);

uint16_t ADC_read(uint8_t PIN);

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

// MAIN LOOP
int main(void)
{
	setup();
	while (1)
	{	
		if (modo == 0)
		{
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

			_delay_ms(20);  // Tiempo entre actualizaciones
		}
		
		else if (modo == 1)
		{
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

void TMR0_init()
{
	
}

uint16_t ADC_read(uint8_t PIN)
{
	ADMUX = (ADMUX & 0xF0) | (PIN & 0x0F);	// Selecciona pin de lectura
	ADCSRA |= (1 << ADSC);                   // Inicia conversión
	while (ADCSRA & (1 << ADSC));            // Espera a que termine
	return ADC;                              // Devuelve valor (10 bits)
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
	temporal + 1;
	writeEEPROM(temporal, (dato & 0xFF));
}

uint8_t load_(uint8_t servo, uint8_t posicion, uint8_t punch)
{
	uint8_t temporal = (posicion * 8) + (servo * 2) + punch;
	return read_EEPROM(temporal);
}

// Interrupt routines
ISR(TIMER0_OVF_vect)
{
	cli();
		
	sei();
}

ISR(PCINT2_vect)
{
	cli();
	// Si el pin está encendido en el pin 2 incrementa
	if (!(PIND & (1 << PORTD2)))
	{
		modo++;
		if (modo == 4)
		{
			modo = 0;
		}
	}
	// Si el pin está encendido en el pin 4 incrementa
	else if (!(PIND & (1 << PORTD4)))
	{
		
	}
	// Si el pin está encendido en el pin 5 decrementa
	else if (!(PIND & (1 << PORTD5)))
	{
		
	}
	sei();
}