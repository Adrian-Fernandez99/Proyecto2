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

uint16_t ADC_read(uint8_t PIN);

uint16_t ADC_servo1 = 0;
uint16_t ADC_servo2 = 0;
uint16_t ADC_servo3 = 0;
uint16_t ADC_servo4 = 0;

uint16_t PWM_1 = 0;
uint16_t PWM_2 = 0;
uint8_t PWM_3 = 0;
uint8_t PWM_4 = 0;

// MAIN LOOP
int main(void)
{
	setup();
	while (1)
	{	
		ADC_servo1 = ADC_read(6);
		ADC_servo2 = ADC_read(7);
		
		mapeo_servo(1, ADC_servo1, ADC_servo2, &PWM_1, &PWM_2);
		mapeo_servo(0, ADC_servo1, ADC_servo2, &PWM_1, &PWM_2);
		
		OCR1A = PWM_1;
		OCR1B = PWM_2;
		
		ADC_servo3 = ADC_read(4);
		PWM_3 = ((ADC_servo3 * 200UL / 1023) + 25);
		
		OCR2A = PWM_3; // Setear duty cycle
		
		ADC_servo4 = ADC_read(5);
		PWM_4 = ((ADC_servo4 * 200UL / 1023) + 25);
		
		OCR2B = PWM_4; // Setear duty cycle

		_delay_ms(20);  // Tiempo entre actualizaciones

	}
}

// NON-Interrupt subroutines
// Funciones de Seteo
void setup()
{
	cli();
	DDRB |= (1 << PINB1) | (1 << PINB2) | (1 << PINB3) | (1 << PINB4);  // D9 y D10 como salida
	DDRD |= (1 << PIND3);  // D9 y D10 como salida
	
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

// Interrupt routines
ISR(TIMER0_OVF_vect)
{
	cli();
	
	sei();
}