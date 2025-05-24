/*
 * PWM.c
 *
 * Created: 5/23/2025 12:56:10 PM
 *  Author: adria
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

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

uint16_t map_servo2(uint16_t ADC_need, uint8_t cosito)
{
	if (cosito == 0)
	{
		uint16_t valor = ((ADC_need * 200UL / 1023) + 25);
		return valor;
	}
	else
	{
		uint16_t valor = ((ADC_need * 90UL / 1023) + 25);
		return valor;
	}
}