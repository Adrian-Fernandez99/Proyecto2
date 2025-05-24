/*
 * EEPROM.c
 *
 * Created: 5/23/2025 3:20:56 PM
 *  Author: adria
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

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
