/*
 * USART.c
 *
 * Created: 5/23/2025 12:56:30 PM
 *  Author: adria
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

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