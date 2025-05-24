/*
 * USART.h
 *
 * Created: 5/23/2025 12:57:34 PM
 *  Author: adria
 */ 


#ifndef USART_H_
#define USART_H_

void write_char(char caracter);
void write_str(char* texto);
void serviltime(uint16_t* servil);
void envio_datos(uint16_t dato, char prefijo);

#endif /* USART_H_ */