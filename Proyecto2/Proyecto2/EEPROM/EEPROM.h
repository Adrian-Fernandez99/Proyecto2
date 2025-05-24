/*
 * EEPROM.h
 *
 * Created: 5/23/2025 3:24:15 PM
 *  Author: adria
 */ 


#ifndef EEPROM_H_
#define EEPROM_H_

void writeEEPROM(uint16_t direccion, uint8_t dato);
uint8_t read_EEPROM(uint16_t direccion);
void save_(uint8_t servo, uint8_t posicion, uint16_t dato);
uint8_t load_(uint8_t servo, uint8_t posicion, uint8_t punch);

#endif /* EEPROM_H_ */