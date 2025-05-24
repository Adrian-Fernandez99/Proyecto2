/*
 * PWM.h
 *
 * Created: 5/23/2025 12:56:18 PM
 *  Author: adria
 */ 


#ifndef PWM_H_
#define PWM_H_

void mapeo_servo(uint8_t PWM_select, uint16_t ADC_servo1, uint16_t ADC_servo2, uint16_t* PWM_1, uint16_t* PWM_2);
uint16_t map_servo2(uint16_t ADC_need, uint8_t cosito);

#endif /* PWM_H_ */