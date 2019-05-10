/************************************************************************
 * Created:     25/11/2018
 * Author:      mk28
 * Description: Automated water pump control system. For Atmega328P
 ************************************************************************/
/* LCD PINS:
 * 16|15|14|13|12|11|10|09|08|07|06|05|04|03|02|01
 * 0V 5V D7 D6 D5 D4             EN 0V RS PR 5V 0V */

/* Ignore below 4 lines */
#define D0 eS_PORTD0
#define D1 eS_PORTD1
#define D2 eS_PORTD2
#define D3 eS_PORTD3
/* This are important */
#define RS eS_PORTD0
#define EN eS_PORTD1

#define D4 eS_PORTD2
#define D5 eS_PORTD3
#define D6 eS_PORTD4
#define D7 eS_PORTD5

/* Constants */
#define SET_POINT_BAT_L 65
#define SET_POINT_BAT_H 75
#define SET_POINT_SUN_L 55
#define SET_POINT_SUN_H 65
/* Self-commenting Macros */
#define SW_PRESSED ~(PINB&(1<<PINB0))
#define TURN_PUMP_ON PORTD |= (1<<PORTD7)
#define TURN_PUMP_OFF PORTD &= ~(1<<PORTD7)
#define TOGGLE_LED PIND |= (1<<PORTD7)
/* Clock of Atmega328P */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "lcd.h" 

double dutyCycle = 0;
int adc_i = 0;
uint16_t lightValue;
uint16_t batteryValue;

void setupLCD() {
	DDRD |= (1<<PORTD5) | (1<<PORTD4) | (1<<PORTD3) | (1<<PORTD2); // 4-bit data out for LCD
	DDRD |= (1<<PORTD1) | (1<<PORTD0); // RS & EN out for LCD
	
	Lcd4_Init();
	Lcd4_Clear();
}
void setupT1LED() {
	DDRD |= (1<<PORTD7); // Set PIN7 as output for LED Green - Pump ON
	/* Waveform Generation Mode set to CTC, with TOP = OCR1A and TOV1 set on MAX */
	TCCR1B = (1<<WGM12); // TC1 Control Register B 
	/* Set Clock / 1024 (from prescaler) */
	TCCR1B |= (1<<CS12) | (1<<CS10);
	/* OCIE1A: Timer/Counter Output Compare A Match interrupt is enabled */
	TIMSK1 = (1<<OCIE1A); // Timer/Counter 1 Interrupt Mask Register
	OCR1A = 15625; // Output Compare Register 1 A Low byte
}

void startConversion() {
	ADCSRA |= (1<<ADSC); // ADC Control and Status Register A: Start conversion
}

void setupADC() {
	/* REFSn: Reference Selection -> AVcc w external cap. as ADC Voltage Reference,
	 * MUXn: Analog Channel Selection -> Set ADC5 as Analogue Input */
	ADMUX  = (1<<REFS0) | (1<<MUX0) | (1<<MUX2); // ADC Multiplexer Selection Register
	/* ADEN: Enable ADC, 
	 * ADIE: ADC Interrupt Enable -> Activate ADC Conversion Complete Interrupt,
	 * ADPSn: ADC Prescaler Select -> Set division Factor as 128 */
	ADCSRA = (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // ADC Control and Status Register A
	/* Disable Digital Input on ADC4 & ADC5 -> PIN A4 & A5 */ 
	DIDR0  = (1<<ADC5D) | (1<<ADC4D); // Digital Input Disable Register 0

	startConversion();
}

void setupPWM(){
	DDRD |= (1<<PORTD6); // LED dimmed by Anal
	/* Clear OCxA on Compare Match, Set OCxA at BOTTOM (non-inverting mode)
	 * Fast PWM, update OCRxy at BOTTOM, TOV set at MAX */
	TCCR0A = (1<<COM0A1) | (1<<WGM01) | (1<<WGM00); // TCx Control Register
	TIMSK0 = (1<<TOIE0); // TCx Interrupt Mask Register = Overflow Interrupt Enabled
			
	TCCR0B = (1<<CS02) | (1<<CS00); // clock select / 1024 (from prescaler)
}
void setupSwitch(){
	DDRB &= ~(1<<PORTB0) ; // Switch - tank full = ON
	PCMSK0 |= (1<<PCINT0); // Pin Change Mask Register 0
	PCIFR |= (1<<PCIE0);
	PCICR |= (1<<PCIE0); // Pin Change Interrupt Control Register
}

int main(void) {
	setupLCD(); // Set-up LCD for 4-bit mode 
	
	setupSwitch(); // set-up the switch (Pin Change Interrupt - Water level limit)

	setupT1LED(); // Set up the blinking (Timer 1 Comparison interrupt) green LED (Pump ON) 

	setupADC(); // Set-up analog inputs for photoresistor (Sun light level) and the potentiometer (battery level)

	setupPWM(); // Set-up the adjustable  (PWM Timer 0 overflow interrupt) red LED (battery level)
	
	sei(); // Enable interrupts
	
	while (1) {	} // do nothing forever
	
}

ISR(TIMER0_OVF_vect) {	
	OCR0A = dutyCycle; // TC0 Output Compare Register A: Update PWM
}

ISR(ADC_vect) {
	uint8_t tmp_sp_h, tmp_sp_l;
	uint16_t tempVal; 
	/* Update one analog input at a time */ 
	lightValue = adc_i ? ADC/10.2255 : lightValue;
	batteryValue = adc_i ? batteryValue : ADC/10.2255;
	dutyCycle = batteryValue; // Update dutycycle for PWM
	adc_i = adc_i ? 0 : 1;
	ADMUX = (ADMUX & 0xF0) | (adc_i ? 4 : 5); // ADC Multiplexer Selection Register: Set PIN A4 or A5 as Analogue Input
	startConversion();
	Lcd4_Set_Cursor(adc_i,0);
	char s[8];
	tempVal = adc_i ? lightValue : batteryValue;
	tmp_sp_h = adc_i ? SET_POINT_SUN_H : SET_POINT_BAT_H;
	tmp_sp_l = adc_i ? SET_POINT_SUN_L : SET_POINT_BAT_L;
	sprintf(s, "%s %3d%% %s", adc_i ? "SUN" : "BAT", (int)tempVal, tempVal>tmp_sp_h ? "Good   " : tempVal>tmp_sp_l ? "Low    " : "Alarm  ");
	Lcd4_Write_String(s);
}
ISR(PCINT0_vect) {
	/* turn the Pump OFF immediately once the water reach the FULL setpoint */
	if (SW_PRESSED) {
		TURN_PUMP_OFF;
	}
}
ISR(TIMER1_COMPA_vect) {
	/* If enough sun light and enough battery and the limit switch not engaged turn the pump ON */ 
	if ((lightValue>SET_POINT_SUN_L)&&(batteryValue>SET_POINT_BAT_L)&(SW_PRESSED)) {
		TOGGLE_LED;
	} else {
		TURN_PUMP_OFF;
	}
}
