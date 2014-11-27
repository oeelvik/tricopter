#include "Motors.h"

Motors::Motors(){}

void Motors::init(){
	DDRB = DDRB | B00001110;                                  // Set ports to output PB1-3
	DDRD = DDRD | B00001000;                                  // Set port to output PD3

	byte init[4] = {0,0,0,0};
	command(init);

	// Init PWM Timer 1  16 bit
	TCCR1A = (1<<WGM10)|(1<<COM1A1)|(1<<COM1B1);
	TCCR1B = (1<<WGM12)|(1<<CS12);
	ICR1 = 256;
	// Init PWM Timer 2   8bit                                 // WGMn1 WGMn2 = Mode ? Fast PWM, TOP = 0xFF ,Update of OCRnx at BOTTOM
	TCCR2A = (1<<WGM20)|(1<<WGM21)|(1<<COM2A1)|(1<<COM2B1);    // Clear OCnA/OCnB on compare match, set OCnA/OCnB at BOTTOM (non-inverting mode)
	TCCR2B = (1<<CS22)|(1<<CS21);                              // Prescaler set to 256, that gives us a resolution of 16us
	// TOP is fixed at 255                                     // Output_PWM_Frequency = 244hz = 16000000/(256*(1+255)) = Clock_Speed / (Prescaler * (1 + TOP))

}

/**
 * command between 0-250 
 * {pin 9, pin 10, pin 11, pin 3}
 **/
void Motors::command(byte command[]){
	OCR1A = (1000 + (command[0] * 4)) / 16;
	OCR1B = (1000 + (command[1] * 4)) / 16;
	OCR2A = (1000 + (command[2] * 4)) / 16;
	OCR2B = (1000 + (command[3] * 4)) / 16;
}