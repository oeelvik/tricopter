#include "Motors.h"

Motors::Motors(){}

void Motors::init(int motors){
	this->motors = motors;

    #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    	DDRE = DDRE | B00111000;							// Set ports to output PE3-5, OC3A, OC3B, OC3C
    	if(motors > 7) {
    		DDRH = DDRH | B00111000;                                // Set ports to output PH3-5, OC4A, OC4B, OC4C
			DDRB = DDRB | B01100000;								  // OC1A, OC1B
    	}
    	else if (motors > 5) {
    		DDRH = DDRH | B00111000;                                // Set ports to output PH3-5, OC4A, OC4B, OC4C
    	}
    	else if (motors > 3) {
    		DDRH = DDRH | B00001000;                                // Set port to output PH3, OC4A
    	}
    #else
    	DDRB = DDRB | B00001110;                                  // Set ports to output PB1-3
		DDRD = DDRD | B00001000;                                  // Set port to output PD3
    #endif


	byte init[8] = {0,0,0,0,0,0,0,0};
	command(init);

	#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
		// Init PWM Timer 3                                       // WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTTOM
		TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mod
		TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);                 // Prescaler set to 8, that gives us a resolution of 0.5us
		ICR3 = PWM_COUNTER_PERIOD;                                // Clock_speed / ( Prescaler * desired_PWM_Frequency) #defined above.
		if(motors > 7) {
			// Init PWM Timer 4
			TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
			TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
			ICR4 = PWM_COUNTER_PERIOD;    		

			// Init PWM Timer 1
			TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
			TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
			ICR1 = PWM_COUNTER_PERIOD;

    	}
    	else if (motors > 5) {
			// Init PWM Timer 4
			TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
			TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
			ICR4 = PWM_COUNTER_PERIOD;
    	}
    	else if (motors > 3) {
				// Init PWM Timer 4
			TCCR4A = (1<<WGM41)|(1<<COM4A1);
			TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
			ICR4 = PWM_COUNTER_PERIOD;
    	}
	#else
		// Init PWM Timer 1  16 bit
		TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
		TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
		ICR1 = PWM_COUNTER_PERIOD;
		// Init PWM Timer 2   8bit                                 // WGMn1 WGMn2 = Mode ? Fast PWM, TOP = 0xFF ,Update of OCRnx at BOTTOM
		TCCR2A = (1<<WGM20)|(1<<WGM21)|(1<<COM2A1)|(1<<COM2B1);    // Clear OCnA/OCnB on compare match, set OCnA/OCnB at BOTTOM (non-inverting mode)
		TCCR2B = (1<<CS22)|(1<<CS21);                              // Prescaler set to 256, that gives us a resolution of 16us
		// TOP is fixed at 255  
	#endif
}

/**
 * command between 0-250 
 * {pin 9, pin 10, pin 11, pin 3}
 **/
void Motors::command(byte command[]){
    #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    	OCR3B = 2000 + (command[0] * 8 ) ;
		OCR3C = 2000 + (command[1] * 8 ) ;
		OCR3A = 2000 + (command[2] * 8 ) ;
		OCR4A = 2000 + (command[3] * 8 ) ;
		if(motors > 7) {
			OCR4B = 2000 + (command[4] * 8 ) ;
			OCR4C = 2000 + (command[5] * 8 ) ;
    		OCR1A = 2000 + (command[6] * 8 ) ;
			OCR1B = 2000 + (command[7] * 8 ) ;
    	}
    	else if (motors > 5) {
    		OCR4B = 2000 + (command[4] * 8 ) ;
			OCR4C = 2000 + (command[5] * 8 ) ;
    	}
    #else
    	OCR2B = (250 + command[0] ) / 4 ;
		OCR1A = 2000 + (command[1] * 8 ) ;
		OCR1B = 2000 + (command[2] * 8 ) ;
		OCR2A = (250 + command[3] ) / 4 ;
    #endif
}