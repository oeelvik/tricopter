/*
 * Configuration.h
 *
 * author: Øystein Schrøder Elvik
 */
#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "ConfigAdressing.h"

class Configuration {
public:
	Configuration(int eepromStart);
	byte get(byte adress);
	void set(byte conf[]);
	void reset();

private:
	byte config[CV_END_BYTE + 1];
	int eepromStart;
	
	void readEEPROM();
	void writeEEPROM();
};

#endif /* CONFIGURATION_H_ */