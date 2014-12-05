/*
 * GroundStation.h
 *
 * author: Øystein Schrøder Elvik
 */
#ifndef GROUNDSTATION_H_
#define GROUNDSTATION_H_

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Configuration.h>


#define MESSAGE_MAX_SIZE 50

#define MESSAGE_HEADER 255

#define MESSAGE_TYPE_LOG 0
#define MESSAGE_TYPE_INFO 1
#define MESSAGE_TYPE_WARN 2
#define MESSAGE_TYPE_ERROR 3
#define MESSAGE_TYPE_CONFIG_REQUEST 9
#define MESSAGE_TYPE_CONFIG 10
#define MESSAGE_TYPE_DATA 100


class GroundStation {
public:
	GroundStation(Configuration& config);

	void regByte(byte inByte);

	void log(String message);
	void warning(String message);
	void error(String message);

private:

	byte data[MESSAGE_MAX_SIZE];
	byte dataLength;
	byte parityOdd;
	byte parityEven;

	Configuration& config;

	void sendString(byte type, String string);
	void sendConfig();
	void execute(byte data[]);
};


#endif /* GROUNDSTATION_H_ */