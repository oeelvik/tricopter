/**
 * DataStream.h
 *
 * @author Øystein Schrøder Elvik
 * @version 1.0
 * @since 28.11.2014
 * 
 */

#ifndef DATASTREAM_H_
#define DATASTREAM_H_

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#define MAX_MESSAGE_SIZE 50
#define HEADER 255

typedef byte (*datasource)();

class DataStream {
public:
	DataStream() : messageSize(0), next(0) {};
	void begin(int baudrate); //Not nesseserry if Serial.begin is already called
	void attach(datasource datasource);
	void send(); //Send one byte
private:
	datasource datasources[MAX_MESSAGE_SIZE];
	byte messageSize;
	byte next;
	byte oddsParityByte;
	byte evenParityByte;
};


#endif /* DATASTREAM_H_ */