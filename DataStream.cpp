#include "DataStream.h"

void DataStream::begin(int baudrate){
	Serial.begin(baudrate);
}

void DataStream::attach(datasource datasource){
	datasources[messageSize] = datasource;
	messageSize++;
}

void DataStream::send(){
	sendNextByte();
}

void DataStream::send(byte numberOfBytes){
	for(byte i = 0; i < numberOfBytes; i++){
		sendNextByte();
	}
}

void DataStream::flush(){
	while(next != 0) sendNextByte();
}

void DataStream::sendNextByte(){
	byte payload;
	if(next == 0) {
		payload = HEADER;
		next++;
	} else if (next <= messageSize) {
		payload = datasources[next-1]();
		if (payload == HEADER) payload = HEADER-1; //Header is reserved for header only
		next++;
		if(payload % 2 == 0) evenParityByte++;
		else oddsParityByte++;
	} else if(next == messageSize + 1) {
		payload = oddsParityByte;
		next++;
	} else if(next == messageSize + 2) {
		payload = evenParityByte;
		next = 0;
	}
	Serial.write(payload);
}