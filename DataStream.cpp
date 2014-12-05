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
		payload = MESSAGE_HEADER;
		next++;
	}
	else if(next == 1) {
		payload = MESSAGE_TYPE_DATA;
		next++;
	} else if (next < messageSize + 2) {
		payload = datasources[next-2]();
		if (payload == MESSAGE_HEADER) payload = MESSAGE_HEADER-1; //Header is reserved for header only
		next++;
		if(payload % 2 == 0) evenParityByte++;
		else oddsParityByte++;
	} else if(next == messageSize + 2) {
		payload = oddsParityByte;
		next++;
	} else if(next == messageSize + 3) {
		payload = evenParityByte;
		next = 0;
	}
	Serial.write(payload);
}