#include "GroundStation.h"

GroundStation::GroundStation(Configuration& config) : config(config){
  parityOdd = 0;
  parityEven = 0;
  dataLength = 0;
}

void GroundStation::regByte(byte inByte){
  if(inByte == MESSAGE_HEADER && dataLength > 2) {
    //Last two bytes i parityBytes and should not be included in parity count
    if(data[dataLength-1] % 2 == 0) parityEven--;
    else parityOdd--;
    if(data[dataLength-2] % 2 == 0) parityEven--;
    else parityOdd--;

    //execute valid message
    if(   
        data[dataLength-1] == parityEven &&
        data[dataLength-2] == parityOdd
        ){

      //Remove parityBytes
      dataLength--;
      dataLength--;

      //add valid message to messages
      execute(data);
    } else {
      error("Invalid message received from ground station");
      error((char*) data);
    }

    parityOdd = 0;
    parityEven = 0;
    dataLength = 0;
  } else {
    if(inByte % 2 == 0) parityEven++;
    else parityOdd++;

    data[dataLength] = inByte;
    dataLength++;
  }
}

void GroundStation::log(String message){
	sendString(MESSAGE_TYPE_LOG, message);
}

void GroundStation::warning(String message){
	sendString(MESSAGE_TYPE_WARN, message);
}

void GroundStation::error(String message){
	sendString(MESSAGE_TYPE_ERROR, message);
}

void GroundStation::sendString(byte type, String string){
  Serial.write(MESSAGE_HEADER);
  Serial.write(type);

  Serial.print(string);

  Serial.write(MESSAGE_HEADER);
}

void GroundStation::sendConfig(){
  Serial.write(MESSAGE_HEADER);
  Serial.write(MESSAGE_TYPE_CONFIG);

  for(byte i=0; i < CV_END_BYTE + 1; i++)
    Serial.write(config.get(i));

  Serial.write(MESSAGE_HEADER);
  log("Configuration sendt form tricopter");
}

void GroundStation::execute(byte data[]){
  byte CMD = data[0];

  //remove CMD byte
  for (byte i = 0; i < CV_END_BYTE + 1; i++){
    data[i] = data[i+1];
  }
  dataLength--;

  switch(CMD){
    case MESSAGE_TYPE_CONFIG_REQUEST: //request config
      sendConfig();
      break;
    case MESSAGE_TYPE_CONFIG: //setConfig
      if(dataLength == CV_END_BYTE + 1){
        log("Configuration received by tricopter");
        config.set(data);
        sendConfig(); //Respond by sending updated config
      } else error("Byte count Missmatch");
      break;
  }
}