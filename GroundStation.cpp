#include "GroundStation.h"

GroundStation::GroundStation(Configuration& config) : config(config){
  parityOdd = 0;
  parityEven = 0;
  dataLength = 0;
}

void GroundStation::regByte(byte inByte){
  if(inByte == MESSAGE_HEADER) {
    if(dataLength > 2) {
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
        error("Invalid message received from ground station (parity does not match)");
        error((char*) data);
      }
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
  byte size = string.length();
  char chars[size + 1]; //Dont know why we have to add 1
  string.toCharArray(chars, size + 1); //Dont know why we have to add 1

  byte data[size + 1];
  data[0] = type;

  for(byte i = 0; i < size; i++){
    data[i+1] = chars[i];
  }

  send(data, size + 1);
}

void GroundStation::sendConfig(){
  byte size = CV_END_BYTE + 1;
  byte data[size + 1];
  data[0] = MESSAGE_TYPE_CONFIG;

  for(byte i=0; i < size; i++){
    data[i+1] = config.get(i);
  }

  send(data, size + 1);
}

void GroundStation::send(byte data[], byte size){
  Serial.flush();
  Serial.write(MESSAGE_HEADER);

  byte odd = 0;
  byte even = 0;
  for(byte i=0; i < size; i++){
    if(data[i] == MESSAGE_HEADER) data[i]--; //Not allowed to be the same as header

    if(data[i] % 2 == 0) even++;
    else odd++;
    Serial.write(data[i]);
  }

  Serial.write(odd);
  Serial.write(even);
  Serial.write(MESSAGE_HEADER);
  Serial.flush();
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