#include "GroundStation.h"

GroundStation::GroundStation(
		Configuration& config, 
		SatelliteReceive& receiver, 
		IMURazor& imu, 
		Directions& output, 
		Mixer& mix) : 
			config(config), 
			receiver(receiver), 
			imu(imu), 
			output(output), 
			mix(mix){
				
 	//TODO: refactor:
	inMessage = false;
	receivedByteCount = 0;
}

void GroundStation::regByte(byte inByte){
	receivedByteCount++;

	if(!inMessage && inByte != (byte)'>') { //Not this protocol
		receivedByteCount = 0;
	} 
	else if(inByte == (byte)'>' && receivedByteCount == 3) { //prefix received
		inMessage = true;
		CMD = 0;
	} 
	else if (inMessage) {
		if(receivedByteCount == 4) CMD = inByte;
		else {
			data[receivedByteCount - 5] = inByte;

			if( //End of message buffer full og postfix received
				receivedByteCount - 4 >= MAX_MESSAGE_SIZE ||
				(receivedByteCount >= 7 && 
				data[receivedByteCount - 5] == (byte)'*' && 
				data[receivedByteCount - 6] == (byte)'*' && 
				data[receivedByteCount - 7] == (byte)'*')) {

				inMessage = false;
				receivedByteCount = 0;

				executeCommand(CMD, data);
			}
		}
	}
}

void GroundStation::log(String message){
	sendMessage(LOG_LEVEL_INFO, message);
}

void GroundStation::warning(String message){
	sendMessage(LOG_LEVEL_WARNING, message);
}

void GroundStation::error(String message){
	sendMessage(LOG_LEVEL_ERROR, message);
}

void GroundStation::sendConfig(){
  Serial.print(">>>"); //Prefix
  Serial.write(5); //CMD Tricopter
  for(int i=0; i < CV_END_BYTE + 1; i++) Serial.write(config.get(i));  
  Serial.println("***"); //Suffix
  sendMessage(0, "Configuration sendt form tricopter");
}

void GroundStation::sendCopter(){
  if(bitRead(config.get(CV_TRICOPTER_ENABLE_BYTE), CV_TRIGUI_ENABLE_BIT) == 1){
    //Tricopter
    Serial.print(">>>"); //Prefix
    Serial.write(2); //CMD Tricopter
    
    //--------- Data --------------
    Serial.write(millis() / 10000); //time 0 - ca 45 min
    Serial.write(((receiver.getThro() > config.get(CV_MIN_THRO_BYTE) * 4)? 0x30 : 0x20));
    Serial.write(0);
   
    //Motors and servo
    Serial.write(map(mix.getLeftThrust(), 0, 179, 0, 255));
    Serial.write(map(mix.getRightThrust(), 0, 179, 0, 255));
    Serial.write(map(mix.getRearThrust(), 0, 179, 0, 255));
    Serial.write(map(mix.getYawPos(), 0, 179, 0, 255));
    
    //PID
    Serial.write(((int)output.roll + 1023) / 8);
    Serial.write(((int)output.nick + 1023) / 8);
    Serial.write(((int)output.yaw + 1023) / 8);
    
    Serial.println("***"); //Suffix
  }
}

void GroundStation::sendReceiver(){
  if(bitRead(config.get(CV_TRICOPTER_ENABLE_BYTE), CV_TRIGUI_ENABLE_BIT) == 1){
    Serial.print(">>>"); //Prefix
    Serial.write(3); //CMD receiver
    
    //Signal
    Serial.write(receiver.getThro() / 4);
    Serial.write(receiver.getAile() / 4);
    Serial.write(receiver.getElev() / 4);
    Serial.write(receiver.getRudd() / 4);
    Serial.write(receiver.getGear() / 4);
    Serial.write(receiver.getFlap() / 4);
    
    Serial.println("***"); //Suffix
  }
}

void GroundStation::sendIMU(){
  if(bitRead(config.get(CV_TRICOPTER_ENABLE_BYTE), CV_TRIGUI_ENABLE_BIT) == 1){
    Serial.print(">>>"); //Prefix
    Serial.write(4); //CMD IMU
    
    //Signal
    Serial.write(map(imu.getRollDegree(), -180, 180, 0, 255));
    Serial.write(map(imu.getNickDegree(), -180, 180, 0, 255));
    Serial.write(map(imu.getYawDegree(), -180, 180, 0, 255));
    
    Serial.write(0);//imu.getGyroRoll() / 8 + 127);
    Serial.write(0);//imu.getGyroNick() / 8 + 127);
    Serial.write(0);//imu.getGyroYaw() / 8 + 127);
    
    Serial.write(0);//imu.getAccRoll() / 4);
    Serial.write(0);//imu.getAccNick() / 4);
    Serial.write(0); // AccVert //TODO: not implemented in IMU class
    
    Serial.println("***"); //Suffix
  }
}

/**
 * Transfere attitude data compatible with Happy Killmore ground station
 * 
 * Only if HK_ENABLED = 1
 */
void GroundStation::happyKillmoreSendAttitude(){
  if(bitRead(config.get(CV_TRICOPTER_ENABLE_BYTE), CV_HK_ENABLE_BIT) == 1){
    Serial.print("+++"); //Prefix
    Serial.print("ASP:"); //Airspeed
    Serial.print(20, DEC);
    Serial.print(",THH:"); //Throttle
    Serial.print(map(receiver.getThro(),153,862,0,100), DEC);
    Serial.print (",RLL:"); //Roll
    Serial.print(imu.getRollDegree());//(analogRead(ACC_ROLL_PIN) - init_acc_roll) * acc_scale, DEC);
    Serial.print (",PCH:"); //Pitch
    Serial.print(imu.getNickDegree());
    Serial.println(",***"); //Suffix
  }
}

/**
 * Transfere location data compatible with Happy Killmore ground station
 * 
 * Only if HK_ENABLED = 1
 */
void GroundStation::happyKillmoreSendLocation(){
  if(bitRead(config.get(CV_TRICOPTER_ENABLE_BYTE), CV_HK_ENABLE_BIT) == 1){
    Serial.print("!!!"); //Prefix
    Serial.print("LAT:"); //Latitude
    Serial.print("67967300");//"33952600");
    Serial.print(",LON:"); //Longitude
    Serial.print("14994700");//"-117409072");
    Serial.print(",SPD:"); //Speed over ground from GPS
    Serial.print(30);
    Serial.print(",CRT:"); //Climb rate m/s
    Serial.print("100");
    Serial.print(",ALT:"); //Altitude in meters
    Serial.print(1000);
    Serial.print(",ALH:"); //Target Altitude
    Serial.print("450");
    Serial.print(",CRS:"); //Course over ground in degrees
    Serial.print(imu.getYawDegree()); 
    Serial.print(",BER:"); //Bearing (target heading)
    Serial.print("94");
    Serial.print(",WPN:"); //Waypoint number
    Serial.print("0");
    Serial.print(",DST:"); //Distance from Waypoint
    Serial.print("25853");
    Serial.print(",BTV:"); //Battery voltage
    Serial.print("11840");
    Serial.println(",***"); //suffix
  }
}



void GroundStation::sendMessage(byte type, String message){
    Serial.print(">>>"); //Prefix
    Serial.write(1); //CMD message
    Serial.write(type); //type
    Serial.print(message);
    Serial.println("***");
}

void GroundStation::executeCommand(byte CMD, byte data[]){
  switch(CMD){
    case 5: //request config
      sendConfig();
      break;
    case 6: //setConfig
      if(receivedByteCount - 4 == CV_END_BYTE + 1){
        log("Configuration received by tricopter");
        config.set(data);
      } else error("Byte count Missmatch");
      break;
  }
}