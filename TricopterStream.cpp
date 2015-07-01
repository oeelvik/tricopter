#include "TricopterStream.h"

SatelliteReceive *TricopterStream::receiver = 0;
Directions *TricopterStream::setPoint = 0;
IMURazor *TricopterStream::imu = 0;
Directions *TricopterStream::output = 0;
Mixer *TricopterStream::mix = 0;


void TricopterStream::attachAll(DataStream *dataStream){
	dataStream->attach(&type);
	dataStream->attach(&time);
	dataStream->attach(&receiverThrottle);
	dataStream->attach(&receiverAileron);
	dataStream->attach(&receiverElevator);
	dataStream->attach(&receiverRudder);
	dataStream->attach(&receiverGear);
	dataStream->attach(&receiverFlaps);
	dataStream->attach(&setPointVertical);
	dataStream->attach(&setPointRoll);
	dataStream->attach(&setPointNick);
	dataStream->attach(&setPointYaw);
	dataStream->attach(&imuRollDegree);
	dataStream->attach(&imuNickDegree);
	dataStream->attach(&imuYawDegree);
	dataStream->attach(&imuRollRotation); 
	dataStream->attach(&imuNickRotation);
	dataStream->attach(&imuYawRotation);
	dataStream->attach(&imuAccelerationX);
	dataStream->attach(&imuAccelerationY);
	dataStream->attach(&imuAccelerationZ);
	dataStream->attach(&outputVertical);
	dataStream->attach(&outputRoll);
	dataStream->attach(&outputNick);
	dataStream->attach(&outputYaw);
	dataStream->attach(&mixLeftThrust);
	dataStream->attach(&mixRightThrust);
	dataStream->attach(&mixRearThrust);
	dataStream->attach(&mixYawPos); 
	//29
}

byte TricopterStream::type(){
	return MESSAGE_TYPE_DATA;
}

byte TricopterStream::time(){
	return millis() / 10000;
}
	
	//receiver
byte TricopterStream::receiverThrottle(){
	return map(receiver->getThro(), 0, 1023, 0, 254);
}

byte TricopterStream::receiverAileron(){
	return map(receiver->getAile(), 0, 1023, 0, 254);
}

byte TricopterStream::receiverElevator(){
	return map(receiver->getElev(), 0, 1023, 0, 254);
}

byte TricopterStream::receiverRudder(){
	return map(receiver->getRudd(), 0, 1023, 0, 254);
}

byte TricopterStream::receiverGear(){
	return map(receiver->getGear(), 0, 1023, 0, 254);
}

byte TricopterStream::receiverFlaps(){
	return map(receiver->getFlap(), 0, 1023, 0, 254);
}


//Setpoint
byte TricopterStream::setPointVertical(){
	return map(setPoint->vertical, 0, 1023, 0, 254);
}

byte TricopterStream::setPointRoll(){
	return map(setPoint->roll, 0, 1023, 0, 254);
}

byte TricopterStream::setPointNick(){
	return map(setPoint->nick, 0, 1023, 0, 254);
}

byte TricopterStream::setPointYaw(){
	return map(setPoint->yaw, 0, 1023, 0, 254);
}

    
//IMU
byte TricopterStream::imuRollDegree(){
	return map(imu->getRollDegree(), -180, 180, 0, 254);
}

byte TricopterStream::imuNickDegree(){
	return map(imu->getNickDegree(), -180, 180, 0, 254);
}

byte TricopterStream::imuYawDegree(){
	return map(imu->getYawDegree(), -180, 180, 0, 254);
}

byte TricopterStream::imuRollRotation(){
	return map(imu->getRollRotation() * 10000, -600, 600, 0, 254);
}

byte TricopterStream::imuNickRotation(){
	return map(imu->getNickRotation() * 10000, -600, 600, 0, 254);
}

byte TricopterStream::imuYawRotation(){
	return map(imu->getYawRotation() * 10000, -600, 600, 0, 254);
}

byte TricopterStream::imuAccelerationX(){
	return map(imu->getAccelerationX() * 20, -100, 100, 0, 254);
}

byte TricopterStream::imuAccelerationY(){
	return map(imu->getAccelerationY() * 20, -100, 100, 0, 254);
}

byte TricopterStream::imuAccelerationZ(){
	return map(imu->getAccelerationZ() * 20, -100, 100, 0, 254);
}

    
//PID out
byte TricopterStream::outputVertical(){
	return map(output->vertical, 0, 1023, 0, 254);
}

byte TricopterStream::outputRoll(){
	return map(output->roll, -1023, 1023, 0, 254);
}

byte TricopterStream::outputNick(){
	return map(output->nick, -1023, 1023, 0, 254);
}

byte TricopterStream::outputYaw(){
	return map(output->yaw, -1023, 1023, 0, 254);
}


//Motors
byte TricopterStream::mixLeftThrust(){
	return map(mix->getLeftThrust(), 0, 179, 0, 254);
}

byte TricopterStream::mixRightThrust(){
	return map(mix->getRightThrust(), 0, 179, 0, 254);
}

byte TricopterStream::mixRearThrust(){
	return map(mix->getRearThrust(), 0, 179, 0, 254);
}

byte TricopterStream::mixYawPos(){
	return map(mix->getYawPos(), 0, 179, 0, 254);
}


//############## HappyKillmore ##############
void TricopterStream::attachHK(DataStream *dataStream){
	dataStream->attach(&HKAttitudePre);
	dataStream->attach(&HKAttitudePre);
	dataStream->attach(&HKAttitudePre);

	dataStream->attach(&HK_T);
	dataStream->attach(&HK_H);
	dataStream->attach(&HK_H);
	dataStream->attach(&HK_Colon);
	dataStream->attach(&HKThro);

	dataStream->attach(&HK_Comma);
	dataStream->attach(&HK_R);
	dataStream->attach(&HK_L);
	dataStream->attach(&HK_L);
	dataStream->attach(&HK_Colon);
	dataStream->attach(&HKRoll);

	dataStream->attach(&HK_Comma);
	dataStream->attach(&HK_P);
	dataStream->attach(&HK_C);
	dataStream->attach(&HK_H);
	dataStream->attach(&HK_Colon);
	dataStream->attach(&HKNick);

	dataStream->attach(&HKPost);
	dataStream->attach(&HKPost);
	dataStream->attach(&HKPost);
}

byte TricopterStream::HKAttitudePre(){
	return '+';
}

byte TricopterStream::HKLocationPre(){
	return '!';
}

byte TricopterStream::HK_C(){
	return 'C';
}

byte TricopterStream::HK_H(){
	return 'H';
}

byte TricopterStream::HK_L(){
	return 'L';
}

byte TricopterStream::HK_P(){
	return 'P';
}

byte TricopterStream::HK_R(){
	return 'R';
}

byte TricopterStream::HK_S(){
	return 'S';
}

byte TricopterStream::HK_T(){
	return 'T';
}

byte TricopterStream::HK_Colon(){
	return ':';
}

byte TricopterStream::HK_Comma(){
	return ',';
}

byte TricopterStream::HKPost(){
	return '*';
}

byte TricopterStream::HKThro(){
	return map(receiver->getThro(),153,862,0,100);
}

byte TricopterStream::HKRoll(){
	return imu->getRollDegree();
}

byte TricopterStream::HKNick(){
	return imu->getNickDegree();
}

byte TricopterStream::HKYaw(){
	return imu->getYawDegree();
}
