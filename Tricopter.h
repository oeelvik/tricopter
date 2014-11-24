/*
 * Tricopter.h
 *
 * author: Øystein Schrøder Elvik
 */
#ifndef TRICOPTER_H_
#define TRICOPTER_H_

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <SatelliteReceive.h>
#include <IMURazor.h>
#include <StopWatch.h> //TODO:remove after benchmark test

#include <Configuration.h>
#include <Mixer.h>
#include <PID.h>

//Should only be set to 1 for testing purpose
#define RESET_CONFIG 0 //Set to 1 to loade default config and save it to eeprom on startup


#define MODE_POSITION 0 //Hold position GPS and Compass ass setpoint Receiver to alter desired position
#define MODE_HOVER 1 //IMU stabilized Receiver setpoint
#define MODE_STUNT 2 //GYRO stabilized Receiver setpoint

#define STATE_OFF 0x00
#define STATE_CONFIG 0x20
#define STATE_READY 0x40
#define STATE_AIRBORNE 0x60
#define STATE_ERROR 0xA0

struct Directions {
  int throttle;
  int roll;
  int nick;
  int yaw;
  };

//TODO: refactor
//---------------- TriGUI ------------- 
#define TRIGUI_MESSAGE_TYPE_INFO 0
#define TRIGUI_MESSAGE_TYPE_WARNING 1
#define TRIGUI_MESSAGE_TYPE_ERROR 2


class Tricopter {
public:
	Tricopter();
	void init();

	void reconfigure();

	void fastLoop();
	void mediumLoop();
	void slowLoop();

	void setState(byte state);
	void updateSetPoints();

	//TODO: refactor
	void gsReceive(byte inByte);
	void gsCallReceiver();
	void HappyKillmoreSendAttitude();
	void HappyKillmoreSendLocation();
	void TriGUIsendMessage(int type, String message);
	void TriGUIsendConfig();
	void TriGUIsendCopter();
	void TriGUIsendReceiver();
	void TriGUIsendIMU();

private:
	byte state;
	byte mode;

	Configuration config;

	unsigned long fastLoopCount;
	unsigned long mediumLoopStartTime;
	unsigned long mediumLoopCount;

	SatelliteReceive receiver;

	Directions setPoint;
	Directions output;

	IMURazor imu;

	PID rollHoverPID;
	PID nickHoverPID;
	PID yawHoverPID;

	PID rollAcroPID;
	PID nickAcroPID;
	PID yawAcroPID;

	Mixer mix;

	StopWatch stopWatch; //TODO:remove after benchmark test

	//TODO: refactor:
	bool gsInMessage;
	byte gsMessageByteCount;
	byte gsPostFixCount;
	byte gsData[50];
	byte gsDataByteCount;
	byte gsCMD;

};

#endif /* TRICOPTER_H_ */