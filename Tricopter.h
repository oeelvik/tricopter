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
#include <GroundStation.h>
#include <DataStream.h>
#include <TricopterStream.h>
#include <Directions.h>
#include <Mixer.h>
#include <PID.h>

//Should only be set to 1 for testing purpose
#define RESET_CONFIG 0 //Set to 1 to loade default config and save it to eeprom on startup


#define MODE_POSITION 0 //Hold position GPS and Compass ass setpoint Receiver to alter desired position
#define MODE_HOVER 1 //IMU stabilized Receiver setpoint
#define MODE_STUNT 2 //GYRO stabilized Receiver setpoint


#define STATE_ERROR 0x00
#define STATE_CONFIG 0x20
#define STATE_READY 0x40
#define STATE_ARMED 0x60
#define STATE_AIRBORNE 0x80


class Tricopter {
public:
	Tricopter();
	void init();

	void reconfigure();

	void fastLoop();
	void mediumLoop();
	void slowLoop();

	bool setState(byte state);
	void updateSetPoints();


private:
	byte state;
	byte mode;

	Configuration config;
	GroundStation groundStation;
	DataStream dataStream;

	unsigned long fastLoopCount;
	unsigned long mediumLoopStartTime;
	unsigned long mediumLoopCount;

	SatelliteReceive receiver;

	Directions setPoint;
	Directions output;

	IMURazor imu;

	PID rollHoverPID;
	PID nickHoverPID;

	PID rollAcroPID;
	PID nickAcroPID;

	PID yawPID;

	Mixer mix;

	StopWatch stopWatch; //TODO:remove after benchmark test

};

#endif /* TRICOPTER_H_ */