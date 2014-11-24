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

#include <SatelliteReceive.h>
#include <IMURazor.h>

#include <Configuration.h>
#include <Directions.h>
#include <Mixer.h>
 
#define LOG_LEVEL_INFO 0
#define LOG_LEVEL_WARNING 1
#define LOG_LEVEL_ERROR 2

#define MAX_MESSAGE_SIZE 50


class GroundStation {
public:
	GroundStation(Configuration& config, SatelliteReceive& receiver, IMURazor& imu, Directions& output, Mixer& mix);

	void regByte(byte inByte);

	void log(String message);
	void warning(String message);
	void error(String message);

	void sendConfig();
	void sendCopter();
	void sendReceiver();
	void sendIMU();

	void happyKillmoreSendAttitude();
	void happyKillmoreSendLocation();

private:

	bool inMessage;
	byte receivedByteCount;
	byte data[MAX_MESSAGE_SIZE];
	byte CMD;

	Configuration& config;
	SatelliteReceive& receiver;
	IMURazor& imu;
	Directions& output;
	Mixer& mix;

	void sendMessage(byte type, String message);
	void executeCommand(byte CMD, byte data[]);
};


#endif /* GROUNDSTATION_H_ */