/**
 * TricopterStream.h
 *
 * @author Øystein Schrøder Elvik
 * @version 1.0
 * @since 28.11.2014
 * 
 */

#ifndef TRICOPTERSTREAM_H_
#define TRICOPTERSTREAM_H_

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <DataStream.h>

#include <SatelliteReceive.h>
#include <Directions.h>
#include <IMURazor.h>
#include <Mixer.h>

class TricopterStream {
public:
	static SatelliteReceive *receiver;
	static Directions *setPoint;
	static IMURazor *imu;
	static Directions *output;
	static Mixer *mix;

	static void attachAll(DataStream *dataStream);

	static byte time(); // 1 = 10 sec, 6 = 1 minute

	//receiver
	static byte receiverThrottle();
	static byte receiverAileron();
	static byte receiverElevator();
	static byte receiverRudder();
	static byte receiverGear();
	static byte receiverFlaps();

    //Setpoint
	static byte setPointVertical();
	static byte setPointRoll();
	static byte setPointNick();
	static byte setPointYaw();

    //IMU
	static byte imuRollDegree();
	static byte imuNickDegree();
	static byte imuYawDegree();

	static byte imuRollRotation(); //radians last interval
	static byte imuNickRotation();
	static byte imuYawRotation();

	static byte imuAccelerationX(); //-5g <-> +5g 
	static byte imuAccelerationY();
	static byte imuAccelerationZ();
    
    //PID out
	static byte outputVertical();
	static byte outputRoll();
	static byte outputNick();
	static byte outputYaw();

    //Motors
	static byte mixLeftThrust();
	static byte mixRightThrust();
	static byte mixRearThrust();
	static byte mixYawPos();



	//############## HappyKillmore ##############
	static void attachHK(DataStream *dataStream);

	static byte HKAttitudePre();
	static byte HKLocationPre();
	static byte HK_C();
	static byte HK_H();
	static byte HK_L();
	static byte HK_P();
	static byte HK_R();
	static byte HK_S();
	static byte HK_T();
	static byte HK_Colon();
	static byte HK_Comma();
	static byte HKPost();

	static byte HKThro();
	static byte HKRoll();
	static byte HKNick();
	static byte HKYaw();
};


#endif /* TRICOPTERSTREAM_H_ */