/*
 * Tricopter.cpp
 *
 * author: Øystein Schrøder Elvik
 */

#include "Tricopter.h"

 Tricopter::Tricopter() : 
 		config(0), 
 		groundStation(config, receiver, imu, setPoint, output, mix),
 		state(STATE_CONFIG)
 		{

 	mode = MODE_POSITION;

 	setPoint.throttle = 0;
 	setPoint.roll = RXCENTER;
 	setPoint.nick = RXCENTER;
 	setPoint.yaw = RXCENTER;
}


void Tricopter::init(){
 	stopWatch.init(); //TODO:remove after benchmark test
 	analogReference(EXTERNAL);
	Serial.begin(115200);

	//Set PID limits
	rollHoverPID.setOutputLimits(-1023,1023);
	nickHoverPID.setOutputLimits(-1023,1023);
	yawHoverPID.setOutputLimits(-1023,1023);
	rollAcroPID.setOutputLimits(-1023,1023);
	nickAcroPID.setOutputLimits(-1023,1023);
	yawAcroPID.setOutputLimits(-1023,1023);

	//Reset configuration
	if(RESET_CONFIG == 1) config.reset();

	reconfigure();
}


void Tricopter::reconfigure(){
	if(!setState(STATE_CONFIG)) return;

	groundStation.log("Setting configuration");

	//setup receiver reversing
	int reversing = config.get(CV_RX_REVERSING_BYTE); 
	receiver.setReversing( (bitRead(reversing,CV_RX_THRO_REV_BIT )==1), (bitRead(reversing,CV_RX_AILE_REV_BIT )==1), (bitRead(reversing,CV_RX_ELEV_REV_BIT )==1), (bitRead(reversing,CV_RX_RUDD_REV_BIT )==1), (bitRead(reversing,CV_RX_GEAR_REV_BIT )==1), (bitRead(reversing,CV_RX_FLAP_REV_BIT )==1));

	//Setup Mixer
	mix.setArmESC(map(config.get(CV_MIN_ESC_BYTE), 0, 255, 0, 250));
	//TODO: implement separate config for this
	mix.setIdleSpin(map(config.get(CV_MIN_ESC_BYTE), 0, 255, 0, 250) + 30);
	mix.setMinThro(map(config.get(CV_MIN_THRO_BYTE), 0, 255, 0, 1023));
	mix.setYawRev((bitRead(config.get(CV_TRICOPTER_ENABLE_BYTE), CV_YAW_SERVO_REV_BIT) == 1));
	mix.setMotorsEnabled((bitRead(config.get(CV_TRICOPTER_ENABLE_BYTE), CV_MOTORS_ENABLE_BIT) == 1));
	mix.setPins(config.get(CV_LEFT_MOTOR_PIN_BYTE), config.get(CV_RIGHT_MOTOR_PIN_BYTE), config.get(CV_REAR_MOTOR_PIN_BYTE), config.get(CV_YAW_SERVO_PIN_BYTE));
	mix.init();

	//Setup PID
	rollHoverPID.setTunings((float)config.get(CV_HOVER_PID_KP_BYTE) / 25, (float)config.get(CV_HOVER_PID_KI_BYTE) / 25500, (float)config.get(CV_HOVER_PID_KD_BYTE) / 10);
	nickHoverPID.setTunings((float)config.get(CV_HOVER_PID_KP_BYTE) / 25, (float)config.get(CV_HOVER_PID_KI_BYTE) / 25500, (float)config.get(CV_HOVER_PID_KD_BYTE) / 10);
	yawHoverPID.setTunings((float)config.get(CV_YAW_PID_KP_BYTE) / 25, (float)config.get(CV_YAW_PID_KI_BYTE) / 25500, (float)config.get(CV_YAW_PID_KD_BYTE) / 10);

	rollAcroPID.setTunings((float)config.get(CV_HOVER_PID_KP_BYTE) / 25, (float)config.get(CV_HOVER_PID_KI_BYTE) / 25500, (float)config.get(CV_HOVER_PID_KD_BYTE) / 10);
	nickAcroPID.setTunings((float)config.get(CV_HOVER_PID_KP_BYTE) / 25, (float)config.get(CV_HOVER_PID_KI_BYTE) / 25500, (float)config.get(CV_HOVER_PID_KD_BYTE) / 10);
	yawAcroPID.setTunings((float)config.get(CV_YAW_PID_KP_BYTE) / 25, (float)config.get(CV_YAW_PID_KI_BYTE) / 25500, (float)config.get(CV_YAW_PID_KD_BYTE) / 10);

	//Setup IMU
	imu.setAccelWeight((float)config.get(CV_IMU_ACC_WEIGHT_BYTE) / 100);
	imu.setMagWeight(0.0);
	imu.setAccelTrim(
		(config.get(CV_IMU_ACC_NICK_TRIM_BYTE) * 4) - 511, 
		(config.get(CV_IMU_ACC_ROLL_TRIM_BYTE) * 4) - 511, 
		(config.get(CV_IMU_ACC_VERT_TRIM_BYTE) * 4) - 511
		);
	imu.setPins( 
		config.get(CV_IMU_ACC_NICK_PIN_BYTE), 
		config.get(CV_IMU_ACC_ROLL_PIN_BYTE),
		config.get(CV_IMU_ACC_VERT_PIN_BYTE),
		config.get(CV_IMU_GYRO_ROLL_PIN_BYTE), 
		config.get(CV_IMU_GYRO_NICK_PIN_BYTE), 
		config.get(CV_IMU_GYRO_YAW_PIN_BYTE)
		);
	reversing = config.get(CV_IMU_REVERSING_BYTE); 
	imu.setReversing( 
		(bitRead(reversing,CV_IMU_ACC_NICK_REV_BIT )==1),
		(bitRead(reversing,CV_IMU_ACC_ROLL_REV_BIT )==1), 
		(bitRead(reversing,CV_IMU_ACC_VERT_REV_BIT )==1), 
		(bitRead(reversing,CV_IMU_GYRO_ROLL_REV_BIT )==1), 
		(bitRead(reversing,CV_IMU_GYRO_NICK_REV_BIT )==1), 
		(bitRead(reversing,CV_IMU_GYRO_YAW_REV_BIT )==1) 
		);
	imu.init();

	config.dirty = false;
}


/**
 * Fastest running loop, initial loop timing and running realy fast tasks.
 */
void Tricopter::fastLoop(){
	// stopWatch.split();
	// delay(10);
	// stopWatch.split();

	// Serial.println("Medium loop:" + String(stopWatch.splitTimes[1]));
	// Serial.println("IMU:" + String(stopWatch.splitTimes[2]));
	// Serial.println("PID:" + String(stopWatch.splitTimes[3]));
	// Serial.println("Mix:" + String(stopWatch.splitTimes[4]));
	// Serial.println("Loop:" + String(stopWatch.splitTimes[5]));
	// Serial.println("delay(10):" + String(stopWatch.splitTimes[6]));
	// stopWatch.clear();
	// stopWatch.split();
	
	fastLoopCount ++;

	//100 Hz Loop
	if(millis()-mediumLoopStartTime > 9){
		mediumLoopStartTime = millis();

		mediumLoop();
	}

	// stopWatch.split();
	// ############################
	// ######## Update IMU ########
	// ############################
	imu.update();


	// stopWatch.split();
	// #####################################################
	// ######## Update PID's and get output thrusts ########
	// #####################################################
	output.throttle = setPoint.throttle;
	if(setPoint.throttle > config.get(CV_MIN_THRO_BYTE) * 4 and setState(STATE_AIRBORNE)){

		//TODO: Refactor and implement stunt mode
		//if(mode != MODE_STUNT){ //Hover or Position hold Mode (IMU stabled)
			
			output.roll = rollHoverPID.updatePid(setPoint.roll, map(imu.getRollDegree(), -180, 180, 0, 1023));
			output.nick = nickHoverPID.updatePid(setPoint.nick, map(imu.getNickDegree(), -180, 180, 0, 1023));
			output.yaw = yawHoverPID.updatePid(setPoint.yaw, map(imu.getYawDegree(), -180, 180, 0, 1023));
		/*} else { //Stunt Mode (Gyro stabled)
			output.roll = rollAcroPID.updatePid(setPoint.roll, imu.getGyroRoll() + 511);
			output.nick = nickAcroPID.updatePid(setPoint.nick, imu.getGyroNick() + 511);
			output.yaw = yawAcroPID.updatePid(setPoint.yaw, map(imu.getGyroYawDegree(), -180, 180, 0, 1023));
		}*/

	} else if(state == STATE_AIRBORNE) setState(STATE_ARMED);

	// stopWatch.split();
	// #########################################
	// ######## Update motors and servo ########
	// #########################################
	mix.setThrust(output.throttle, output.roll, output.nick, output.yaw);

	// stopWatch.split();
}


/**
 * 100Hz Loop
 */
void Tricopter::mediumLoop(){
	mediumLoopCount++;

	// ###########################################
	// ######## Read Serial from receiver ########
	// ###########################################
	while (Serial.available() > 0) {
		byte inByte = Serial.read();
		receiver.regByte(inByte);

		//TODO: remove when soft serial is used
		groundStation.regByte(inByte);
	}

	// Splits loop into 10 (10Hz each)
	// Each case at 10Hz
	switch(mediumLoopCount) {
	    case 0:
	    case 5:
			// 20Hz
			updateSetPoints();
			break;
	    case 1:
			if(config.dirty) {
				reconfigure();
			} 
			//ARM 
			if(receiver.getGear() > RXCENTER and state == STATE_READY) {
				setState(STATE_ARMED);
				mix.armed = true;
			} else if(receiver.getGear() < RXCENTER and setState(STATE_READY)) {
				mix.armed = false;
			}
			break;
	    case 2:
	    	// stopWatch.split();
	    	// groundStation.log("StopWatch times:");
	    	// for(int i = 0; i < stopWatch.index; i++){
	    	// 	groundStation.log(String(stopWatch.splitTimes[i]));
	    	// }
	    	// stopWatch.clear();
	    	// stopWatch.split();
			break;
	    case 3:
			groundStation.sendCopter();
			break;
	    case 4:
			groundStation.sendReceiver();
			break;
	    case 6:
			groundStation.sendIMU();
			break;
	    case 7:
			groundStation.happyKillmoreSendAttitude();
			break;
	    case 8:
			groundStation.happyKillmoreSendLocation();
			break;
	    case 9:
			slowLoop();
	    default:
			mediumLoopCount = -1;
	}
}


/**
 * 10Hz loop
 * 
 * Navigation should be implemented here
 */
 //TODO: split in 5 (2 Hz) like medium loop
void Tricopter::slowLoop(){
  
}

bool Tricopter::setState(byte state){
	if(state == this->state) return true;

	//TODO: implement status lights
	switch (state) {
		case STATE_CONFIG:

			if(
					this->state != STATE_READY &&
					this->state != STATE_ARMED)
				return false;

			groundStation.log("STATE_CONFIG");
			break;
		case STATE_READY:

			if(
					this->state != STATE_CONFIG &&
					this->state != STATE_ARMED)
				return false;

			groundStation.log("STATE_READY");
			break;
		case STATE_ARMED:

			if(
					this->state != STATE_READY &&
					this->state != STATE_AIRBORNE)
				return false;

			groundStation.log("STATE_ARMED");
			break;
		case STATE_AIRBORNE:
		
			if(
					this->state != STATE_ARMED)
				return false;

			groundStation.log("STATE_AIRBORNE");
			break;
		case STATE_ERROR:
		default:
			groundStation.log("STATE_ERROR");
	}

	this->state = state;

	return true;
}


void Tricopter::updateSetPoints(){
	// only MODE_HOVER is implemented
	mode = MODE_HOVER;
	// if (receiver.getFlap() < RXCENTER)
	//   mode = MODE_POSITION;
	// else if(receiver.getGear() < RXCENTER)
	//   mode = MODE_HOVER;
	// else
	//   mode = MODE_STUNT;

	//TODO: Check if failsafe works
	//Failsafe (more than 500 millis sins last message from receiver)
	bool failsafe = 
		receiver.getTimeSinceMessage() > 500;

	switch (mode){
	case MODE_HOVER:
		if(failsafe) {
			setPoint.throttle = 0;
			setPoint.roll = RXCENTER;
			setPoint.nick = RXCENTER;
			setPoint.yaw = map(imu.getYawDegree(), -180, 180, 0, 1023);
		} else {

			setPoint.throttle = receiver.getThro();
			
			//roll and nick scaled to ca 30 degree max setpoint angle 
			setPoint.roll = map(receiver.getAile(), RXMIN, RXMAX, RXCENTER - 85, RXCENTER + 85);
			setPoint.nick = map(receiver.getElev(), RXMIN, RXMAX, RXCENTER - 85, RXCENTER + 85);

			//use rudd to increment setpoint when airborn
			if(state < STATE_AIRBORNE) {
				setPoint.yaw = map(imu.getYawDegree(), -180, 180, 0, 1023);
			}
			setPoint.yaw += map(receiver.getRudd(), RXMIN, RXMAX, -15, +15);
		}
		break;

	case MODE_STUNT:
		//TODO: implement
		break;

	case MODE_POSITION:
	default:
		//TODO: implement
		break;
	}
}

