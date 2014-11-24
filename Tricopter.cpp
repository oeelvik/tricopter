/*
 * Tricopter.cpp
 *
 * author: Øystein Schrøder Elvik
 */

#include "Tricopter.h"

 Tricopter::Tricopter() : config(0){
	setState(STATE_CONFIG);

 	mode = MODE_POSITION;

 	setPoint.throttle = 0;
 	setPoint.roll = RXCENTER;
 	setPoint.nick = RXCENTER;
 	setPoint.yaw = RXCENTER;

 	//TODO: refactor:
	gsInMessage = false;
	gsMessageByteCount = 0;
	gsPostFixCount = 0;
	gsDataByteCount = 0;
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
	setState(STATE_CONFIG);

	TriGUIsendMessage(TRIGUI_MESSAGE_TYPE_INFO,"Setting configuration");

	//setup receiver reversing
	int reversing = config.get(CV_RX_REVERSING_BYTE); 
	receiver.setReversing( (bitRead(reversing,CV_RX_THRO_REV_BIT )==1), (bitRead(reversing,CV_RX_AILE_REV_BIT )==1), (bitRead(reversing,CV_RX_ELEV_REV_BIT )==1), (bitRead(reversing,CV_RX_RUDD_REV_BIT )==1), (bitRead(reversing,CV_RX_GEAR_REV_BIT )==1), (bitRead(reversing,CV_RX_FLAP_REV_BIT )==1));

	//Setup Mixer
	mix.setMinESC(map(config.get(CV_MIN_ESC_BYTE), 0, 255, 0, 179));
	mix.setMinThro(map(config.get(CV_MIN_THRO_BYTE), 0, 255, 0, 1023));
	mix.setYawRev((bitRead(config.get(CV_TRICOPTER_ENABLE_BYTE), CV_YAW_SERVO_REV_BIT) == 1));
	mix.setMotorsEnabled((bitRead(config.get(CV_TRICOPTER_ENABLE_BYTE), CV_MOTORS_ENABLE_BIT) == 1));
	mix.setPins(config.get(CV_LEFT_MOTOR_PIN_BYTE), config.get(CV_RIGHT_MOTOR_PIN_BYTE), config.get(CV_REAR_MOTOR_PIN_BYTE), config.get(CV_YAW_SERVO_PIN_BYTE));

	//Setup PID
	rollHoverPID.setTunings((float)config.get(CV_HOVER_PID_KP_BYTE) / 25, (float)config.get(CV_HOVER_PID_KI_BYTE) / 25500, (float)config.get(CV_HOVER_PID_KD_BYTE) / 25);
	nickHoverPID.setTunings((float)config.get(CV_HOVER_PID_KP_BYTE) / 25, (float)config.get(CV_HOVER_PID_KI_BYTE) / 25500, (float)config.get(CV_HOVER_PID_KD_BYTE) / 25);
	yawHoverPID.setTunings((float)config.get(CV_YAW_PID_KP_BYTE) / 25, (float)config.get(CV_YAW_PID_KI_BYTE) / 25500, (float)config.get(CV_YAW_PID_KD_BYTE) / 25);

	rollAcroPID.setTunings((float)config.get(CV_HOVER_PID_KP_BYTE) / 25, (float)config.get(CV_HOVER_PID_KI_BYTE) / 25500, (float)config.get(CV_HOVER_PID_KD_BYTE) / 25);
	nickAcroPID.setTunings((float)config.get(CV_HOVER_PID_KP_BYTE) / 25, (float)config.get(CV_HOVER_PID_KI_BYTE) / 25500, (float)config.get(CV_HOVER_PID_KD_BYTE) / 25);
	yawAcroPID.setTunings((float)config.get(CV_YAW_PID_KP_BYTE) / 25, (float)config.get(CV_YAW_PID_KI_BYTE) / 25500, (float)config.get(CV_YAW_PID_KD_BYTE) / 25);

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

	setState(STATE_READY);
}


/**
 * Fastest running loop, initial loop timing and running realy fast tasks.
 */
void Tricopter::fastLoop(){
	fastLoopCount ++;

	//100 Hz Loop
	if(millis()-mediumLoopStartTime > 9){
		mediumLoopStartTime = millis();

		mediumLoop();
	}

	// ############################
	// ######## Update IMU ########
	// ############################
	imu.update();


	// #####################################################
	// ######## Update PID's and get output thrusts ########
	// #####################################################
	if(setPoint.throttle > config.get(CV_MIN_THRO_BYTE) * 4){
		setState(STATE_AIRBORNE);

		//TODO: Refactor and implement stunt mode
		//if(mode != MODE_STUNT){ //Hover or Position hold Mode (IMU stabled)
			output.throttle = setPoint.throttle;
			output.roll = rollHoverPID.updatePid(setPoint.roll, map(imu.getRollDegree(), -180, 180, 0, 1023));
			output.nick = nickHoverPID.updatePid(setPoint.nick, map(imu.getNickDegree(), -180, 180, 0, 1023));
			output.yaw = yawHoverPID.updatePid(setPoint.yaw, map(imu.getYawDegree(), -180, 180, 0, 1023));
		/*} else { //Stunt Mode (Gyro stabled)
			output.throttle = setPoint.throttle;
			output.roll = rollAcroPID.updatePid(setPoint.roll, imu.getGyroRoll() + 511);
			output.nick = nickAcroPID.updatePid(setPoint.nick, imu.getGyroNick() + 511);
			output.yaw = yawAcroPID.updatePid(setPoint.yaw, map(imu.getGyroYawDegree(), -180, 180, 0, 1023));
		}*/

	} else setState(STATE_READY);


	// #########################################
	// ######## Update motors and servo ########
	// #########################################
	mix.setThrust(output.throttle, output.roll, output.nick, output.yaw);
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
    if(state < STATE_AIRBORNE) gsReceive(inByte); //TODO: remove when soft serial is used
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
      break;
    case 2:
      break;
    case 3:
      TriGUIsendCopter();
      break;
    case 4:
      TriGUIsendReceiver();
      break;
    case 6:
      TriGUIsendIMU();
      break;
    case 7:
      HappyKillmoreSendAttitude();
      break;
    case 8:
      HappyKillmoreSendLocation();
      break;
    case 9:
      slowLoop();
    default:
      mediumLoopCount = -1;
    
  }
  mediumLoopCount++;
}


/**
 * 10Hz loop
 * 
 * Navigation should be implemented here
 */
 //TODO: split in 5 (2 Hz) like medium loop
void Tricopter::slowLoop(){
  
}


void Tricopter::setState(byte state){
  this->state = state;
  //TODO: implement status lights
  // switch (state) {
  //     case STATE_OFF:
  //       // do something
  //       break;
  //     case STATE_CONFIG:
  //       // do something
  //       break;
  //     case STATE_READY:
  //       // do something
  //       break;
  //     case STATE_AIRBORNE:
  //       // do something
  //       break;
  //     case STATE_ERROR:
  //     default:
  //       // do something
  //}
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
		receiver.getTimeSinceMessage() > 500 ||
		state < STATE_READY ||
		state >= STATE_ERROR;

	switch (mode){
	case MODE_HOVER:
		if(failsafe) {
			setPoint.throttle = 0;
			setPoint.roll = RXCENTER;
			setPoint.nick = RXCENTER;
			setPoint.yaw = RXCENTER;
		} else {
			setPoint.throttle = receiver.getThro();
			setPoint.roll = receiver.getAile();
			setPoint.nick = receiver.getElev();
			setPoint.yaw = receiver.getRudd();
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


//TODO: ##################################### refactor after this: ################################
void Tricopter::gsReceive(byte inByte){
  gsMessageByteCount++;

  if(!gsInMessage  && inByte != (byte)'>') { //Not this protocol
    gsMessageByteCount = 0;
  } 
  else if(inByte == (byte)'>' && gsMessageByteCount == 3) { //prefix received
    gsInMessage = true;
    gsPostFixCount = 0;
    gsDataByteCount = 0;
    gsCMD = 0;
  } 
  else if (gsInMessage) {

    if(inByte == (byte)'*') gsPostFixCount++;
    else gsPostFixCount = 0;
    
    if(gsPostFixCount == 3 || gsMessageByteCount - 5 >= 50) { //All bytes in message reseived
      gsInMessage = false;
      gsMessageByteCount = 0;
      gsDataByteCount = gsDataByteCount - 2;
      gsCallReceiver();
    }
    else {

      if(gsMessageByteCount == 4) gsCMD = inByte;
      else {
        gsData[gsMessageByteCount - 5] = inByte;
        gsDataByteCount++;
      }
    }
  }
}

void Tricopter::gsCallReceiver(){
  switch(gsCMD){
    case 5: //request config
      TriGUIsendConfig();
      break;
    case 6: //setConfig
      if(gsDataByteCount == CV_END_BYTE + 1){
        TriGUIsendMessage(0, "Configuration received by tricopter");
        config.set(gsData);
        reconfigure();
      } else TriGUIsendMessage(0, "Byte count Missmatch");
      break;
  }
}

/**
 * Transfere attitude data compatible with Happy Killmore ground station
 * 
 * Only if HK_ENABLED = 1
 */
void Tricopter::HappyKillmoreSendAttitude(){
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
void Tricopter::HappyKillmoreSendLocation(){
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


void Tricopter::TriGUIsendMessage(int type, String message){
    Serial.print(">>>"); //Prefix
    Serial.write(1); //CMD message
    Serial.write(type); //type
    Serial.print(message);
    Serial.println("***");
}

void Tricopter::TriGUIsendConfig(){
  Serial.print(">>>"); //Prefix
  Serial.write(5); //CMD Tricopter
  for(int i=0; i < CV_END_BYTE + 1; i++) Serial.write(config.get(i));  
  Serial.println("***"); //Suffix
  TriGUIsendMessage(0, "Configuration sendt form tricopter");
}

void Tricopter::TriGUIsendCopter(){
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

void Tricopter::TriGUIsendReceiver(){
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

void Tricopter::TriGUIsendIMU(){
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
