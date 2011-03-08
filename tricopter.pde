//TODO: create HappyKillmore methods
//TODO: enable turning on of serial coms
//TODO: Change to PIDLibrary
//TODO: Use software serial for communication with the compeuter
//TODO: 

#include <SatelliteReceive.h>
#include <Servo.h>
#include <IMU.h>
#include <EEPROM.h>
#include <NewSoftSerial.h>
#include <PID_Beta6.h>


#include "ConfigAdressing.cpp"
#include "Mixer.h"


#define RESET_CONFIG 1 //Set to 1 to loade default config and save it to eeprom

//-------------- Ground Station ------------------------
#define GS_RX_PIN 6
#define GS_TX_PIN 5
#define GS_BYTE_LIMIT 50

NewSoftSerial gsSerial(GS_RX_PIN,GS_TX_PIN);

boolean gsInMessage = false;
int gsMessageByteCount = 0;
int gsPostFixCount = 0;
int gsData[GS_BYTE_LIMIT];
int gsDataByteCount = 0;
int gsCMD;

//---------------- TriGUI -------------
#define TRIGUI_MESSAGE_TYPE_INFO 0
#define TRIGUI_MESSAGE_TYPE_WARNING 1
#define TRIGUI_MESSAGE_TYPE_ERROR 2

//------------- Loop Timers --------------------
unsigned long fastLoopTimer;
unsigned long fastLoopCount;
unsigned int mediumLoopCount;

//--------- Desired forces ---------------
int throttle = 0;
int rollForce = 0;
int nickForce = 0;
int yawForce = 0;

//---------- PID --------------
double rollSetpoint, rollInput, rollOutput;
double nickSetpoint, nickInput, nickOutput;
double yawSetpoint, yawInput, yawOutput;

PID rollPID(&rollInput, &rollOutput, &rollSetpoint,2,5,1);
PID nickPID(&nickInput, &nickOutput, &nickSetpoint,2,5,1);
PID yawPID(&yawInput, &yawOutput, &yawSetpoint,2,5,1);

//----------- Configuration --------------
int config[CV_END_BYTE + 1];

SatelliteReceive receiver;
IMU imu;
Mixer mix;

void setup(){
  analogReference(EXTERNAL);
  Serial.begin(115200);
  
  //Setup Ground Station Serial
  gsSerial.begin(19200);
  
  if(RESET_CONFIG == 1) resetConfig();
  
  readEEPROMConfig();
  
  reloade();
  
  rollPID.SetOutputLimits(-1023,1023);
  nickPID.SetOutputLimits(-1023,1023);
  yawPID.SetOutputLimits(-1023,1023);
}

void reloade(){
  TriGUIsendMessage(TRIGUI_MESSAGE_TYPE_INFO,"Setting configuration");
  
  //setup receiver reversing
  int reversing = config[CV_RX_REVERSING_BYTE]; 
  receiver.setReversing( (bitRead(reversing,CV_RX_THRO_REV_BIT )==1), (bitRead(reversing,CV_RX_AILE_REV_BIT )==1), (bitRead(reversing,CV_RX_ELEV_REV_BIT )==1), (bitRead(reversing,CV_RX_RUDD_REV_BIT )==1), (bitRead(reversing,CV_RX_GEAR_REV_BIT )==1), (bitRead(reversing,CV_RX_FLAP_REV_BIT )==1));
  
  //Setup IMU
  float gyro_scale = 11.044;//(1024/360)/1024 * 3,3/0,00083 (0,83mv/deg/sec)
  float acc_scale = (float)1024 / 4 / 102;//(float) 300 / 3300 * 1024; //300[mV pr g] / 3300[Aref] * 1024 = 93 [val at 1g] //Adjusted to 102 after trail and error
  imu.setGyroScale(gyro_scale);
  imu.setAccScale(acc_scale);
  imu.setAccGain((float)config[CV_IMU_ACC_GAIN_BYTE] / 100);
  imu.setAccTrim((config[CV_IMU_ACC_ROLL_TRIM_BYTE] * 4) - 511, (config[CV_IMU_ACC_NICK_TRIM_BYTE] * 4) - 511, (config[CV_IMU_ACC_VERT_TRIM_BYTE] * 4) - 511);
  imu.setPins(config[CV_IMU_GYRO_ROLL_PIN_BYTE], config[CV_IMU_GYRO_NICK_PIN_BYTE], config[CV_IMU_GYRO_YAW_PIN_BYTE], config[CV_IMU_ACC_ROLL_PIN_BYTE], config[CV_IMU_ACC_NICK_PIN_BYTE], config[CV_IMU_ACC_VERT_PIN_BYTE]);
  reversing = config[CV_IMU_REVERSING_BYTE]; 
  imu.setReversing( (bitRead(reversing,CV_IMU_GYRO_ROLL_REV_BIT )==1), (bitRead(reversing,CV_IMU_GYRO_NICK_REV_BIT )==1), (bitRead(reversing,CV_IMU_GYRO_YAW_REV_BIT )==1), (bitRead(reversing,CV_IMU_ACC_ROLL_REV_BIT )==1), (bitRead(reversing,CV_IMU_ACC_NICK_REV_BIT )==1), (bitRead(reversing,CV_IMU_ACC_VERT_REV_BIT )==1) );
  imu.calibrateGyro();
  
  //Setup Mixer
  mix.setMinESC(map(config[CV_MIN_ESC_BYTE], 0, 255, 0, 179));
  mix.setMinThro(map(config[CV_MIN_THRO_BYTE], 0, 255, 0, 1023));
  mix.setMotorsEnabled((bitRead(config[CV_TRICOPTER_ENABLE_BYTE], CV_MOTORS_ENABLE_BIT) == 1));
  mix.setPins(config[CV_LEFT_MOTOR_PIN_BYTE], config[CV_RIGHT_MOTOR_PIN_BYTE], config[CV_REAR_MOTOR_PIN_BYTE], config[CV_YAW_SERVO_PIN_BYTE]);
  
  //Setup PID
  int mode = (bitRead(config[CV_TRICOPTER_ENABLE_BYTE], CV_PID_ENABLE_BIT) == 1) ? AUTO : MANUAL;
  rollPID.SetMode(mode);
  nickPID.SetMode(mode);
  yawPID.SetMode(mode);
  
  rollPID.SetSampleTime(config[CV_PID_SAMPLE_TIME_BYTE]);
  nickPID.SetSampleTime(config[CV_PID_SAMPLE_TIME_BYTE]);
  yawPID.SetSampleTime(config[CV_PID_SAMPLE_TIME_BYTE]);
  
  /*TriGUIsendMessage(0, "-----------");
  TriGUIsendMessage(0,(int)rollPID.GetP_Param());
  TriGUIsendMessage(0,(int)rollPID.GetI_Param());
  TriGUIsendMessage(0,(int)rollPID.GetD_Param());
  TriGUIsendMessage(0, "-----------");
  /*TriGUIsendMessage(0, (int)(float)config[CV_PID_KP_BYTE] / 10);
  TriGUIsendMessage(0, (int)(float)config[CV_PID_KI_BYTE] / 10);
  TriGUIsendMessage(0, (int)(float)config[CV_PID_KD_BYTE] / 10);
  */
  
  TriGUIsendMessage(0, (int)(float)config[CV_PID_KP_BYTE] / 10);
  
  //TODO: enable d term and find bug
  rollPID.SetTunings((float)config[CV_PID_KP_BYTE] / 10, (float)config[CV_PID_KI_BYTE] / 10, 0);//(float)config[CV_PID_KD_BYTE] / 10);
  nickPID.SetTunings((float)config[CV_PID_KP_BYTE] / 10, (float)config[CV_PID_KI_BYTE] / 10, 0);//(float)config[CV_PID_KD_BYTE] / 10);
  yawPID.SetTunings((float)config[CV_PID_KP_BYTE] / 10, (float)config[CV_PID_KI_BYTE] / 10, 0);//(float)config[CV_PID_KD_BYTE] / 10);
}

void loop(){
  //50 Hz Loop
  if(millis()-fastLoopTimer > 19){
    fastLoopTimer = millis();
    fastLoopCount++;
    
    fastLoop();
    
    mediumLoop();
  }
  
  //Make shure we get all updates from receiver
  if (Serial.available() > 0) {
    int inByte = Serial.read();
    receiver.regByte(inByte);
    gsReceive(inByte); //TODO: remove when soft serial is used
  }
  //TODO: implement setup on startup or interrup with org SoftSerial. NewSoftSerial diturbs receiver signal
  /*if (gsSerial.available() > 0)
  {
    gsReceive(gsSerial.read());
  }*/
}

/**
 * 50Hz Loop
 *
 * Used fore:
 * -IMU updates
 * -PID controll
 * -Thrust updates
 */
 
void fastLoop(){
  imu.update();
  
  
  if(receiver.getThro() > config[CV_MIN_THRO_BYTE] * 4){
    throttle = receiver.getThro();
    
    if(receiver.getFlap() < RXCENTER){ //Hover Mode (IMU stabled)
      rollInput = imu.getRoll();
      nickInput = imu.getNick();
      yawInput = imu.getGyroYaw() + 511;
      
      rollSetpoint = receiver.getAile();
      nickSetpoint = receiver.getElev();
      yawSetpoint = receiver.getRudd();
      //Output = Output;
      //TODO: Switch to PIDLibrary
      //rollForce = updatePid(receiver.getAile(), imu.getRoll());
      //nickForce = updatePid(receiver.getElev(), imu.getNick());
      //yawForce = updatePid(receiver.getRudd(), imu.getGyroYaw() + 511); //Uses the derivated version (Gyro signal)
    } else { //Stunt Mode (Gyro stabled)
      //TODO: use gyro signal
      
      rollInput = imu.getGyroRoll() + 511;
      nickInput = imu.getGyroNick() + 511;
      yawInput = imu.getGyroYaw() + 511;
      
      rollSetpoint = receiver.getAile();
      nickSetpoint = receiver.getElev();
      yawSetpoint = receiver.getRudd();
      //TODO: Switch to PIDLibrary
      //Gyro signal / 2 to increase max speed to 2 * 360 degree / sec
      //rollForce = updatePid(receiver.getAile(), constrain((imu.getGyroRoll() / 2) + 511, 0, 1023));
      //nickForce = updatePid(receiver.getElev(), constrain((imu.getGyroNick() / 2) + 511, 0, 1023));
      //yawForce = updatePid(receiver.getRudd(), constrain((imu.getGyroYaw() / 2) + 511, 0, 1023));
    }
    
    
    rollPID.Compute();
    nickPID.Compute();
    yawPID.Compute();
    
  } else {
    rollPID.Reset();
    nickPID.Reset();
    yawPID.Reset();
    //TODO: Reset PID terms
  }
  
  //TODO: implement manualMode as part of PIDLibrary
  //Manual mode (Only for debug purpose)
  //setThrust(receiver.getThro(), receiver.getAile(), receiver.getElev(), receiver.getYaw());
  
  mix.setThrust(receiver.getThro(), rollOutput, nickOutput, yawOutput);
}

/**
 * Splits fast loop into 5 (10Hz each)
 * 
 * Used fore:
 * 0. 
 * 1. Get configuration
 * 2. Happy Killmore location
 * 3. Happy Killmore attitude
 * 4. slow_loop()
 */
 
void mediumLoop(){
  
  /*Serial.print(Input);
  Serial.print(":");
  Serial.println((Output + 1023) / 8, BYTE);*/
  //Each case at 10Hz
  switch(mediumLoopCount) {
    case 0:
      TriGUIsendCopter();
      break;
    case 1:
      TriGUIsendReceiver();
      break;
    case 2:
      TriGUIsendIMU();
      HappyKillmoreSendAttitude();
      break;
    case 3:
      HappyKillmoreSendLocation();
      break;
    case 4:
      mediumLoopCount = -1;
      slowLoop();
      break;
  }
  mediumLoopCount++;
}

/**
 * 10Hz loop
 */
 //TODO: split in 5 (2 Hz) like medium loop
void slowLoop(){
  
}



/**
 * Communicate with config software
 */
//TODO: implement GUI config tool
/*
void triGUI(){
  if(TRIGUI_ENABLED){
    //TODO: split in multiple functions and run in different medium_loop cases to spred load
    //RECEIVER
    Serial.print(">>>"); //Prefix
    Serial.print(3,BYTE); //CMD receiver
    
    //Signal
    Serial.print(receiver.getThro() / 4,BYTE);
    Serial.print(receiver.getAile() / 4,BYTE);
    Serial.print(receiver.getElev() / 4,BYTE);
    Serial.print(receiver.getRudd() / 4,BYTE);
    Serial.print(receiver.getGear() / 4,BYTE);
    Serial.print(receiver.getFlap() / 4,BYTE);
    
    //Reversing
    /*Serial.print(0,BYTE);
    Serial.print(0,BYTE);
    Serial.print(0,BYTE);
    Serial.print(0,BYTE);
    Serial.print(0,BYTE);
    Serial.print(0,BYTE);
    */
  /*  Serial.println("***"); //Suffix
    
    //IMU
    Serial.print(">>>"); //Prefix
    Serial.print(5,BYTE); //CMD IMU
    
    //Signal
    Serial.print(imu.getRoll() / 4,BYTE);
    Serial.print(imu.getNick() / 4,BYTE);
    Serial.print(imu.getYaw() / 4,BYTE);
    
    Serial.print(imu.getGyroRoll() / 4,BYTE);
    Serial.print(imu.getGyroNick() / 4,BYTE);
    Serial.print(imu.getGyroYaw() / 4,BYTE);
    
    Serial.print(imu.getAccRoll() / 4,BYTE);
    Serial.print(imu.getAccNick() / 4,BYTE);
    Serial.print(0,BYTE); // AccVert
    
    //Reversing
    /*Serial.print(0,BYTE);
    Serial.print(0,BYTE);
    Serial.print(0,BYTE);
    
    Serial.print(0,BYTE);
    Serial.print(0,BYTE);
    Serial.print(0,BYTE);
    
    //Acc Trim
    Serial.print(ACC_ROLL_TRIM,BYTE);
    Serial.print(ACC_NICK_TRIM,BYTE);
    Serial.print(ACC_VERT_TRIM,BYTE);
    
    //Gain
    Serial.print(acc_gain,BYTE);
    */
    /*Serial.println("***"); //Suffix
    
    //Tricopter
    Serial.print(">>>"); //Prefix
    Serial.print(1, BYTE); //CMD Tricopter
    /*
    //------- Setup -------------
    Serial.print(MOTOR_ENABLE,BYTE);
    Serial.print(MIN_THROTTLE / 4,BYTE);
    Serial.print(map(MIN_ESC, 0,179,0,1023),BYTE); //0-179
    
    //Serial communication
    Serial.print(HK_ENABLED,BYTE);
    Serial.print(TRIGUI_ENABLED,BYTE);
    
    //PID
    Serial.print(1,BYTE);
    Serial.print(0,BYTE);
    Serial.print((int)(Kp * 25),BYTE);
    Serial.print((int)(Ki * 1000),BYTE);
    Serial.print((int)(Kd * 100),BYTE);
    */
    //--------- Data --------------
    /*Serial.print(millis() / 10000,BYTE); //time 0 - ca 45 min
    Serial.print(((receiver.getThro() > MIN_THROTTLE)? 0x30 : 0x20), BYTE);
    Serial.print(((receiver.getFlap() < RXCENTER) ? 0x00: 0x10), BYTE);
   
    //Motors and servo
    Serial.print(map(leftThrust, 0, 179, 0, 255),BYTE);
    Serial.print(map(rightThrust, 0, 179, 0, 255),BYTE);
    Serial.print(map(rearThrust, 0, 179, 0, 255),BYTE);
    Serial.print(map(yawPos, 0, 179, 0, 255),BYTE);
    
    //PID
    Serial.print((rollForce + 1023) / 8,BYTE);
    Serial.print((nickForce + 1023) / 8,BYTE);
    Serial.print((yawForce + 1023) / 8,BYTE);
    
    Serial.println("***"); //Suffix
    
    /*Serial.println();
    
    Serial.print(receiver.getRudd() / 4,BYTE);
    Serial.print(receiver.getThro() / 4,BYTE);
    Serial.print(receiver.getElev() / 4,BYTE);
    Se
    rial.print(receiver.getAile() / 4,BYTE);
    Serial.print(map(yawPos, 0, 179, 0, 255),BYTE);
    Serial.print(map(leftThrust, 0, 179, 0, 255),BYTE);
    Serial.print(map(rearThrust, 0, 179, 0, 255),BYTE);
    Serial.print(map(rightThrust, 0, 179, 0, 255),BYTE);
    
    Serial.print(receiver.getAile() / 4,BYTE);
    Serial.print(imu.getRoll() / 4 ,BYTE);
    Serial.print((rollForce + 1023) / 8,BYTE);
    
    Serial.print(receiver.getElev() / 4,BYTE);
    Serial.print(imu.getNick() / 4 ,BYTE);
    Serial.print((nickForce + 1023) / 8,BYTE);
    
    Serial.print(receiver.getRudd() / 4,BYTE);
    Serial.print(imu.getYaw() / 4 ,BYTE);
    Serial.print((yawForce + 1023) / 8,BYTE);
    
    Serial.print(0,BYTE);
    Serial.print(0,BYTE);
    Serial.print(0,BYTE);
    */
  /*}
  
  //PID param trim
  //This can be used to trim PID params. (Setpoints (Se fast_loop) to PID must be set static (511) before enabeling this)
  /*K = (float)(receiver.getElev() - 511)/10;
  Kp = (float)(receiver.getRudd() - 511)/50;
  Ki = (float)(receiver.getGear() - 870)/100;
  Kd = (float)(receiver.getFlap() - 511)/50;
  
  Serial.print("K: ");
  Serial.print(K);
  Serial.print("\t");
  Serial.print("Kp: ");
  Serial.print(Kp);
  Serial.print("\t");
  Serial.print("Ki: ");
  Serial.print(Ki);
  Serial.print("\t");
  Serial.print("Kd: ");
  Serial.print(Kd);
  Serial.print("\t");*/
/*}

/**
 * Transfere attitude data compatible with Happy Killmore ground station
 * 
 * Only if HK_ENABLED = 1
 */
 /*
void send_attitude(){
  if(HK_ENABLED == 1){
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
 /*
void send_location(){
  if(HK_ENABLED == 1){
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


*/
//TODO: Switch to PIDLibrary
int integrated_error = 0;
int last_error = 0;

int updatePid(int targetPosition, int currentPosition)   {
  int error = targetPosition - currentPosition;
  int pTerm = (float)config[CV_PID_KP_BYTE] / 25 * error;
  integrated_error += error; 
  integrated_error = constrain(integrated_error, -10000, 10000);  
  int iTerm = (float)config[CV_PID_KI_BYTE] / 255 * integrated_error;
  int dTerm = (float)config[CV_PID_KD_BYTE] / 50 * (error - last_error);                            
  last_error = error;
  
  //Uncomment to debug
  /*Serial.print(pTerm);
  Serial.print("\t");
  Serial.print(iTerm);
  Serial.print("\t");
  Serial.print(dTerm);
  Serial.print("\t");
  Serial.print(-constrain(K*(pTerm + iTerm + dTerm), -1024, 1024));
  Serial.println("\t");*/
  
  
  return constrain(1*(pTerm + iTerm + dTerm), -1023, 1023);
}

/**
 * Update the motors thrust and servo angle based on desired forses
 * @param int throttle Desired throttle force (0-1023)
 * @param int roll Desired roll torque (-1023 <  > 1023)
 * @param int nick Desired nick torque (-1023 <  > 1023)
 * @param int yaw Desired yaw torque (-1023 <  > 1023)
 *//*
void setThrust(int throttle, int roll, int nick, int yaw ){ 
  
  // 1/2 nick on left 1/2 on right = 1 total front
  leftThrust = constrain(map(throttle + roll + (nick / 2), 0, 1024, 0, 179), MIN_ESC, 179);
  rightThrust = constrain(map(throttle - roll + (nick / 2), 0, 1024, 0, 179), MIN_ESC, 179);
  
  //added yaw angle devided by some constant to compensate for vertical thrust loss
  //TODO: adjust constant
  rearThrust = constrain(map(throttle - nick + (abs(yaw) / 4), 0, 1024, 0, 179), MIN_ESC, 179);
  
  //yaw servo angle
  yawPos = constrain(map(yaw, -1023, 1023, 0, 179), 0, 179);
  
  if(MOTOR_ENABLE == 1){
    leftMotor.write(leftThrust);
    rightMotor.write(rightThrust);
    rearMotor.write(rearThrust);
    yawServo.write(yawPos);
  }
  
  //Uncomment to debug
  /*Serial.print("Throttle: ");
  Serial.print(throttle);
  Serial.print("\t");
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("\t");
  Serial.print("Nick: ");
  Serial.print(nick);
  Serial.print("\t");
  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("Left: ");
  Serial.print(left);
  Serial.print("\t");
  Serial.print("Right: ");
  Serial.print(right);
  Serial.print("\t");
  Serial.print("Rear: ");
  Serial.print(rear);
  Serial.print("\t");
  Serial.print("Yaw: ");
  Serial.print(yawVal);
  Serial.println("\t");*/
/*}
*/

