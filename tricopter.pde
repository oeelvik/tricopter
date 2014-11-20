//TODO: Use software serial for communication with the compeuter

#include <SatelliteReceive.h>
#include <Servo.h>
#include <IMURazor.h>
#include <EEPROM.h>
#include <math.h>
//#include <NewSoftSerial.h>

#include "ConfigAdressing.cpp"
#include "Mixer.h"
#include "PID.h"

//Should only be set to 1 for testing purpose
#define RESET_CONFIG 0 //Set to 1 to loade default config and save it to eeprom on startup

//-------------- Ground Station ------------------------
#define GS_RX_PIN 6
#define GS_TX_PIN 5
#define GS_BYTE_LIMIT 50

//NewSoftSerial gsSerial(GS_RX_PIN,GS_TX_PIN);

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

//---------- PID --------------
double rollOutput;
double nickOutput;
double yawOutput;

PID rollHoverPID;
PID nickHoverPID;

PID rollAcroPID;
PID nickAcroPID;

PID yawPID;

//----------- Configuration --------------
int config[CV_END_BYTE + 1];

//------------ Objects and copter vars --------------
SatelliteReceive receiver;
IMURazor imu;
Mixer mix;

int mode = 0x00; //0x00 = hover, 0x10 = acro
int state = 0x00; //0x00 = off, 0x10 = configuring, 0x20 = armed, 0x30 = airborne, 0xA0 = Error

int throttle = 0;

/**
 * Run once on startup
 *
 */
void setup(){
  state = 0x10;
  
  analogReference(EXTERNAL);
  Serial.begin(115200);
  
  //Setup Ground Station Serial
  //gsSerial.begin(19200);
  
  //Reset configuration
  if(RESET_CONFIG == 1) resetConfig();
  
  //Loade saved configuration
  readEEPROMConfig();
  
  //Setup copter based on loaded config
  reloade();
  
  
  //Some setup only needed to run once
  rollHoverPID.setOutputLimits(-1023,1023);
  nickHoverPID.setOutputLimits(-1023,1023);
  rollAcroPID.setOutputLimits(-1023,1023);
  nickAcroPID.setOutputLimits(-1023,1023);
  yawPID.setOutputLimits(-1023,1023);
}

/**
 * Setup copter based on configuration
 */
void reloade(){
  state = 0x10;
  
  TriGUIsendMessage(TRIGUI_MESSAGE_TYPE_INFO,"Setting configuration");
  
  //setup receiver reversing
  int reversing = config[CV_RX_REVERSING_BYTE]; 
  receiver.setReversing( (bitRead(reversing,CV_RX_THRO_REV_BIT )==1), (bitRead(reversing,CV_RX_AILE_REV_BIT )==1), (bitRead(reversing,CV_RX_ELEV_REV_BIT )==1), (bitRead(reversing,CV_RX_RUDD_REV_BIT )==1), (bitRead(reversing,CV_RX_GEAR_REV_BIT )==1), (bitRead(reversing,CV_RX_FLAP_REV_BIT )==1));
  
  //Setup Mixer
  mix.setMinESC(map(config[CV_MIN_ESC_BYTE], 0, 255, 0, 179));
  mix.setMinThro(map(config[CV_MIN_THRO_BYTE], 0, 255, 0, 1023));
  mix.setYawRev((bitRead(config[CV_TRICOPTER_ENABLE_BYTE], CV_YAW_SERVO_REV_BIT) == 1));
  mix.setMotorsEnabled((bitRead(config[CV_TRICOPTER_ENABLE_BYTE], CV_MOTORS_ENABLE_BIT) == 1));
  mix.setPins(config[CV_LEFT_MOTOR_PIN_BYTE], config[CV_RIGHT_MOTOR_PIN_BYTE], config[CV_REAR_MOTOR_PIN_BYTE], config[CV_YAW_SERVO_PIN_BYTE]);
  
  //Setup PID
  rollHoverPID.setTunings((float)config[CV_HOVER_PID_KP_BYTE] / 25, (float)config[CV_HOVER_PID_KI_BYTE] / 25500, (float)config[CV_HOVER_PID_KD_BYTE] / 25);
  nickHoverPID.setTunings((float)config[CV_HOVER_PID_KP_BYTE] / 25, (float)config[CV_HOVER_PID_KI_BYTE] / 25500, (float)config[CV_HOVER_PID_KD_BYTE] / 25);
  
  rollAcroPID.setTunings((float)config[CV_HOVER_PID_KP_BYTE] / 25, (float)config[CV_HOVER_PID_KI_BYTE] / 25500, (float)config[CV_HOVER_PID_KD_BYTE] / 25);
  nickAcroPID.setTunings((float)config[CV_HOVER_PID_KP_BYTE] / 25, (float)config[CV_HOVER_PID_KI_BYTE] / 25500, (float)config[CV_HOVER_PID_KD_BYTE] / 25);
  
  yawPID.setTunings((float)config[CV_YAW_PID_KP_BYTE] / 25, (float)config[CV_YAW_PID_KI_BYTE] / 25500, (float)config[CV_YAW_PID_KD_BYTE] / 25);

  //Setup IMU
  imu.setAccelWeight((float)config[CV_IMU_ACC_WEIGHT_BYTE] / 100);
  imu.setMagWeight(0.0);
  imu.setAccelTrim(
    (config[CV_IMU_ACC_NICK_TRIM_BYTE] * 4) - 511, 
    (config[CV_IMU_ACC_ROLL_TRIM_BYTE] * 4) - 511, 
    (config[CV_IMU_ACC_VERT_TRIM_BYTE] * 4) - 511
    );
  imu.setPins( 
    config[CV_IMU_ACC_NICK_PIN_BYTE], 
    config[CV_IMU_ACC_ROLL_PIN_BYTE],
    config[CV_IMU_ACC_VERT_PIN_BYTE],
    config[CV_IMU_GYRO_ROLL_PIN_BYTE], 
    config[CV_IMU_GYRO_NICK_PIN_BYTE], 
    config[CV_IMU_GYRO_YAW_PIN_BYTE]
    );
  reversing = config[CV_IMU_REVERSING_BYTE]; 
  imu.setReversing( 
    (bitRead(reversing,CV_IMU_ACC_NICK_REV_BIT )==1),
    (bitRead(reversing,CV_IMU_ACC_ROLL_REV_BIT )==1), 
    (bitRead(reversing,CV_IMU_ACC_VERT_REV_BIT )==1), 
    (bitRead(reversing,CV_IMU_GYRO_ROLL_REV_BIT )==1), 
    (bitRead(reversing,CV_IMU_GYRO_NICK_REV_BIT )==1), 
    (bitRead(reversing,CV_IMU_GYRO_YAW_REV_BIT )==1) 
    );
  imu.init();

  state = 0x20; //Armed
}

/**
 * Main Loop
 *
 * Fastest running loop, initial loop timing and running realy fast tasks.
 */
void loop(){
  //50 Hz Loop
  if(millis()-fastLoopTimer > 4){
    fastLoopTimer = millis();
    fastLoopCount++;
    
    fastLoop();
    
    mediumLoop();
  }
  
  //Make shure we register all serial updates from receiver
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
  
  mode = (receiver.getFlap() < RXCENTER) ? 0x00 : 0x10;
  
  throttle = receiver.getThro();
  
  //TODO: Check if failsafe works
  //Failsafe (more than 500 millis sins last message from receiver)
  if(receiver.getTimeSinceMessage() > 500) {
    throttle = 0;
  }
  
  if(throttle > config[CV_MIN_THRO_BYTE] * 4){
    state = 0x30; //Airborne
    
    //TODO: Refactor and implement stunt mode
    //if(mode != 0x10){ //Hover Mode (IMU stabled)
      rollOutput = rollHoverPID.updatePid(receiver.getAile(), map(imu.getRollDegree(), -180, 180, 0, 1023));
      nickOutput = nickHoverPID.updatePid(receiver.getElev(), map(imu.getNickDegree(), -180, 180, 0, 1023));
    /*} else { //Stunt Mode (Gyro stabled)
      rollOutput = rollAcroPID.updatePid(receiver.getAile(), imu.getGyroRoll() + 511);
      nickOutput = nickAcroPID.updatePid(receiver.getElev(), imu.getGyroNick() + 511);
    }*/
    
    yawOutput = yawPID.updatePid(receiver.getRudd(), map(imu.getYawDegree(), -180, 180, 0, 1023));
  } else state = 0x10; //Armed
  
  
  mix.setThrust(throttle, rollOutput, nickOutput, yawOutput);
}

/**
 * Splits fast loop into 5 (10Hz each)
 * 
 * Used fore:
 * 0. TriGUI Copter data
 * 1. TriGUI Receiver data
 * 2. Happy Killmore attitude and TriGUI IMU data
 * 3. Happy Killmore location
 * 4. slow_loop()
 */
void mediumLoop(){
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
      slowLoop();
      break;
    default:
      if(mediumLoopCount > 18) mediumLoopCount = -1;
    
  }
  mediumLoopCount++;
}

/**
 * 10Hz loop
 * 
 * Navigation should be implemented here
 */
 //TODO: split in 5 (2 Hz) like medium loop
void slowLoop(){
  TriGUIsendMessage(0, "------------");
  TriGUIsendMessage(0, String(millis()-fastLoopTimer));
  TriGUIsendMessage(0, String(fastLoopCount));
  
}
