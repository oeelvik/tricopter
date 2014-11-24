//TODO: Use software serial for communication with the compeuter

#include <SatelliteReceive.h>
#include <Servo.h>
#include <IMURazor.h>
#include <EEPROM.h>
#include <math.h>
//#include <NewSoftSerial.h>
#include <StopWatch.h> //TODO:remove after benchmark test

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

unsigned long fastLoopCount;

unsigned long mediumLoopStartTime;
unsigned long mediumLoopCount;

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


int inByte;

struct SetPoint {
  int throttle;
  int roll;
  int nick;
  int yaw;
  };

SetPoint setPoint = {0, RXCENTER, RXCENTER, RXCENTER};

#define MODE_POSITION 0 //Hold position GPS and Compass ass setpoint Receiver to alter desired position
#define MODE_HOVER 1 //IMU stabilized Receiver setpoint
#define MODE_STUNT 2 //GYRO stabilized Receiver setpoint
byte mode = MODE_POSITION;

#define STATE_OFF 0x00
#define STATE_CONFIG 0x20
#define STATE_READY 0x40
#define STATE_AIRBORNE 0x60
#define STATE_ERROR 0xA0
byte state = STATE_OFF;

void setState(byte s){
  state = s;
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


StopWatch stopWatch; //TODO:remove after benchmark test


/**
 * Run once on startup
 *
 */
void setup(){
  stopWatch.init(); //TODO:remove after benchmark test

  setState(STATE_CONFIG);
  
  analogReference(EXTERNAL);
  Serial.begin(115200);
  
  //Setup Ground Station Serial
  //gsSerial.begin(19200);

  //Some setup only needed to run once
  rollHoverPID.setOutputLimits(-1023,1023);
  nickHoverPID.setOutputLimits(-1023,1023);
  rollAcroPID.setOutputLimits(-1023,1023);
  nickAcroPID.setOutputLimits(-1023,1023);
  yawPID.setOutputLimits(-1023,1023);
  
  //Reset configuration
  if(RESET_CONFIG == 1) resetConfig();
  
  //Loade saved configuration
  readEEPROMConfig();
  
  //Setup copter based on loaded config
  reloade();
  
}

/**
 * Setup copter based on configuration
 */
void reloade(){
  setState(STATE_CONFIG);
  
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

  setState(STATE_READY);
}

/**
 * Main Loop
 *
 * Fastest running loop, initial loop timing and running realy fast tasks.
 */
void loop(){
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
  if(setPoint.throttle > config[CV_MIN_THRO_BYTE] * 4){
    setState(STATE_AIRBORNE);
    
    //TODO: Refactor and implement stunt mode
    //if(mode != MODE_STUNT){ //Hover or Position hold Mode (IMU stabled)
      rollOutput = rollHoverPID.updatePid(setPoint.roll, map(imu.getRollDegree(), -180, 180, 0, 1023));
      nickOutput = nickHoverPID.updatePid(setPoint.nick, map(imu.getNickDegree(), -180, 180, 0, 1023));
      yawOutput = yawPID.updatePid(setPoint.yaw, map(imu.getYawDegree(), -180, 180, 0, 1023));
    /*} else { //Stunt Mode (Gyro stabled)
      rollOutput = rollAcroPID.updatePid(setPoint.roll, imu.getGyroRoll() + 511);
      nickOutput = nickAcroPID.updatePid(setPoint.nick, imu.getGyroNick() + 511);
      yawOutput = yawPID.updatePid(setPoint.yaw, map(imu.getGyroYawDegree(), -180, 180, 0, 1023));
    }*/

  } else setState(STATE_READY);
  
  
  // #########################################
  // ######## Update motors and servo ########
  // #########################################
  mix.setThrust(setPoint.throttle, rollOutput, nickOutput, yawOutput);
}

/**
 * 100Hz Loop
 */
void mediumLoop(){
  mediumLoopCount++;

  // ###########################################
  // ######## Read Serial from receiver ########
  // ###########################################
  if (Serial.available() > 0) {
    inByte = Serial.read();
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
      break;
    case 4:
      TriGUIsendCopter();
      break;
    case 6:
      TriGUIsendReceiver();
      break;
    case 7:
      TriGUIsendIMU();
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
void slowLoop(){
  
}


void updateSetPoints(){
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
