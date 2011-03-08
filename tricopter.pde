//TODO: Use software serial for communication with the compeuter
//TODO: 

#include <SatelliteReceive.h>
#include <Servo.h>
#include <IMU.h>
#include <EEPROM.h>
#include <NewSoftSerial.h>


#include "ConfigAdressing.cpp"
#include "Mixer.h"
#include "PID.h"


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
double rollOutput;
double nickOutput;
double yawOutput;

PID rollPID;
PID nickPID;
PID yawPID;

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
  
  rollPID.setOutputLimits(-1023,1023);
  nickPID.setOutputLimits(-1023,1023);
  yawPID.setOutputLimits(-1023,1023);
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
  rollPID.setTunings((float)config[CV_PID_KP_BYTE] / 25, (float)config[CV_PID_KI_BYTE] / 255, (float)config[CV_PID_KD_BYTE] / 25);
  nickPID.setTunings((float)config[CV_PID_KP_BYTE] / 25, (float)config[CV_PID_KI_BYTE] / 255, (float)config[CV_PID_KD_BYTE] / 25);
  yawPID.setTunings((float)config[CV_PID_KP_BYTE] / 25, (float)config[CV_PID_KI_BYTE] / 255, (float)config[CV_PID_KD_BYTE] / 25);
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
    
      rollOutput = rollPID.updatePid(receiver.getAile(), imu.getRoll());
      nickOutput = nickPID.updatePid(receiver.getElev(), imu.getNick());
      yawOutput = yawPID.updatePid(receiver.getRudd(), imu.getGyroYaw() + 511);
      
      
    } else { //Stunt Mode (Gyro stabled)
      rollOutput = rollPID.updatePid(receiver.getAile(), imu.getGyroRoll() + 511);
      nickOutput = nickPID.updatePid(receiver.getElev(), imu.getGyroNick() + 511);
      yawOutput = yawPID.updatePid(receiver.getRudd(), imu.getGyroYaw() + 511);
    }
    
  }
  
  //TODO: Reset PID for roll and nick (yaw is the same in both modes) when switshing to new mode or use seperate PID objects for eact mode
  
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
