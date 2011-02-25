#include <SatelliteReceive.h>
#include <Servo.h>
#include <IMU.h>

//Sensor pins
#define GYRO_ROLL_PIN 4
#define GYRO_NICK_PIN 3
#define GYRO_YAW_PIN 5
#define ACC_ROLL_PIN 1
#define ACC_NICK_PIN 2
#define ACC_VERT_PIN 0

//Output pins
#define YAW_SERVO_PIN 6
#define REAR_MOTOR_PIN 9
#define RIGHT_MOTOR_PIN 10
#define LEFT_MOTOR_PIN 11

//Sensor reversing
boolean GYRO_ROLL_REV = true;
boolean GYRO_NICK_REV = true;
boolean GYRO_YAW_REV = true;
boolean ACC_ROLL_REV = false;
boolean ACC_NICK_REV = false;
boolean ACC_VERT_REV = false;

//Accelerometer trim
#define ACC_ROLL_TRIM -12
#define ACC_NICK_TRIM -10
#define ACC_VERT_TRIM 40

//Minimum throttle to start flight
//TODO: test and adjust
#define MIN_THROTTLE 200

#define MOTOR_ENABLE 1

//Minimum value to arm ESC
//TODO: test and adjust
#define MIN_ESC 10

//Enable Happy Killmore data transfere 1 = enabled
#define HK_ENABLED 0
//Enable Processing triGUI data transfere 1 = enabled
#define TRIGUI_ENABLED 1

//IMU settings:
float acc_gain = 0.04;
float gyro_scale = 11.044;//(1024/360)/1024 * 3,3/0,00083 (0,83mv/deg/sec)
float acc_scale = (float)1024 / 4 / 102;//(float) 300 / 3300 * 1024; //300[mV pr g] / 3300[Aref] * 1024 = 93 [val at 1g] //Adjusted to 102 after trail and error

//Loop Timers
unsigned long fast_loop_timer = 0;
unsigned long fast_loop_count = 0;
unsigned int medium_loop_count = 0;

//TODO: implement falesafe center values
SatelliteReceive receiver;
Servo yawServo;
Servo rearMotor;
Servo rightMotor;
Servo leftMotor;
IMU imu;

int throttle = 0;
int rollForce = 0;
int nickForce = 0;
int yawForce = 0;

int leftThrust = 0;
int rightThrust = 0;
int rearThrust = 0;
int yawPos = 0;

//TODO: Switch to PIDLibrary
float K = 1;
float Kp = 1;
float Ki = 0.05;
float Kd = 0.8;
int integrated_error = 0;
int last_error = 0;


void setup(){
  analogReference(EXTERNAL);
  
  //Setup IMU
  imu.setGyroScale(gyro_scale);
  imu.setAccScale(acc_scale);
  imu.setAccGain(acc_gain);
  imu.setAccTrim(ACC_ROLL_TRIM, ACC_NICK_TRIM, ACC_VERT_TRIM);
  imu.setPins(GYRO_ROLL_PIN, GYRO_NICK_PIN, GYRO_YAW_PIN, ACC_ROLL_PIN, ACC_NICK_PIN, ACC_VERT_PIN);
  imu.setReversing(GYRO_ROLL_REV, GYRO_NICK_REV, GYRO_YAW_REV, ACC_ROLL_REV, ACC_NICK_REV, ACC_VERT_REV);
  imu.calibrateGyro();
  
  Serial.begin(115200);
  yawServo.attach(YAW_SERVO_PIN);
  yawServo.write(90);
  rearMotor.attach(REAR_MOTOR_PIN);
  rearMotor.write(MIN_ESC);
  rightMotor.attach(RIGHT_MOTOR_PIN);
  rightMotor.write(MIN_ESC);
  leftMotor.attach(LEFT_MOTOR_PIN);
  leftMotor.write(MIN_ESC);
}

void loop(){
  //50 Hz Loop
  if(millis()-fast_loop_timer > 19){
    fast_loop_timer = millis();
    fast_loop_count++;
    
    fast_loop();
    
    medium_loop();
  }
  
  //Make shure we get all updates from receiver
  if (Serial.available() > 0) {
    receiver.regByte(Serial.read());
  }
}

/**
 * 50Hz Loop
 *
 * Used fore:
 * -IMU updates
 * -PID controll
 * -Thrust updates
 */
void fast_loop(){
  imu.update();
  
  throttle = 0;
  rollForce = 0;
  nickForce = 0;
  yawForce = 0;
  
  if(receiver.getThro() > MIN_THROTTLE){
    throttle = receiver.getThro();
    
    //TODO: integrate yaw signal
    
    //TODO: remove safety "true ||" to enable switching to stunt mode
    if(true || receiver.getFlap() < RXCENTER){ //Hover Mode (IMU stabled)
      //TODO: Switch to PIDLibrary
      rollForce = updatePid(receiver.getAile(), imu.getRoll());
      //nickForce = updatePid(receiver.getElev(), imu.getNick());
      //yawForce = updatePid(receiver.getRudd(), imu.getYaw());
    } else { //Stunt Mode (Gyro stabled)
      //TODO: Switch to PIDLibrary
      //Gyro signal / 2 to increase max speed to 2 * 360 degree / sec
      rollForce = updatePid(receiver.getAile(), constrain((imu.getGyroRoll() / 2) + 511, 0, 1023));
      //nickForce = updatePid(receiver.getElev(), constrain((imu.getGyroNick() / 2), 0, 1023));
      //yawForce = updatePid(receiver.getRudd(), constrain((imu.getGyroYaw() / 2), 0, 1023));
    }
    
  } else {
    //TODO: Reset PID terms
  }
  
  //TODO: implement manualMode as part of PIDLibrary
  //Manual mode (Only for debug purpose)
  //setThrust(receiver.getThro(), receiver.getAile(), receiver.getElev(), receiver.getYaw());
  
  setThrust(throttle, rollForce, nickForce, yawForce);
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
void medium_loop(){
  //Each case at 10Hz
  switch(medium_loop_count) {
    case 0:
      medium_loop_count++;
      break;
    case 1:
      medium_loop_count++;
      triGUI();
      break;
    case 2:
      medium_loop_count++;
      send_location();
      break;
    case 3:
      medium_loop_count++;
      send_attitude();
      break;
    case 4:
      medium_loop_count = 0;
      slow_loop();
      break;
  }
}

/**
 * 10Hz loop
 */
 //TODO: split in 5 (2 Hz) like medium loop
void slow_loop(){
  
}

/**
 * Communicate with config software
 */
//TODO: implement GUI config tool
void triGUI(){
  if(TRIGUI_ENABLED){
    //TODO: split in multiple functions and run in different medium_loop cases to spred load
    //RECEIVER
    Serial.print(">>>"); //Prefix
    Serial.print(byte(1)); //CMD receiver
    
    //Signal
    Serial.print(receiver.getThro() / 4,BYTE);
    Serial.print(receiver.getAile() / 4,BYTE);
    Serial.print(receiver.getElev() / 4,BYTE);
    Serial.print(receiver.getRudd() / 4,BYTE);
    Serial.print(receiver.getGear() / 4,BYTE);
    Serial.print(receiver.getFlap() / 4,BYTE);
    
    //Reversing
    Serial.print(0,BYTE);
    Serial.print(0,BYTE);
    Serial.print(0,BYTE);
    Serial.print(0,BYTE);
    Serial.print(0,BYTE);
    Serial.print(0,BYTE);
    
    Serial.println("***"); //Suffix
    
    //IMU
    Serial.print(">>>"); //Prefix
    Serial.print(byte(2)); //CMD IMU
    
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
    Serial.print(0,BYTE);
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
    
    Serial.println("***"); //Suffix
    
    //Tricopter
    Serial.print(">>>"); //Prefix
    Serial.print(byte(0)); //CMD Tricopter
    
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
    
    //--------- Data --------------
    Serial.print(millis() / 10000,BYTE); //time 0 - ca 45 min
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
    Serial.print(receiver.getAile() / 4,BYTE);
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
  }
  
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
}

/**
 * Transfere attitude data compatible with Happy Killmore ground station
 * 
 * Only if HK_ENABLED = 1
 */
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



//TODO: Switch to PIDLibrary
int updatePid(int targetPosition, int currentPosition)   {
  int error = targetPosition - currentPosition;
  int pTerm = Kp * error;
  integrated_error += error; 
  integrated_error = constrain(integrated_error, -10000, 10000);  
  int iTerm = Ki * integrated_error;
  int dTerm = Kd * (error - last_error);                            
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
  
  
  return constrain(K*(pTerm + iTerm + dTerm), -1023, 1023);
}

/**
 * Update the motors thrust and servo angle based on desired forses
 * @param int throttle Desired throttle force (0-1023)
 * @param int roll Desired roll torque (-1023 <  > 1023)
 * @param int nick Desired nick torque (-1023 <  > 1023)
 * @param int yaw Desired yaw torque (-1023 <  > 1023)
 */
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
}


