#include <SatelliteRecive.h>
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

//IMU settings:
float acc_gain = 0.04;
float gyro_scale = 11.044;//(1024/360)/1024 * 3,3/0,00083 (0,83mv/deg/sec)
float acc_scale = (float)1024 / 4 / 102;//(float) 300 / 3300 * 1024; //300[mV pr g] / 3300[Aref] * 1024 = 93 [val at 1g] //Adjusted to 102 after trail and error

//Loop Timers
unsigned long fast_loop_timer = 0;
unsigned long fast_loop_count = 0;
unsigned int medium_loop_count = 0;


SatelliteRecive reciver;
Servo yawServo;
Servo rearMotor;
Servo rightMotor;
Servo leftMotor;
IMU imu;

float K = 1;
float Kp = 1.3;
float Ki = 0.05;
float Kd = 0.80;


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
  rearMotor.write(0);
  rightMotor.attach(RIGHT_MOTOR_PIN);
  rightMotor.write(0);
  leftMotor.attach(LEFT_MOTOR_PIN);
  leftMotor.write(0);
}

void loop(){
  //50 Hz Loop
  if(millis()-fast_loop_timer > 19){
    fast_loop_timer = millis();
    fast_loop_count++;
    
    fast_loop();
    
    medium_loop();
  }
  
  if (Serial.available() > 0) {
    reciver.regByte(Serial.read());
  }
}

void fast_loop(){
  /*PSEUDO CODE
  updateIMU
  PID Calculations:
  -rollForce
  -nickForce
  -yawForce
  Update engines
  
  */
  
  imu.update();
  //yawServo.write(map(reciver.getRudd(),RXMIN,RXMAX,0,179));
  
  //PID param trim
  K = (float)(reciver.getElev() - 511)/10;
  Kp = (float)(reciver.getRudd() - 511)/50;
  Ki = (float)(reciver.getGear() - 870)/100;
  Kd = (float)(reciver.getFlap() - 511)/50;
  
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
  Serial.print("\t");
  
  int left = 10;
  if(reciver.getThro() > 200) left = constrain(map(reciver.getThro() + updatePid(reciver.getAile(), imu.getRoll()), -1024, 1024, 10, 179), 10, 179);
  
  yawServo.write(left);
  
  Serial.print(left);
  Serial.print("\t");
  //Serial.print(updatePid(reciver.getAile(), imu.getRoll()));
  Serial.println("\t");
  
  
}

void medium_loop(){
  //Each case at 10Hz
  switch(medium_loop_count) {
    case 0:
      medium_loop_count++;
  
      break;
    case 1:
      medium_loop_count++;
      //send_location();
      break;
    case 2:
      medium_loop_count++;
      break;
    case 3:
      medium_loop_count++;
      //send_attitude();
      break;
    case 4:
      medium_loop_count = 0;
      slow_loop();
      break;
  }
}

void slow_loop(){
  /*GUI setup //TODO: implement softwareSerial
  if (Serial.available() > 0) {
    byte command = Serial.read();
    switch (command){
      case 0x10: //K
        K = Serial.read();
        Serial.print("K: ");
        Serial.println(K);
        break;
      case 0x11: //Kp
        Kp = Serial.read();
        break;
      case 0x12: //Ki
        Ki = Serial.read();
        break;
      case 0x13: //Kd
        Kd = Serial.read();
        break;
    }
    
    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);
  }*/
}

void send_attitude(){
  //Attitude
  Serial.print("+++"); //Prefix
  Serial.print("ASP:"); //Airspeed
  Serial.print(20, DEC);
  Serial.print(",THH:"); //Throttle
  Serial.print(map(reciver.getThro(),153,862,0,100), DEC);
  Serial.print (",RLL:"); //Roll
  Serial.print(imu.getRollDegree());//(analogRead(ACC_ROLL_PIN) - init_acc_roll) * acc_scale, DEC);
  Serial.print (",PCH:"); //Pitch
  Serial.print(imu.getNickDegree());
  Serial.println(",***"); //Suffix
}

void send_location(){
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

int integrated_error = 0;
int last_error = 0;

int updatePid(int targetPosition, int currentPosition)   {
  int error = targetPosition - currentPosition;
  int pTerm = Kp * error;
  integrated_error += error; 
  integrated_error = constrain(integrated_error, -10000, 10000);  
  int iTerm = Ki * integrated_error;
  int dTerm = Kd * (error - last_error);                            
  last_error = error;
  
  /*Serial.print(pTerm);
  Serial.print("\t");
  Serial.print(iTerm);
  Serial.print("\t");
  Serial.print(dTerm);
  Serial.print("\t");
  Serial.print(-constrain(K*(pTerm + iTerm + dTerm), -1024, 1024));
  Serial.print("\t");*/
  
  
  return constrain(K*(pTerm + iTerm + dTerm), -1024, 1024);
}

/**
 * Update the motors thrust and servo angle based on desired forses
 * @param int throttle Desired throttle force (0-1023)
 * @param int roll Desired roll torque (0-1023)
 * @param int nick Desired nick torque (0-1023)
 * @param int yaw Desired yaw torque (0-1023)
 */
void setThrust(int throttle, int roll, int nick, int yaw ){
  
  //half roll on left halfe on right, halfe roll back 1/4 right and 1/4 left
  int left = constrain(map(throttle + (roll / 2) + (nick / 4), 0, 1024, 0, 179), 0, 179);
  int right = constrain(map(throttle - (roll / 2) + (nick / 4), 0, 1024, 0, 179), 0, 179);
  
  //added yaw angle devided by some constant to compensate for vertical thrust loss
  //TODO: adjust constant
  int rear = constrain(map(throttle - (nick / 2) + (abs(yaw - 511) / 4), 0, 1024, 0, 179), 0, 179);
  
  //yaw servo angle
  int yawVal = constrain(map(yaw, 0, 1024, 0, 179), 0, 179);
  
  yawServo.write(yawVal);
  leftMotor.write(left);
  rightMotor.write(right);
  rearMotor.write(rear);
}
