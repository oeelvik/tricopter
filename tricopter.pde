#include <SatelliteRecive.h>
#include <Servo.h>

//Sensor pins
#define GYRO_ROLL_PIN 4
#define GYRO_NICK_PIN 3
#define GYRO_YAW_PIN 5
#define ACC_ROLL_PIN 1
#define ACC_NICK_PIN 2
#define ACC_VERT_PIN 0
#define YAW_SERVO_PIN 9

//Sensor reversing
#define GYRO_ROLL_DIR 0
#define GYRO_NICK_DIR 0
#define GYRO_YAW_DIR 0
#define ACC_ROLL_DIR 1
#define ACC_NICK_DIR 1
#define ACC_VERT_DIR 0

//Accelerometer trim
#define ACC_ROLL_TRIM -11
#define ACC_NICK_TRIM 0
#define ACC_VERT_TRIM 0

//Accelerometer value at 1g of force
float acc_1g = 102;//(float) 300 / 3300 * 1024; //300[mV pr g] / 3300[Aref] * 1024 = 93 [val at 1g]

//The accelerometer gain on the complementary filter
float acc_gain = 0.04;

//Gyro value at angular velocity of 1 degree / sec
float gyro_1d = 0.83 / 3300 * 1024;//0.83[mV pr degree] / 3300[Aref] * 1024 = 0,2576[val pr degree pr sec]

//The gyro gain on the complementary filter
float gyro_gain = 1 - acc_gain;

//Sensor signal scale
float acc_scale = 90 / acc_1g;

//Sensor initial values
int init_gyro_roll = 0;
int init_gyro_nick = 0;
int init_gyro_yaw = 0;
int init_acc_roll = 511 + ACC_ROLL_TRIM;
int init_acc_nick = 511 + ACC_NICK_TRIM;
int init_acc_vert = 511 + ACC_VERT_TRIM;

//Vessals orientation
float roll_angle = 0;
float nick_angle = 0;
float yaw_angle = 0;

unsigned long fast_loop_timer = 0;
unsigned long fast_loop_count = 0;
unsigned int medium_loop_count = 0;

float gyro_dt = 0;

SatelliteRecive reciver;
Servo yawServo;

int val = 0;

void setup(){
  analogReference(EXTERNAL);
  
  Serial.begin(115200);
  yawServo.attach(YAW_SERVO_PIN);
  yawServo.write(90);
  calibrate();
}

void loop(){
  //50 Hz Loop
  if(millis()-fast_loop_timer > 19){
    int timeMillis = millis() - fast_loop_timer;
    gyro_dt = (float) timeMillis / 1000.f;
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
  calcAttitude();
  yawServo.write(map(reciver.getRudd(),RXMIN,RXMAX,0,179));
  
}

void medium_loop(){
  //Each case at 10Hz
  switch(medium_loop_count) {
    case 0:
      medium_loop_count++;
  
      break;
    case 1:
      medium_loop_count++;
      send_location();
      break;
    case 2:
      medium_loop_count++;
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

void slow_loop(){
  
}

void calibrate(){
  long v = 0;
  for(int i = 0; i < 50; i++) v += analogRead(GYRO_ROLL_PIN);
  init_gyro_roll = v/50;
  v = 0;
  for(int i = 0; i < 50; i++) v += analogRead(GYRO_NICK_PIN);
  init_gyro_nick = v/50;
  v = 0;
  for(int i = 0; i < 50; i++) v += analogRead(GYRO_YAW_PIN);
  init_gyro_yaw = v/50;
}

float getGyroRoll(){
  if(GYRO_ROLL_DIR == 0){
    return (float)(init_gyro_roll - analogRead(GYRO_ROLL_PIN)) / gyro_1d;
  } else {
    return (float)(analogRead(GYRO_ROLL_PIN) - init_gyro_roll) / gyro_1d;
  }
}

int getGyroNick(){
  if(GYRO_NICK_DIR == 0){
    return (float)(init_gyro_nick - analogRead(GYRO_NICK_PIN)) / gyro_1d;
  } else {
    return (float)(analogRead(GYRO_NICK_PIN) - init_gyro_nick) / gyro_1d;
  }
}

int getGyroYaw(){
  if(GYRO_YAW_DIR == 0){
    return (float)(init_gyro_yaw - analogRead(GYRO_YAW_PIN))  / gyro_1d;
  } else {
    return (float)(analogRead(GYRO_YAW_PIN) - init_gyro_yaw)  / gyro_1d;
  }
}

int getAccRoll(){
  if(ACC_ROLL_DIR == 0){
    return (float)(init_acc_roll - analogRead(ACC_ROLL_PIN)) * acc_scale;
  } else {
    return (float)(analogRead(ACC_ROLL_PIN) - init_acc_roll) * acc_scale;
  }
}

int getAccNick(){
  if(ACC_NICK_DIR == 0){
    return (float)(init_acc_nick - analogRead(ACC_NICK_PIN)) * acc_scale;
  } else {
    return (float)(analogRead(ACC_NICK_PIN) - init_acc_nick) * acc_scale;
  }
}

int getAccVert(){
  if(ACC_VERT_DIR == 0){
    return (float)(init_acc_vert - analogRead(ACC_VERT_PIN)) * acc_scale;
  } else {
    return (float)(analogRead(ACC_VERT_PIN) - init_acc_vert) * acc_scale;
  }
}

//Complementary Filter
void calcAttitude(){
  roll_angle = (gyro_gain * (roll_angle + (getGyroRoll() * gyro_dt))) + (acc_gain * getAccRoll());
  nick_angle = (gyro_gain * (nick_angle + (getGyroNick() * gyro_dt))) + (acc_gain * getAccNick());
  yaw_angle = yaw_angle + (getGyroYaw() * gyro_dt);
  
  //Wrap around:
  /*while(roll_angle > 180) roll_angle = roll_angle - 180;
  while(roll_angle < -180) roll_angle = roll_angle + 180;
  while(nick_angle > 180) nick_angle = nick_angle - 180;
  while(nick_angle < -180) nick_angle = nick_angle + 180;*/
  while(yaw_angle > 360) yaw_angle = yaw_angle - 360;
  while(yaw_angle < 0) yaw_angle = yaw_angle + 360;
}

void send_attitude(){
  //Attitude
  Serial.print("+++"); //Prefix
  Serial.print("ASP:"); //Airspeed
  Serial.print(20, DEC);
  Serial.print(",THH:"); //Throttle
  Serial.print(map(reciver.getThro(),153,862,0,100), DEC);
  Serial.print (",RLL:"); //Roll
  Serial.print(roll_angle, DEC);//(analogRead(ACC_ROLL_PIN) - init_acc_roll) * acc_scale, DEC);
  Serial.print (",PCH:"); //Pitch
  Serial.print(nick_angle, DEC);
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
  Serial.print(yaw_angle); 
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
