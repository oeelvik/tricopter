#include "Mixer.h"



Mixer::Mixer(){}

void Mixer::setMinESC(int val){
  minESC = val;
}
void Mixer::setMinThro(int val){
  minThro = val;
}

void Mixer::setYawRev(bool val){
  yawRev = val;
}
void Mixer::setMotorsEnabled(bool val){
  motorsEnabled = val;
}
void Mixer::setPins(int left, int right, int rear, int yaw){
  leftMotor.attach(left);
  leftMotor.write(minESC);
  rightMotor.attach(right);
  rightMotor.write(minESC);
  rearMotor.attach(rear);
  rearMotor.write(minESC);
  yawServo.attach(yaw);
  yawServo.write(90);
}

/**
 * Update the motors thrust and servo angle based on desired forses
 * @param int throttle Desired throttle force (0-1023)
 * @param int roll Desired roll torque (-1023 <  > 1023)
 * @param int nick Desired nick torque (-1023 <  > 1023)
 * @param int yaw Desired yaw torque (-1023 <  > 1023)
 */
void Mixer::setThrust(int throttle, int roll, int nick, int yaw ){
  
  // 1/2 nick on left 1/2 on right = 1 total front
  leftThrust = constrain(map(throttle + roll + (nick / 2), 0, 1024, 0, 179), minESC, 179);
  rightThrust = constrain(map(throttle - roll + (nick / 2), 0, 1024, 0, 179), minESC, 179);
  
  //added yaw angle devided by some constant to compensate for vertical thrust loss
  //TODO: adjust constant deviding yaw
  rearThrust = constrain(map(throttle - nick + (abs(yaw) / 4), 0, 1024, 0, 179), minESC, 179);
  
  //yaw servo angle
  if(yawRev) yawPos = constrain(map(yaw, -1023, 1023, 0, 179), 0, 179);
  else yawPos = constrain(map(yaw, -1023, 1023, 0, 179), 179, 0);
  
  if(throttle < minThro){
    leftThrust = minESC;
    rightThrust = minESC;
    rearThrust = minESC;
    yawPos = 90;
  }
    
  
  if(motorsEnabled){
    leftMotor.write(leftThrust);
    rightMotor.write(rightThrust);
    rearMotor.write(rearThrust);
    yawServo.write(yawPos);
  } else {
    leftMotor.write(minESC);
    rightMotor.write(minESC);
    rearMotor.write(minESC);
    yawServo.write(90);
  }
}


int Mixer::getLeftThrust(){
  return leftThrust;
}

int Mixer::getRightThrust(){
  return rightThrust;
}

int Mixer::getRearThrust(){
  return rearThrust;
}

int Mixer::getYawPos(){
  return yawPos;
}
