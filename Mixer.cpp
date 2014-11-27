#include "Mixer.h"


Mixer::Mixer(){}

void Mixer::init(){
  motors.init();

  byte command[4] = {minESC, minESC, minESC, 125};
  motors.command(command);
}

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
  leftThrust = constrain(map(throttle + roll + (nick / 2), 0, 1024, 0, 250), minESC, 250);
  rightThrust = constrain(map(throttle - roll + (nick / 2), 0, 1024, 0, 250), minESC, 250);
  
  //added yaw angle devided by some constant to compensate for vertical thrust loss
  //TODO: adjust constant deviding yaw
  rearThrust = constrain(map(throttle - nick + (abs(yaw) / 4), 0, 1024, 0, 250), minESC, 250);
  
  //yaw servo angle
  if(yawRev) yawPos = constrain(map(yaw, -1023, 1023, 250, 0), 0, 250);
  else yawPos = constrain(map(yaw, -1023, 1023, 0, 250), 0, 250);
  
  if(!motorsEnabled or throttle < minThro){
    leftThrust = minESC;
    rightThrust = minESC;
    rearThrust = minESC;
    yawPos = 125;
  }
    
  byte command[4] = {rightThrust, leftThrust, rearThrust, yawPos};
  motors.command(command);
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
