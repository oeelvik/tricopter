#include "Mixer.h"


Mixer::Mixer(){
  armed = false;
}

void Mixer::init(){
  motors.init(4);

  byte command[8] = {armESC, armESC, armESC, 125, 0, 0, 0, 0};
  motors.command(command);
}

void Mixer::setArmESC(int val){
  armESC = val;
}

void Mixer::setIdleSpin(int val){
  idleSpin = val;
}

void Mixer::setMinThro(int val){
  minThro = val;
}

void Mixer::setYawRev(bool val){
  yawRev = val;
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
  if(!armed){
    leftThrust = armESC;
    rightThrust = armESC;
    rearThrust = armESC;
    yawPos = 125;
  }
  else if(throttle < minThro){
    leftThrust = idleSpin;
    rightThrust = idleSpin;
    rearThrust = idleSpin;
    yawPos = 125;
  } else {

    // 1/2 nick on left 1/2 on right = 1 total front
    leftThrust = constrain(map(throttle + roll + (nick / 2), 0, 1024, 0, 250), idleSpin, 250);
    rightThrust = constrain(map(throttle - roll + (nick / 2), 0, 1024, 0, 250), idleSpin, 250);
    
    //added yaw angle devided by some constant to compensate for vertical thrust loss
    //TODO: adjust constant deviding yaw
    rearThrust = constrain(map(throttle - nick + (abs(yaw) / 16), 0, 1024, 0, 250), idleSpin, 250);
    
    //yaw servo angle
    if(yawRev) yawPos = constrain(map(yaw, -1023, 1023, 250, 0), 0, 250);
    else yawPos = constrain(map(yaw, -1023, 1023, 0, 250), 0, 250);
    
  }
    
  byte command[8] = {rightThrust, leftThrust, rearThrust, yawPos, 0, 0, 0, 0};
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
