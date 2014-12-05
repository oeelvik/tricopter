#include "PID.h"

PID::PID(){
  _integratedError = 0;
  _lastError = 0;
  lastTime = 0;
  expectedDelta = 0;
}

void PID::setTunings(double Kp, double Ki, double Kd){
  //Scale Accumulated error to the new Tunings
  _integratedError = _integratedError * _Ki / Ki;
  
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
}

void PID::setOutputLimits(int outputMin, int outputMax){
  _outputMin = outputMin;
  _outputMax = outputMax;
}

int PID::update(int setPoint, int input){
  //Calculate deltaFactor to adjust iTerm and dTerm for difference in sampling time
  unsigned long now = micros();
  if(expectedDelta > 0){
    int delta = now - lastTime;

    //TODO: enable, use time sins last update to adjust iTerm and dTerm to account for difference in sampling time
    deltaFactor = 1;//delta / expectedDelta;
  }
  else { //Us difference in first two runns to determine expectedDelta
    if(lastTime > 0) {
      expectedDelta = now - lastTime;
    }
    lastTime = now;
  }

  int error = setPoint - input;
  
  //Make shure we dont integrate when limits are reached
  if(!(_lastOutput >= _outputMax && error > 0) && !(_lastOutput <= _outputMin && error<0)) {
    _integratedError += error * deltaFactor; 
  }
  
  int pTerm = _Kp * error;
  int iTerm = _Ki * _integratedError;
  int dTerm = _Kd * (error - _lastError) / deltaFactor;
  
  _lastError = error;
  
  int output = pTerm + iTerm + dTerm;
  _lastOutput = output;
  
  return constrain(output, _outputMin, _outputMax);
}

double PID::getKp(){
  return _Kp;
};

double PID::getKi(){
  return _Ki;
};

double PID::getKd(){
  return _Kd;
};
