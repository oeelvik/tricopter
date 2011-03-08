#include "PID.h"

PID::PID(){
  _integratedError = 0;
  _lastError = 0;
}

void PID::setTunings(double Kp, double Ki, double Kd){
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
}

void PID::setOutputLimits(int outputMin, int outputMax){
  _outputMin = outputMin;
  _outputMax = outputMax;
}

int PID::updatePid(int setPoint, int input){
  int error = setPoint - input;
  
  //Make shure we dont integrate when limits are reached
  if(!(_lastOutput >= _outputMax && error > 0) && !(_lastOutput <= _outputMin && error<0)) {
    _integratedError += error; 
  }
  
  int iTerm = _Ki * _integratedError;
  int dTerm = _Kd * (error - _lastError);                            
  
  _lastError = error;
  
  int output = _Kp * (error + iTerm + dTerm);
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
