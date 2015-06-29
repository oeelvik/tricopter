#include "PID.h"

PID::PID(){
  _integratedError = 0;
  _lastError = 0;
  lastTime = 0;
}

void PID::setTunings(double Kp, double Ti, double Td){
  //Scale Accumulated error to the new Tunings
  _integratedError = _integratedError * (1/_Ti) / (1/Ti);
  
  _Kp = Kp;
  _Ti = Ti; 
  _Td = Td;
}

void PID::setOutputLimits(int outputMin, int outputMax){
  _outputMin = outputMin;
  _outputMax = outputMax;
}

int PID::update(int setPoint, int input){
  //Calculate deltaFactor to adjust iTerm and dTerm for difference in sampling time
  unsigned long now = micros();
  //Translate micros into second
  double deltaFactor = (now - lastTime) / 1000000;//micros in 1 sec
  lastTime = now;

  int error = setPoint - input;
  
  //Make shure we dont integrate when limits are reached
  if(!(_lastOutput >= _outputMax && error > 0) && !(_lastOutput <= _outputMin && error<0)) {
    _integratedError += error; 
  }
  
  int pTerm = error;
  int iTerm = ((1 / _Ti) * _integratedError) * deltaFactor;
  int dTerm = (_Td * (error - _lastError)) / deltaFactor;
  
  _lastError = error;
  
  int output = _Kp * (pTerm + iTerm + dTerm);
  _lastOutput = output;
  
  return constrain(output, _outputMin, _outputMax);
}

double PID::getKp(){
  return _Kp;
};

double PID::getTi(){
  return _Ti;
};

double PID::getTd(){
  return _Td;
};
