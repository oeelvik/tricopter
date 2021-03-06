#include "PID.h"

PID::PID(){
  _integratedError = 0;
  _lastError = 0;
  lastTime = 0;

  setDTermFilter(1);
}

void PID::setTunings(float Kp, float Ti, float Td){
  //Scale Accumulated error to the new Tunings
  /*if(_integratedError != 0 && _Ti > 0 && Ti > 0) {
    _integratedError = _integratedError * _Ti / Ti;
  }*/
  _integratedError = 0;
  
  _Kp = Kp;
  if(Ti == 0) _Ki = 0;
  else _Ki = (1.0 / Ti); 
  _Td = Td;
}

void PID::setOutputLimits(int outputMin, int outputMax){
  _outputMin = outputMin;
  _outputMax = outputMax;
}

void PID::setDTermFilter(int size) {
  if(size < 1 ) size = 1;
  
  if(_dTerms) delete [] _dTerms;
  _dTerms = new float[size];

  for (int i = 0; i < size; i++){
    _dTerms[i] = 0;
  }

  _dTermFilterSize = size;
  _dTermFilterPosition = 0;
}

int PID::update(int setPoint, int input){
  //Calculate deltaFactor to adjust iTerm and dTerm for difference in sampling time
  unsigned long now = micros();
  //Translate micros into second
  float deltaFactor = (float)(now - lastTime) / 1000000.0;//micros in 1 sec
  lastTime = now;

  //Check if pid has been on a break, deltaFactor gets to big
  if(deltaFactor > 0.02) {
    deltaFactor = 0.007;
  }

  float error = setPoint - input;
  
  //Make shure we dont integrate when limits are reached
  if(!(_lastOutput >= _outputMax && error > 0) && !(_lastOutput <= _outputMin && error<0)) {
    _integratedError += error * deltaFactor; 
  }
  
  float pTerm = error;
  float iTerm = _Ki * _integratedError;
  float dTerm = filterDTerm(_Td * ((error - _lastError) / deltaFactor));

  _lastError = error;
  
  int output = (int) (_Kp * (pTerm + iTerm + dTerm));
  _lastOutput = output;
  
  return constrain(output, _outputMin, _outputMax);
}

float PID::filterDTerm(float dTerm){
  _dTerms[_dTermFilterPosition] = dTerm;
  
  _dTermFilterPosition++;
  if(_dTermFilterPosition >= _dTermFilterSize) _dTermFilterPosition = 0;

  float sum = 0;
  for (int i = 0; i < _dTermFilterSize; i++){
    sum += _dTerms[i];
  }

  return sum / _dTermFilterSize;
}