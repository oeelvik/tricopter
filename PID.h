#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#ifndef PID_H_
#define PID_H_
class PID {
  public:
    PID();
    void setTunings(float Kp, float Ti, float Td);
    void setDTermFilter(int size);
    void setOutputLimits(int outputMin, int outputMax);
    int update(int setPoint, int input);
  
  private:
    float filterDTerm(float dTerm);
    float *_dTerms;
    int _dTermFilterSize;
    int _dTermFilterPosition;

    float _Kp;
    float _Ki;
    float _Td;
    int _outputMax;
    int _outputMin;

    float _integratedError;
    float _lastError;
    int _lastOutput;
    unsigned long lastTime;
    
};
#endif /* PID_H_ */
