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
    void setOutputLimits(int outputMin, int outputMax);
    int update(int setPoint, int input);
    float getKp();
    float getTi();
    float getTd();
  
  private:
    float _Kp;
    float _Ti;
    float _Td;
    int _outputMax;
    int _outputMin;

    float _integratedError;
    float _lastError;
    int _lastOutput;
    unsigned long lastTime;
    
};
#endif /* PID_H_ */
