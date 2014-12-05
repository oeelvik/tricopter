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
    void setTunings(double Kp, double Ki, double Kd);
    void setOutputLimits(int outputMin, int outputMax);
    int update(int setPoint, int input);
    double getKp();
    double getKi();
    double getKd();
  
  private:
    double _Kp;
    double _Ki;
    double _Kd;
    int _outputMax;
    int _outputMin;

    long _integratedError;
    int _lastError;
    int _lastOutput;
    unsigned long lastTime;
    int expectedDelta;
    double deltaFactor;
    
};
#endif /* PID_H_ */
