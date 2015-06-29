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
    void setTunings(double Kp, double Ti, double Td);
    void setOutputLimits(int outputMin, int outputMax);
    int update(int setPoint, int input);
    double getKp();
    double getTi();
    double getTd();
  
  private:
    double _Kp;
    double _Ti;
    double _Td;
    int _outputMax;
    int _outputMin;

    long _integratedError;
    int _lastError;
    int _lastOutput;
    unsigned long lastTime;
    
};
#endif /* PID_H_ */
