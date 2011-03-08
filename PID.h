#include "WProgram.h"

#ifndef PID_H_
#define PID_H_
class PID{
  public:
    PID();
    void setTunings(double Kp, double Ki, double Kd);
    void setOutputLimits(int outputMin, int outputMax);
    int updatePid(int setPoint, int input);
    double getKp();
    double getKi();
    double getKd();
  
  private:
    double _Kp;
    double _Ki;
    double _Kd;
    long _integratedError;
    int _lastError;
    int _outputMax;
    int _outputMin;
    int _lastOutput;
};
#endif /* PID_H_ */
