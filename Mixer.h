#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <Servo.h>


#ifndef MIXER_H_
#define MIXER_H_
class Mixer{
  public:
    Mixer();
    void setMinESC(int val);
    void setMinThro(int val);
    void setYawRev(bool val);
    void setMotorsEnabled(bool val);
    void setPins(int left, int right, int rear, int yaw);
    void setThrust(int throttle, int roll, int nick, int yaw );
    int getLeftThrust();
    int getRightThrust();
    int getRearThrust();
    int getYawPos();
  
  private:
    Servo yawServo;
    Servo rearMotor;
    Servo rightMotor;
    Servo leftMotor;
    
    int leftThrust;
    int rightThrust;
    int rearThrust;
    int yawPos;
    
    int minESC;
    int minThro;
    bool motorsEnabled;
    bool yawRev;
    
};
#endif /* MIXER_H_ */
