#ifndef MIXER_H_
#define MIXER_H_

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "Motors.h"

class Mixer{
  public:
    Mixer();
    void init();
    void setArmESC(int val);
    void setIdleSpin(int val);
    void setMinThro(int val);
    void setYawRev(bool val);
    //TODO: remove or implement (deprecated)
    void setPins(int left, int right, int rear, int yaw);
    void setThrust(int throttle, int roll, int nick, int yaw );
    int getLeftThrust();
    int getRightThrust();
    int getRearThrust();
    int getYawPos();
  
    bool armed;
  private:

    Motors motors;
    
    int leftThrust;
    int rightThrust;
    int rearThrust;
    int yawPos;
    
    int armESC;
    int idleSpin;
    int minThro;
    bool yawRev;
    
};
#endif /* MIXER_H_ */
