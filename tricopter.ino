#include <SatelliteReceive.h>
#include <IMURazor.h>
#include <StopWatch.h> //TODO:remove after benchmark test

#include <Tricopter.h>



Tricopter tricopter;

void setup(){
  tricopter.init();
}

void loop(){
  tricopter.fastLoop();
}
