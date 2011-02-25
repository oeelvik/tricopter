import controlP5.*;

ControlP5 controlP5;

//TODO: use oop NO OOP ControlP5 needs reference to PApplet 

int sv_x;
int sv_y;
int sv_width;
int sv_height;

DropdownList sv_selectSerial;
DropdownList sv_selectSerialBaud;

Toggle sv_motorsEnable;
Toggle sv_PIDEnable;
Toggle sv_HKEnable;
Toggle sv_TriGUIEnable;

Slider sv_minThrottle;
Slider sv_minESC;

Slider sv_PIDSampleTime;
Slider sv_PIDKp;
Slider sv_PIDKi;
Slider sv_PIDKd;

void createSetupView(int x, int y, int width, int height){
  sv_x = x;
  sv_y = y;
  sv_width = width;
  sv_height = height;
  
  updateSetupView();
  
  controlP5 = new ControlP5(this);
  
  
  
  ToggleDisplay display = new ToggleDisplay();
  sv_HKEnable = controlP5.addToggle("enableHK",false,x+160,y+5,15,15);
  sv_HKEnable.setLabel("Happy Killmore");
  sv_HKEnable.setDisplay(display);
  sv_HKEnable.setColorBackground(color(255,0,0));
  sv_HKEnable.setColorActive(color(0,255,0));
  sv_HKEnable.setLabelVisible(false);
  
  sv_TriGUIEnable = controlP5.addToggle("enableTriGUI",true,x+160,y+25,15,15);
  sv_TriGUIEnable.setLabel("TriGUI data");
  sv_TriGUIEnable.setDisplay(display);
  sv_TriGUIEnable.setColorBackground(color(255,0,0));
  sv_TriGUIEnable.setColorActive(color(0,255,0));
  sv_TriGUIEnable.setLabelVisible(false);
  
  sv_motorsEnable = controlP5.addToggle("enableMotors",false,x+5,y+60,15,15);
  sv_motorsEnable.setLabel("Motors");
  sv_motorsEnable.setDisplay(display);
  sv_motorsEnable.setColorBackground(color(255,0,0));
  sv_motorsEnable.setColorActive(color(0,255,0));
  sv_motorsEnable.setLabelVisible(false);
  
  sv_PIDEnable = controlP5.addToggle("enablePID",true,x+5,y+120,15,15);
  sv_PIDEnable.setLabel("PID");
  sv_PIDEnable.setDisplay(display);
  sv_PIDEnable.setColorBackground(color(255,0,0));
  sv_PIDEnable.setColorActive(color(0,255,0));
  sv_PIDEnable.setLabelVisible(false);
  
  sv_minThrottle = controlP5.addSlider("setMinThrottle",0,1023,0,x+5,y+80,100,10);
  sv_minThrottle.setLabel("Minimum Throttle Value");
  
  sv_minESC = controlP5.addSlider("setMinESC",0,1024,0,x+5,y+95,100,10);
  sv_minESC.setLabel("ESC Arm Value");
  
  sv_PIDSampleTime = controlP5.addSlider("setPIDSampleTime",0,255,0,x+5,y+140,100,10);
  sv_PIDSampleTime.setLabel("PID Sample Time (millis)");
  
  sv_PIDKp = controlP5.addSlider("setPIDKp",0,255,0,x+5,y+155,100,10);
  sv_PIDKp.setLabel("PID proportional gain");
  
  sv_PIDKi = controlP5.addSlider("setPIDKi",0,255,0,x+5,y+170,100,10);
  sv_PIDKi.setLabel("PID integral gain");
  
  sv_PIDKd = controlP5.addSlider("setPIDKd",0,255,0,x+5,y+185,100,10);
  sv_PIDKd.setLabel("PID derivative gain");
  
  sv_createSerialUI(x,y);
}

void sv_createSerialUI(int x, int y){
  controlP5.addButton("Start Serial",0,x + 85,y + 20,70,20);
  controlP5.addButton("Update List",0,x + 5,y + 20,70,20);
  
  sv_selectSerial = controlP5.addDropdownList("Serial Port", x + 5, y + 5 + 10, 70, 120);
  sv_updateSelectSerial(); //TODO: call when Serial.list has changed or att timed intervals
  sv_selectSerialBaud = controlP5.addDropdownList("Baudrate", x + 85, y + 5 + 10, 70, 120);
  sv_selectSerialBaud.addItem("300",300);
  sv_selectSerialBaud.addItem("1200",1200);
  sv_selectSerialBaud.addItem("2400",2400);
  sv_selectSerialBaud.addItem("4800",4800);
  sv_selectSerialBaud.addItem("9600",9600);
  sv_selectSerialBaud.addItem("14400",14400);
  sv_selectSerialBaud.addItem("19200",19200);
  sv_selectSerialBaud.addItem("28800",28800);
  sv_selectSerialBaud.addItem("38400",38400);
  sv_selectSerialBaud.addItem("57600",57600);
  sv_selectSerialBaud.addItem("115200",115200);
}

void enableMotors(boolean theFlag) {
  gui.tricopter.enableMotors = theFlag;
}

void setMinThrottle(int val) {
  gui.tricopter.minThrottle = val;
}

void setMinESC(int val) {
  gui.tricopter.minESC = val;
}

void setPIDSampleTime(int val) {
  gui.tricopter.PIDSampleTime = val;
}

void setPIDKp(int val) {
  gui.tricopter.PIDKp = val;
}

void setPIDKi(int val) {
  gui.tricopter.PIDKi = val;
}

void setPIDKd(int val) {
  gui.tricopter.PIDKd = val;
}

void enableHK(boolean theFlag) {
  gui.tricopter.HKEnable = theFlag;
}

void enableTriGUI(boolean theFlag) {
  gui.tricopter.TriGUITransmitt = theFlag;
}

void enablePID(boolean theFlag) {
  gui.tricopter.PIDEnable = theFlag;
}

void updateSetupView(){
  
  stroke(255);
  fill(0);
  rect(sv_x, sv_y, sv_width, sv_height);
  
  if(sv_motorsEnable != null) sv_motorsEnable.setValue(gui.tricopter.enableMotors);
  if(sv_PIDEnable != null) sv_PIDEnable.setValue(gui.tricopter.PIDEnable);
  if(sv_HKEnable != null) sv_HKEnable.setValue(gui.tricopter.HKEnable);
  if(sv_TriGUIEnable != null) sv_TriGUIEnable.setValue(gui.tricopter.TriGUITransmitt);
  if(sv_minThrottle != null) sv_minThrottle.setValue(gui.tricopter.minThrottle);
  if(sv_minESC != null) sv_minESC.setValue(gui.tricopter.minESC);
  if(sv_PIDSampleTime != null) sv_PIDSampleTime.setValue(gui.tricopter.PIDSampleTime);
  if(sv_PIDKp != null) sv_PIDKp.setValue(gui.tricopter.PIDKp);
  if(sv_PIDKi != null) sv_PIDKi.setValue(gui.tricopter.PIDKi);
  if(sv_PIDKd != null) sv_PIDKd.setValue(gui.tricopter.PIDKd);
  
}

void sv_updateSelectSerial(){
  sv_selectSerial.clear();
  for(int i = 0; i < Serial.list().length; i++) sv_selectSerial.addItem(Serial.list()[i],0);
}

void controlEvent(ControlEvent theEvent) {
  // PulldownMenu is if type ControlGroup.
  // A controlEvent will be triggered from within the ControlGroup.
  // therefore you need to check the originator of the Event with
  // if (theEvent.isGroup())
  // to avoid an error message from controlP5.

  if (theEvent.isGroup()) {
    // check if the Event was triggered from a ControlGroup
    println(theEvent.group().value()+" from "+theEvent.group());
    
    
  } else if(theEvent.isController()) {
    println(theEvent.controller().value()+" from "+theEvent.controller());
    
    
    //Update Serial Port select
    if(theEvent.name() == "Update List"){
      sv_updateSelectSerial();
    }
    
    //Start Serial communication on buttonClick
    if(theEvent.name() == "Start Serial"){
      if(serial != null){
        serial.stop();
        serial = null;
      }
      
      
      String name = sv_selectSerial.stringValue();
      if(name == "Serial Port") {
        if(Serial.list().length > 0) name = Serial.list()[0];
      }
      
      if(name != "Serial Port"){
        int val = (int)sv_selectSerialBaud.value();
        if(val < 1) val = 115200; 
        print(val);
        serial = new Serial(this, name, val);
      }
    }
  }
}

class ToggleDisplay implements ControllerDisplay{
  void display(PApplet p, Controller c){
    Toggle t = (Toggle)c;
    if(t.getState()){
      noStroke();
      fill(t.getColor().getActive());
      p.rect(0,0,t.getWidth(),t.getHeight());
    }
    else {
      
      noStroke();
      fill(t.getColor().getBackground());
      p.rect(0,0,t.getWidth(),t.getHeight());
    }
    
    fill(255);
    textAlign(LEFT,CENTER);
    p.text(t.label(), t.getWidth() + 2, t.getHeight()/2);
    
  }
}
