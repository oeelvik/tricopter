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

Slider sv_Hover_PIDKp;
Slider sv_Hover_PIDKi;
Slider sv_Hover_PIDKd;

Slider sv_Acro_PIDKp;
Slider sv_Acro_PIDKi;
Slider sv_Acro_PIDKd;

Slider sv_Yaw_PIDKp;
Slider sv_Yaw_PIDKi;
Slider sv_Yaw_PIDKd;

void createSetupView(int x, int y, int width, int height){
  sv_x = x;
  sv_y = y;
  sv_width = width;
  sv_height = height;
  
  updateSetupView();
  
  controlP5 = new ControlP5(this);
  
  
  
  sv_HKEnable = controlP5.addToggle("enableHK",false,x+160,y+5,15,15);
  sv_HKEnable.setLabel("Happy Killmore");
  
  sv_TriGUIEnable = controlP5.addToggle("enableTriGUI",true,x+160,y+25,15,15);
  sv_TriGUIEnable.setLabel("TriGUI data");
  
  sv_motorsEnable = controlP5.addToggle("enableMotors",false,x+5,y+60,15,15);
  sv_motorsEnable.setLabel("Motors");
  
  sv_PIDEnable = controlP5.addToggle("enablePID",true,x+5,y+120,15,15);
  sv_PIDEnable.setLabel("PID");
  
  sv_minThrottle = controlP5.addSlider("setMinThrottle",0,255,0,x+5,y+80,100,10);
  sv_minThrottle.setLabel("Minimum Throttle Value");
  
  sv_minESC = controlP5.addSlider("setMinESC",0,255,0,x+5,y+95,100,10);
  sv_minESC.setLabel("ESC Arm Value");
  
  sv_Hover_PIDKp = controlP5.addSlider("setHoverPIDKp",0,255,0,x+5,y+155,100,10);
  sv_Hover_PIDKp.setLabel("H Kp");
  
  sv_Hover_PIDKi = controlP5.addSlider("setHoverPIDKi",0,255,0,x+5,y+170,100,10);
  sv_Hover_PIDKi.setLabel("H Ki");
  
  sv_Hover_PIDKd = controlP5.addSlider("setHoverPIDKd",0,255,0,x+5,y+185,100,10);
  sv_Hover_PIDKd.setLabel("H Kd");
  
  sv_Acro_PIDKp = controlP5.addSlider("setAcroPIDKp",0,255,0,x+135,y+155,100,10);
  sv_Acro_PIDKp.setLabel("A Kp");
  
  sv_Acro_PIDKi = controlP5.addSlider("setAcroPIDKi",0,255,0,x+135,y+170,100,10);
  sv_Acro_PIDKi.setLabel("A Ki");
  
  sv_Acro_PIDKd = controlP5.addSlider("setAcroPIDKd",0,255,0,x+135,y+185,100,10);
  sv_Acro_PIDKd.setLabel("A Kd");
  
  sv_Yaw_PIDKp = controlP5.addSlider("setYawPIDKp",0,255,0,x+265,y+155,100,10);
  sv_Yaw_PIDKp.setLabel("Y Kp");
  
  sv_Yaw_PIDKi = controlP5.addSlider("setYawPIDKi",0,255,0,x+265,y+170,100,10);
  sv_Yaw_PIDKi.setLabel("Y Ki");
  
  sv_Yaw_PIDKd = controlP5.addSlider("setYawPIDKd",0,255,0,x+265,y+185,100,10);
  sv_Yaw_PIDKd.setLabel("Y Kd");
  
  sv_createSerialUI(x,y);
  
  controlP5.addButton("Send Config",0,x + 325,y + 5,70,20);
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
  gui.tricopter.config.data[CV_TRICOPTER_ENABLE_BYTE] = 
    bitWrite(gui.tricopter.config.data[CV_TRICOPTER_ENABLE_BYTE]
             ,CV_MOTORS_ENABLE_BIT
             ,theFlag);
}

void setMinThrottle(int val) {
  gui.tricopter.config.data[CV_MIN_THRO_BYTE] = val;
}

void setMinESC(int val) {
  gui.tricopter.config.data[CV_MIN_ESC_BYTE] = val;
}

void setHoverPIDKp(int val) {
  gui.tricopter.config.data[CV_HOVER_PID_KP_BYTE] = val;
}

void setHoverPIDKi(int val) {
  gui.tricopter.config.data[CV_HOVER_PID_KI_BYTE] = val;
}

void setHoverPIDKd(int val) {
  gui.tricopter.config.data[CV_HOVER_PID_KD_BYTE] = val;
}

void setAcroPIDKp(int val) {
  gui.tricopter.config.data[CV_ACRO_PID_KP_BYTE] = val;
}

void setAcroPIDKi(int val) {
  gui.tricopter.config.data[CV_ACRO_PID_KI_BYTE] = val;
}

void setAcroPIDKd(int val) {
  gui.tricopter.config.data[CV_ACRO_PID_KD_BYTE] = val;
}

void setYawPIDKp(int val) {
  gui.tricopter.config.data[CV_YAW_PID_KP_BYTE] = val;
}

void setYawPIDKi(int val) {
  gui.tricopter.config.data[CV_YAW_PID_KI_BYTE] = val;
}

void setYawPIDKd(int val) {
  gui.tricopter.config.data[CV_YAW_PID_KD_BYTE] = val;
}

void enableHK(boolean theFlag) {
  gui.tricopter.config.data[CV_TRICOPTER_ENABLE_BYTE] = 
    bitWrite(gui.tricopter.config.data[CV_TRICOPTER_ENABLE_BYTE]
             ,CV_HK_ENABLE_BIT
             ,theFlag);
}

void enableTriGUI(boolean theFlag) {
  gui.tricopter.config.data[CV_TRICOPTER_ENABLE_BYTE] = 
    bitWrite(gui.tricopter.config.data[CV_TRICOPTER_ENABLE_BYTE]
             ,CV_TRIGUI_ENABLE_BIT
             ,theFlag);
}

void enablePID(boolean theFlag) {
  gui.tricopter.config.data[CV_TRICOPTER_ENABLE_BYTE] = 
    bitWrite(gui.tricopter.config.data[CV_TRICOPTER_ENABLE_BYTE]
             ,CV_PID_ENABLE_BIT
             ,theFlag);
}

void updateSetupView(){
  
  stroke(255);
  fill(0);
  rect(sv_x, sv_y, sv_width, sv_height);
  
  if(gui != null && !gui.tricopter.config.loadedToGUI){
    
    gui.tricopter.config.loadedToGUI = true;
    if(sv_motorsEnable != null) sv_motorsEnable.setValue(bitRead(gui.tricopter.config.data[CV_TRICOPTER_ENABLE_BYTE], CV_MOTORS_ENABLE_BIT));
    if(sv_PIDEnable != null) sv_PIDEnable.setValue(bitRead(gui.tricopter.config.data[CV_TRICOPTER_ENABLE_BYTE], CV_PID_ENABLE_BIT));
    if(sv_HKEnable != null) sv_HKEnable.setValue(bitRead(gui.tricopter.config.data[CV_TRICOPTER_ENABLE_BYTE], CV_HK_ENABLE_BIT));
    if(sv_TriGUIEnable != null) sv_TriGUIEnable.setValue(bitRead(gui.tricopter.config.data[CV_TRICOPTER_ENABLE_BYTE], CV_TRIGUI_ENABLE_BIT));
    if(sv_minThrottle != null) sv_minThrottle.setValue(gui.tricopter.config.data[CV_MIN_THRO_BYTE]);
    if(sv_minESC != null) sv_minESC.setValue(gui.tricopter.config.data[CV_MIN_ESC_BYTE]);
    if(sv_Hover_PIDKp != null) sv_Hover_PIDKp.setValue(gui.tricopter.config.data[CV_HOVER_PID_KP_BYTE]);
    if(sv_Hover_PIDKi != null) sv_Hover_PIDKi.setValue(gui.tricopter.config.data[CV_HOVER_PID_KI_BYTE]);
    if(sv_Hover_PIDKd != null) sv_Hover_PIDKd.setValue(gui.tricopter.config.data[CV_HOVER_PID_KD_BYTE]);
    if(sv_Acro_PIDKp != null) sv_Acro_PIDKp.setValue(gui.tricopter.config.data[CV_ACRO_PID_KP_BYTE]);
    if(sv_Acro_PIDKi != null) sv_Acro_PIDKi.setValue(gui.tricopter.config.data[CV_ACRO_PID_KI_BYTE]);
    if(sv_Acro_PIDKd != null) sv_Acro_PIDKd.setValue(gui.tricopter.config.data[CV_ACRO_PID_KD_BYTE]);
    if(sv_Yaw_PIDKp != null) sv_Yaw_PIDKp.setValue(gui.tricopter.config.data[CV_YAW_PID_KP_BYTE]);
    if(sv_Yaw_PIDKi != null) sv_Yaw_PIDKi.setValue(gui.tricopter.config.data[CV_YAW_PID_KI_BYTE]);
    if(sv_Yaw_PIDKd != null) sv_Yaw_PIDKd.setValue(gui.tricopter.config.data[CV_YAW_PID_KD_BYTE]);
  }
}

void sv_updateSelectSerial(){
  sv_selectSerial.clear();
  for(int i = 0; i < Serial.list().length; i++) sv_selectSerial.addItem(Serial.list()[i],0);
}

void controlEvent(ControlEvent theEvent) {
  
  if(gui != null)
  // PulldownMenu is if type ControlGroup.
  // A controlEvent will be triggered from within the ControlGroup.
  // therefore you need to check the originator of the Event with
  // if (theEvent.isGroup())
  // to avoid an error message from controlP5.

  if (theEvent.isGroup()) {
    // check if the Event was triggered from a ControlGroup
    println(theEvent.group().value()+" from "+theEvent.group());
    
    
  } else if(theEvent.isController()) {
    //println(theEvent.controller().value()+" from "+theEvent.controller());
    
    
    //Update Serial Port select
    if(theEvent.name() == "Update List"){
      sv_updateSelectSerial();
    }
    
    //Update Serial Port select
    if(theEvent.name() == "Send Config"){
      gui.serialHandler.send(6);
    }
    
    //Start Serial communication on buttonClick
    if(theEvent.name() == "Start Serial"){
      if(serial != null){
        serial.stop();
        serial = null;
      }
      
      
      String name = Serial.list()[(int)sv_selectSerial.getValue()];
      if(name == "Serial Port") {
        if(Serial.list().length > 0) name = Serial.list()[0];
      }
      
      if(name != "Serial Port"){
        int val = (int)sv_selectSerialBaud.value();
        if(val < 1) val = 115200;
        
        serial = new Serial(this, name, val);
        gui.serialHandler.setSerial(serial);
      }
    }
  }
}

