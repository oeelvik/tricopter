import processing.serial.*;

Serial serial = null;
SerialComHandler serialHandler;

boolean currentSetupReceived = false;
int setupRequestTime = 0;

TriGUI gui;

PFont font = createFont("Verdana", 30, true);

void setup() {
  size(displayWidth, displayHeight);

  serialHandler = new SerialComHandler();
  
  //serial = new Serial(this, "COM3", 115200);
  
  gui = new TriGUI( 0, 0, displayWidth, displayHeight, serialHandler);
}

void serialEvent(Serial myPort) {
  serialHandler.receive(myPort.read());
}

void draw() {
  if(setupRequestTime + 1000 < millis() && serial != null && !gui.tricopter.config.currentSetupReceived) {
    gui.serialHandler.send(5); //Request tricopter current config
    setupRequestTime = millis();
  }
  
  updateSetupView();
  gui.update();
}

class TriGUI {

  int x;
  int y;
  int width;
  int height;
  
  public SerialComHandler serialHandler = new SerialComHandler();

  public Tricopter tricopter = new Tricopter();
  
  MessageView messages;

  TricopterGraph tricopterGraph;
  
  BarGraphView receiverBars;
  BarGraph throBar = new BarGraph("Thro", color(0,255,0));
  BarGraph aileBar = new BarGraph("Aile", color(0,255,0));
  BarGraph elevBar = new BarGraph("Elev", color(0,255,0));
  BarGraph ruddBar = new BarGraph("Rudd", color(0,255,0));
  BarGraph gearBar = new BarGraph("Gear", color(0,255,0));
  BarGraph flapBar = new BarGraph("Flap", color(0,255,0));

  LineGraphView rollGraph;
  LineGraph rollSetPointGraph = new LineGraph("Set Point", color(0,0,255));
  LineGraph rollMeasurementGraph = new LineGraph("Measurement", color(0,255,0));
  LineGraph rollThrustGraph = new LineGraph("Thrust", color(255,0,0));
  
  LineGraphView nickGraph;
  LineGraph nickSetPointGraph = new LineGraph("Set Point", color(0,0,255));
  LineGraph nickMeasurementGraph = new LineGraph("Measurement", color(0,255,0));
  LineGraph nickThrustGraph = new LineGraph("Thrust", color(255,0,0));
  
  LineGraphView yawGraph;
  LineGraph yawSetPointGraph = new LineGraph("Set Point", color(0,0,255));
  LineGraph yawMeasurementGraph = new LineGraph("Measurement", color(0,255,0));
  LineGraph yawThrustGraph = new LineGraph("Thrust", color(255,0,0));
  

  TriGUI(int x, int y, int width, int height, SerialComHandler serialHandler) {
    this.x = x;
    this.y = y;
    this.width = width;
    this.height = height;
    
    this.messages = new MessageView(0,200,400,200,14);
    
    this.serialHandler = serialHandler;
    this.serialHandler.addObject(1, this.messages); //Data
    this.serialHandler.addObject(2, this.tricopter); //Data
    this.serialHandler.addObject(3, this.tricopter.receiver); //Data
    this.serialHandler.addObject(4, this.tricopter.imu); //Data
    this.serialHandler.addObject(5, this.tricopter.config); //request config
    this.serialHandler.addObject(6, this.tricopter.config); //set config
    
    
    createSetupView(0, 0, 400, 200);
    
    tricopterGraph = new TricopterGraph(400, 0, 200, 200);
    
    receiverBars = new BarGraphView("Receiver", 400, 200, 200, 200);
    receiverBars.addGraph(throBar);
    receiverBars.addGraph(aileBar);
    receiverBars.addGraph(elevBar);
    receiverBars.addGraph(ruddBar);
    receiverBars.addGraph(gearBar);
    receiverBars.addGraph(flapBar);
    receiverBars.reset();

    
    int graph_height = this.height/3;

    rollGraph = new LineGraphView("Roll", this.x + 600, this.y, this.width - 600, graph_height, 4);
    rollGraph.addGraph(rollSetPointGraph);
    rollGraph.addGraph(rollMeasurementGraph);
    rollGraph.addGraph(rollThrustGraph);
    rollGraph.reset();
    
    nickGraph = new LineGraphView("Nick", this.x + 600, this.y + graph_height, this.width - 600, graph_height, 4);
    nickGraph.addGraph(nickSetPointGraph);
    nickGraph.addGraph(nickMeasurementGraph);
    nickGraph.addGraph(nickThrustGraph);
    nickGraph.reset();
    
    yawGraph = new LineGraphView("Yaw", this.x + 600, this.y + graph_height * 2, this.width - 600, graph_height, 4);
    yawGraph.addGraph(yawSetPointGraph);
    yawGraph.addGraph(yawMeasurementGraph);
    yawGraph.addGraph(yawThrustGraph);
    yawGraph.reset();
  }

  void update() {
    if(this.messages.isUpdated) this.messages.update();
    
    if(!this.tricopter.isUpdated) return;
    this.tricopter.isUpdated = false;
    tricopterGraph.update(this.tricopter.leftMotor, this.tricopter.receiver.thro + this.tricopter.receiver.aile / 2 - this.tricopter.receiver.elev / 4
                        ,this.tricopter.rightMotor, this.tricopter.receiver.thro - (this.tricopter.receiver.aile - (tricopterGraph.getMaxValue() - tricopterGraph.getMinValue())) / 2 - this.tricopter.receiver.elev / 4
                        ,this.tricopter.rearMotor, this.tricopter.receiver.thro + this.tricopter.receiver.elev / 2 - 30
                        ,this.tricopter.yawServo, this.tricopter.receiver.rudd);
    
    throBar.setValue(this.tricopter.receiver.thro);
    aileBar.setValue(this.tricopter.receiver.aile);
    elevBar.setValue(this.tricopter.receiver.elev);
    ruddBar.setValue(this.tricopter.receiver.rudd);
    gearBar.setValue(this.tricopter.receiver.gear);
    flapBar.setValue(this.tricopter.receiver.flap);
    
    receiverBars.update();
    
    rollSetPointGraph.setValue(this.tricopter.receiver.aile);
    if(this.tricopter.mode == 0x00) rollMeasurementGraph.setValue(this.tricopter.imu.roll);
    else rollMeasurementGraph.setValue(this.tricopter.imu.gRoll);
    rollThrustGraph.setValue(this.tricopter.PIDRoll);
    rollGraph.plot();
    
    nickSetPointGraph.setValue(this.tricopter.receiver.elev);
    if(this.tricopter.mode == 0x00) nickMeasurementGraph.setValue(this.tricopter.imu.nick);
    else nickMeasurementGraph.setValue(this.tricopter.imu.gNick);
    nickThrustGraph.setValue(this.tricopter.PIDNick);
    nickGraph.plot();
    
    yawSetPointGraph.setValue(this.tricopter.receiver.rudd);
    if(this.tricopter.mode == 0x00) yawMeasurementGraph.setValue(this.tricopter.imu.yaw);
    else nickMeasurementGraph.setValue(this.tricopter.imu.gYaw);
    yawThrustGraph.setValue(this.tricopter.PIDYaw);
    yawGraph.plot();
    
  }
}





