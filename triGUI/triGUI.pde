import processing.serial.*;

int leftCol = 300;

int barHight = 200;
int plotHeight = (screen.height - 4)/4;
int r[] = {0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255};
int g[] = {0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0};
int b[] = {255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0};
int lastVal[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int minValue = 0;
int maxValue = 255;
float plotScale = (float) plotHeight / (maxValue - minValue);
float barScale = (float) barHight / (maxValue - minValue);

int x = leftCol;

//Receive byte count
int byteCount = 255;

Serial serial;
PFont font;

TricopterGraph tri;
LineGraphView g1;
LineGraph p1;
LineGraph p2;

SerialComHandler serialHandler;

void setup() {
  size(screen.width,screen.height);
  font = createFont("Verdana", 30, true);
  println(Serial.list());
  if(Serial.list().length > 0) serial = new Serial(this, Serial.list()[0], 115200);
  else println("No serial port avilable");
  
  
  
  
  resetPlot();
  resetBarGraphs();
  
  //tri = new TricopterGraph(0, barHight+2, leftCol-1, leftCol-1);
  tri = new TricopterGraph(50, barHight+2, 200, 200);
  
  p1 = new LineGraph("Plot 1", color(255,0,255));
  p2 = new LineGraph("Plot 2", color(0,255, 0));
  
  g1 = new LineGraphView(0, 500, 300, 200);
  g1.addGraph(p1);
  g1.addGraph(p2);
  p2.setValue(100);
  
  
  
  SerialSerializable t = new test();
  SerialSerializable t2 = new test();
  /*
  
  print(t.v1);
  print("\t");
  print(t.v2);
  print("\t");
  println(t.v3);
  
  print(t2.v1);
  print("\t");
  print(t2.v2);
  print("\t");
  println(t2.v3);
  byte[] s = t.toSerialStreem();
  t2.parseSerialStreem(s);
  
  print(t.v1);
  print("\t");
  print(t.v2);
  print("\t");
  println(t.v3);
  
  print(t2.v1);
  print("\t");
  print(t2.v2);
  print("\t");
  println(t2.v3);*/
  
  
  serialHandler = new SerialComHandler();
  serialHandler.addObject(byte(0), t);
  
  test t1 = (test) t;
  print(t1.v1);
  print("\t");
  print(t1.v2);
  print("\t");
  println(t1.v3);
  
  byte[] data = {'>', '>', '>', byte(0), 10, 20, 30, 40, 50, 60, 70, 80, 90, '*', '*', '*'};
  for(int i = 0; i < data.length; i++){
    serialHandler.receive(data[i]);
  }
  
  byte[] data2 = {'-', '-', '-', byte(0), 90, 80, 70, 60, 50, 40, 30, 20, 10, '*', '*', '*'};
  for(int i = 0; i < data2.length; i++){
    serialHandler.receive(data2[i]);
  }
  
  print(t1.v1);
  print("\t");
  print(t1.v2);
  print("\t");
  println(t1.v3);
}

void serialEvent(Serial myPort) {
  
  // read a byte from the serial port:
  int inByte = myPort.read();
  if(inByte == '\n' || byteCount > 19){
    byteCount = 0;
    
    println();
    x++;
    if(x > screen.width){
      x = leftCol;
      resetPlot();
    }
  } else if (byteCount < 8){ //Bar graph
    if(byteCount == 0){
      resetBarGraphs(); //reset at first barGraph byte
      tri.update(
        lastVal[5], lastVal[1] + lastVal[3] / 2 - lastVal[2] / 4
        ,lastVal[7], lastVal[1] - lastVal[3] / 2 + (maxValue - minValue) / 2 - lastVal[2] / 4
        ,lastVal[6], lastVal[1] + lastVal[2] / 2
        ,lastVal[4], lastVal[0]);
    }
    print(inByte);
    print("\t");
    int colorIndex = 0;
    if(byteCount > 3) colorIndex = 2;
    
    barGraph(inByte, byteCount + 1, r[colorIndex], g[colorIndex], b[colorIndex]);
    
    lastVal[byteCount] = inByte;
    byteCount++;
  } else{
    int plotNo = byteCount -8;
    plot(lastVal[byteCount], inByte, ((byteCount-8)/3) + 1, r[plotNo], g[plotNo], b[plotNo]);
    lastVal[byteCount] = inByte;
    
    
    
    byteCount++;
  }
}


void draw() {
  p1.setValue(millis()/100);
  g1.plot();
  /*
  resetBarGraphs();
  barGraph(125, 1, 0, 255, 0);
  barGraph(millis()/50, 2, 0, 255, 0);
  barGraph(10, 3, 0, 255, 0);
  barGraph(255 - millis()/50, 4, 0, 255, 0);
  
  //Update plot
  plot(125,125, 1, 0, 0, 255);
  plot(130,130, 1, 0, 255, 0);
  plot(127,127, 2, 255, 0, 0);
  x++;
  if(x > screen.width){
    x = leftCol;
    resetPlot();
  }*/
}

void barGraph(int value, int graph, int r, int g, int b){
  stroke(0);
  fill(r,g,b);
  int v = value;
  if(v < minValue) v = minValue;
  if(v > maxValue) v = maxValue;
  
  int thisHeight = (int)((v - minValue) * barScale);
  rect((10 * graph) + (20 * (graph - 1)), barHight - thisHeight, 20, thisHeight );
}

void resetBarGraphs(){
  stroke(0);
  fill(0);
  rect(0,0,leftCol-1,barHight+1);
}

void plot(int lastVal, int value, int graph, int r, int g, int b){
  stroke(r,g,b);
  int v = value;
  if(v < minValue) v = minValue;
  if(v > maxValue) v = maxValue;
  
  if(lastVal < minValue) lastVal = minValue;
  if(lastVal > maxValue) lastVal = maxValue;
  
  
  
  //TODO: draw line from prev poin to this point
  line(x, (int)((plotHeight * graph) - ((v - minValue) * plotScale)), x+1, (int)((plotHeight * graph) - ((lastVal - minValue) * plotScale)));
}

void resetPlot(){
  stroke(0);
  fill(0);
  rect(leftCol,0,screen.width - leftCol,screen.height);
  stroke(255);
  line(leftCol, 0, leftCol, screen.height);
  line(leftCol, plotHeight + 1, screen.width, plotHeight + 1);
  line(leftCol, (plotHeight + 1)*2, screen.width, (plotHeight + 1)*2);
  line(leftCol, (plotHeight + 1)*3, screen.width, (plotHeight + 1)*3);
}

//TODO: create library graph
class LineGraphView{
  int _x;
  int _y;
  int _width;
  int _height;
  
  int _pos = 1;
  
  ArrayList graphs = new ArrayList();
  
  LineGraphView(int x, int y, int width, int height){
    this._x = x;
    this._y = y;
    this._width = width;
    this._height = height;
    
    this.reset();
  }
  
  void addGraph(LineGraph graph){
    graphs.add(graph);
    reset();
  }
  
  void reset(){
    stroke(255);
    fill(0);
    rect( _x, _y, _width, _height);
    
    for (int i = 0; i < graphs.size(); i++){
      LineGraph g = (LineGraph)graphs.get(i);
      
      fill(g.getColor());
      textFont(font, 12);
      text(g.getLabel(), this._x + 5, this._y + 14 * (i + 1));
    }
  }
  
  void plot(){
    
    if(this._pos > this._width - 2) {
      this._pos = 1;
      this.reset();
    }
    
    for (int i = 0; i < graphs.size(); i++){
      LineGraph g = (LineGraph)graphs.get(i);
      stroke(g.getColor());
      
      int value = (int)map(constrain(g.getValue(), g.getMinValue(), g.getMaxValue()), g.getMinValue(), g.getMaxValue(), 0, this._height - 2);
      int lastVal = (int)map(constrain(g.getLastValue(), g.getMinValue(), g.getMaxValue()), g.getMinValue(), g.getMaxValue(), 0, this._height - 2);
      
      
      line(this._x + this._pos, this._y + this._height - 1 - value, this._x + this._pos + 1, this._y + this._height - 1 - lastVal);
    }
    
    this._pos++;
  }
}

class LineGraph{
  color _color = color(255);
  String _label = "";
  
  int minValue = 0;
  int maxValue = 255;
  
  int _value = 0;
  int _lastVal = 0;
  
  LineGraph(String label, color col){
    this._label = label;
    this._color = col;
  }
  
  color getColor(){
    return this._color;
  }
  
  String getLabel(){
    return this._label;
  }
  
  void setValue(int value){
    this._value = value;
  }
  
  int getValue(){
    this._lastVal = this._value;
    return this._value;
  }
  
  int getLastValue(){
    return this._lastVal;
  }
  
  void setLimits(int minVal, int maxVal){
    this.minValue = minVal;
    this.maxValue = maxVal;
    
  }
  
  int getMinValue(){
    return this.maxValue;
  }
  
  int getMaxValue(){
    return this.maxValue;
  }
}



class TricopterGraph{
  int _x;
  int _y;
  int _width;
  int _height;
  int _rotorD;
  
  TricopterGraph(int x, int y, int width, int height){
    _x = x;
    _y = y;
    _width = width;
    _height = height;
    
    _rotorD = _width / 5;
    
    this.drawEmpty();
  }
  
  void drawEmpty(){
    stroke(255);
    fill(255);
    rect(_x, _y, _width, _height);
    
    fill(50,100,150);
    textFont(font, 18);
    text("Tricopter", _x + (_width/2) - 30, _y + 20 );
    
    textFont(font, 12);
    text("Rear", _x + _width / 2 - 10 + _rotorD / 2 + 20, _y + _height / 1.43 + 5 );
    text("Right", _x + _width / 1.25 - 10, _y + _height / 5 - _rotorD / 2 - 5 );
    text("Left", _x + _width / 5 - 10, _y + _height / 5 - _rotorD / 2 - 5 );
    
    
    stroke(0);
    strokeWeight(10);
    line(_x + _width / 2, _y + _height / 2.68, _x + _width / 2, _y + _height / 1.43);
    line(_x + _width / 2, _y + _height / 2.68, _x + _width / 1.25, _y + _height / 5);
    line(_x + _width / 2, _y + _height / 2.68, _x + _width / 5, _y + _height / 5);
    strokeWeight(1);
    
    fill(255);
    ellipse(_x + _width / 2, _y + _height / 1.43, _rotorD,  _rotorD);
    ellipse(_x + _width / 1.25, _y + _height / 5,  _rotorD,  _rotorD);
    ellipse(_x + _width / 5, _y + _height / 5,  _rotorD,  _rotorD);
  }
  
  void setLeftMotor(int value, int setPoint){
    value = (int)map(value, minValue, maxValue, 0, _rotorD);
    setPoint = (int)constrain(map(setPoint, minValue, maxValue, 0, _rotorD), 0, _rotorD);
    
    fill(255);
    stroke(0);
    ellipse(_x + _width / 5, _y + _height / 5,  _rotorD,  _rotorD);
    
    noStroke();
    fill(r[2], g[2], b[2]);
    ellipse(_x + _width / 5, _y + _height / 5,  value,  value);
    
    stroke(r[0], g[0], b[0]);
    noFill();
    ellipse(_x + _width / 5, _y + _height / 5,  setPoint,  setPoint);
  }
  
  void setRightMotor(int value, int setPoint){
    value = (int)map(value, minValue, maxValue, 0, _rotorD);
    setPoint = (int)constrain(map(setPoint, minValue, maxValue, 0, _rotorD), 0, _rotorD);
    
    fill(255);
    stroke(0);
    ellipse(_x + _width / 1.25, _y + _height / 5, _rotorD,  _rotorD);
    
    noStroke();
    fill(r[2], g[2], b[2]);
    ellipse(_x + _width / 1.25, _y + _height / 5, value,  value);
    
    stroke(r[0], g[0], b[0]);
    noFill();
    ellipse(_x + _width / 1.25, _y + _height / 5, setPoint,  setPoint);
  }
  
  void setRearMotor(int value, int setPoint){
    value = (int)map(value, minValue, maxValue, 0, _rotorD);
    setPoint = (int)constrain(map(setPoint, minValue, maxValue, 0, _rotorD), 0, _rotorD);
    
    fill(255);
    stroke(0);
    ellipse(_x + _width / 2, _y + _height / 1.43, _rotorD,  _rotorD);
    
    noStroke();
    fill(r[2], g[2], b[2]);
    ellipse(_x + _width / 2, _y + _height / 1.43, value,  value);
    
    stroke(r[0], g[0], b[0]);
    noFill();
    ellipse(_x + _width / 2, _y + _height / 1.43, setPoint,  setPoint);
  }
  
  //TODO: fix this
  void setYaw(int value, int setPoint){
    
    value = (int)map(value, minValue, maxValue, -_width / 2, _width / 2);
    setPoint = (int)map(setPoint, minValue, maxValue, -_width / 2, _width / 2);
    
    int x = _x + _width / 2;
    int y = (int)(_y + _height / 1.43 + _rotorD/2 + 5);
    
    
    noStroke();
    fill(255);
    rect(_x, y, _width, _height / 15 * 2 + 5 );
    
    fill(r[2], g[2], b[2]);
    rect(x, y, value, _height / 15 );
    
    fill(r[0], g[0], b[0]);
    rect(x, y + _height / 15 + 5, setPoint, _height / 15 );
    stroke(0);
  }
  
  void update(int left, int leftSet, int right, int rightSet, int rear, int rearSet, int yaw, int yawSet){
    this.setLeftMotor(left, leftSet);
    this.setRightMotor(right, rightSet);
    this.setRearMotor(rear, rearSet);
    this.setYaw(yaw, yawSet);
  }
}



class Tricopter implements SerialSerializable{
  public RCReceiver receiver;
  public IMU imu;
  
  Tricopter(){
    
  }
  
  byte[] toSerialStreem(){
    byte[] r = new byte[3];
    /*r[0] = byte (constrain(map(v1, 0, 1023, 0, 255), 0, 255));
    r[1] = byte (constrain(map(v2, 0, 1023, 0, 255), 0, 255));
    r[2] = byte (constrain(map(v3, 0, 1023, 0, 255), 0, 255));*/
    return r;
  }
  
  void parseSerialStreem(byte[] data){
    /*v1 = (int) map(data[0], 0, 255, 0, 1023);
    v2 = (int) map(data[1], 0, 255, 0, 1023);
    v3 = (int) map(data[2], 0, 255, 0, 1023);*/
  }
  
  class RCReceiver implements SerialSerializable{
    public byte thro = 0;
    public byte aile = 0;
    public byte elev = 0;
    public byte rudd = 0;
    public byte gear = 0;
    public byte flap = 0;
    
    public byte thro_rev = 0;
    public byte aile_rev = 0;
    public byte elev_rev = 0;
    public byte rudd_rev = 0;
    public byte gear_rev = 0;
    public byte flap_rev = 0;
    
    byte[] data = new byte[12];
    
    
    byte[] toSerialStreem(){
      data[0] = thro;
      data[1] = aile;
      data[2] = elev;
      data[3] = rudd;
      data[4] = gear;
      data[5] = flap;
      
      data[6] = thro_rev;
      data[7] = aile_rev;
      data[8] = elev_rev;
      data[9] = rudd_rev;
      data[10] = gear_rev;
      data[11] = flap_rev;
      return data;
    }
    
    void parseSerialStreem(byte[] data){
      thro = data[0];
      aile = data[0];
      elev = data[0];
      rudd = data[0];
      gear = data[0];
      flap = data[0];
      
      thro_rev = data[0];
      aile_rev = data[0];
      elev_rev = data[0];
      rudd_rev = data[0];
      gear_rev = data[0];
      flap_rev = data[0];
    }
  }
  
  class IMU implements SerialSerializable{
    byte[] toSerialStreem(){
      byte[] r = new byte[3];
      /*r[0] = byte (constrain(map(v1, 0, 1023, 0, 255), 0, 255));
      r[1] = byte (constrain(map(v2, 0, 1023, 0, 255), 0, 255));
      r[2] = byte (constrain(map(v3, 0, 1023, 0, 255), 0, 255));*/
      return r;
    }
    
    void parseSerialStreem(byte[] data){
      /*v1 = (int) map(data[0], 0, 255, 0, 1023);
      v2 = (int) map(data[1], 0, 255, 0, 1023);
      v3 = (int) map(data[2], 0, 255, 0, 1023);*/
    }
    
  }
}

class triGUI{
  
  int x;
  int y;
  int width;
  int height;
  
  Tricopter tricopter;
  
  TricopterGraph tricopterGraph;
  
  LineGraphView rollGraph;
  LineGraph rollSetPointGraph = new LineGraph("Set Point", color(0,0,255));
  LineGraph rollMeasurementGraph = new LineGraph("Measurement", color(0,255,0));
  LineGraph rollThrustGraph = new LineGraph("Thrust", color(255,0,0));
  
  triGUI(int x, int y, int width, int height){
    this.x = x;
    this.y = y;
    this.width = width;
    this.height = height;
    
    int graph_height = this.height/3;
    
    rollGraph = new LineGraphView(this.x + 300, this.y, this.width - 300, graph_height);
    rollGraph.addGraph(rollSetPointGraph);
    rollGraph.addGraph(rollMeasurementGraph);
    rollGraph.addGraph(rollThrustGraph);
  }
  
  void update(){
    rollSetPointGraph.setValue(this.tricopter.receiver.aile);
    rollMeasurementGraph.setValue(0);
    rollThrustGraph.setValue(0);
    
    rollGraph.plot();
  }
  
  
}



interface SerialSerializable{
  byte[] toSerialStreem();
  void parseSerialStreem(byte[] data);
}

class SerialComHandler{
  final int BYTE_LIMIT = 50;
  
  HashMap serializableObjects = new HashMap();
  
  byte CMD;
  byte[] data = new byte[BYTE_LIMIT];
  
  boolean inMessage = false;
  int messageByteCount = 0;
  int postFixCount = 0;
  
  void addObject(byte CMD, SerialSerializable object){
    serializableObjects.put(CMD, object);
  }
  
  void receive(byte inByte){
    messageByteCount++;
    
    if(!inMessage  && inByte != '>'){ //Not this protocol
      messageByteCount = 0;
    } else if(inByte == '>' && messageByteCount == 3){ //prefix received
      inMessage = true;
      postFixCount = 0;
      this.CMD = 0;
    } else if (inMessage){
      
      if(inByte == '*') postFixCount++;
      else postFixCount = 0;
      if(postFixCount == 3 || messageByteCount - 5 >= BYTE_LIMIT) { //All bytes in message reseived
        inMessage = false;
        callReceiver(this.CMD);
      }
      
      if(messageByteCount == 4) this.CMD = inByte;
      else data[messageByteCount - 5] = inByte;
    }
  }
  
  void send(byte CMD){
    if(this.serializableObjects.containsKey(CMD)){
      //TODO: Send by serial
      //((SerialSerializable)this.serializableObjects.get(CMD)).parseSerialStreem(this.data);
    }
  }
  
  void callReceiver(byte CMD){
    
      println(CMD);
    if(this.serializableObjects.containsKey(CMD)){
      ((SerialSerializable)this.serializableObjects.get(CMD)).parseSerialStreem(this.data);
    }
  }
}
