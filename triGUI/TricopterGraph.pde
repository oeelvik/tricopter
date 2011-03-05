class TricopterGraph {
  int _x;
  int _y;
  int _width;
  int _height;
  int _rotorD;

  int minValue = 0;
  int maxValue = 255;

  TricopterGraph(int x, int y, int width, int height) {
    _x = x;
    _y = y;
    _width = width;
    _height = height;

    _rotorD = _width / 5;

    this.drawEmpty();
  }

  void setLimits(int minVal, int maxVal) {
    this.minValue = minVal;
    this.maxValue = maxVal;
  }

  int getMinValue() {
    return this.minValue;
  }

  int getMaxValue() {
    return this.maxValue;
  }

  void drawEmpty() {
    stroke(255);
    fill(0);
    rect(_x, _y, _width, _height);

    fill(255);
    textAlign(CENTER);
    textFont(font, 14);
    text("Tricopter", _x + (_width/2), _y + 20 );

    textFont(font, 10);
    text("Rear", _x + _width / 2 + _rotorD / 2 + 20, _y + _height / 1.43 + 5 );
    text("Right", _x + _width / 1.25, _y + _height / 5 - _rotorD / 2 - 5 );
    text("Left", _x + _width / 5, _y + _height / 5 - _rotorD / 2 - 5 );

    textAlign(LEFT);

    stroke(25,25,100);
    strokeWeight(10);
    line(_x + _width / 2, _y + _height / 2.68, _x + _width / 2, _y + _height / 1.43);
    line(_x + _width / 2, _y + _height / 2.68, _x + _width / 1.25, _y + _height / 5);
    line(_x + _width / 2, _y + _height / 2.68, _x + _width / 5, _y + _height / 5);
    strokeWeight(1);
    
    stroke(0);
    fill(255);
    ellipse(_x + _width / 2, _y + _height / 1.43, _rotorD,  _rotorD);
    ellipse(_x + _width / 1.25, _y + _height / 5,  _rotorD,  _rotorD);
    ellipse(_x + _width / 5, _y + _height / 5,  _rotorD,  _rotorD);
  }

  void setLeftMotor(int value, int setPoint) {
    value = (int)map(value, minValue, maxValue, 0, _rotorD);
    setPoint = (int)constrain(map(setPoint, minValue, maxValue, 0, _rotorD), 0, _rotorD);

    fill(255);
    stroke(0);
    ellipse(_x + _width / 5, _y + _height / 5,  _rotorD,  _rotorD);

    noStroke();
    fill(255,0,0);
    ellipse(_x + _width / 5, _y + _height / 5,  value,  value);

    stroke(0,0,255);
    noFill();
    ellipse(_x + _width / 5, _y + _height / 5,  setPoint,  setPoint);
  }

  void setRightMotor(int value, int setPoint) {
    value = (int)map(value, minValue, maxValue, 0, _rotorD);
    setPoint = (int)constrain(map(setPoint, minValue, maxValue, 0, _rotorD), 0, _rotorD);

    fill(255);
    stroke(0);
    ellipse(_x + _width / 1.25, _y + _height / 5, _rotorD,  _rotorD);

    noStroke();
    fill(255,0,0);
    ellipse(_x + _width / 1.25, _y + _height / 5, value,  value);

    stroke(0,0,255);
    noFill();
    ellipse(_x + _width / 1.25, _y + _height / 5, setPoint,  setPoint);
  }

  void setRearMotor(int value, int setPoint) {
    value = (int)map(value, minValue, maxValue, 0, _rotorD);
    setPoint = (int)constrain(map(setPoint, minValue, maxValue, 0, _rotorD), 0, _rotorD);

    fill(255);
    stroke(0);
    ellipse(_x + _width / 2, _y + _height / 1.43, _rotorD,  _rotorD);

    noStroke();
    fill(255,0,0);
    ellipse(_x + _width / 2, _y + _height / 1.43, value,  value);

    stroke(0,0,255);
    noFill();
    ellipse(_x + _width / 2, _y + _height / 1.43, setPoint,  setPoint);
  }

  //TODO: fix this
  void setYaw(int value, int setPoint) {

    value = -(int)map(value, minValue, maxValue, -_width / 2, _width / 2);
    setPoint = -(int)map(setPoint, minValue, maxValue, -_width / 2, _width / 2);

    int x = _x + _width / 2;
    int y = (int)(_y + _height / 1.43 + _rotorD/2 + 5);


    noStroke();
    fill(0);
    rect(_x, y, _width, _height / 15 * 2 + 5 );

    fill(255,0,0);
    rect(x, y, value, _height / 15 );

    fill(0,0,255);
    rect(x, y + _height / 15 + 5, setPoint, _height / 15 );
    stroke(0);
  }

  void update(int left, int leftSet, int right, int rightSet, int rear, int rearSet, int yaw, int yawSet) {
    this.setLeftMotor(left, leftSet);
    this.setRightMotor(right, rightSet);
    this.setRearMotor(rear, rearSet);
    this.setYaw(yaw, yawSet);
  }
}
