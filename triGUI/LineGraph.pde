class LineGraphView {
  String label;
  int _x;
  int _y;
  int _width;
  int _height;
  int _speed;

  int _pos = 1;

  ArrayList graphs = new ArrayList();

  LineGraphView(String label, int x, int y, int width, int height, int speed) {
    this.label = label;
    this._x = x;
    this._y = y;
    this._width = width;
    this._height = height;
    this._speed = speed;

    this.reset();
  }

  void addGraph(LineGraph graph) {
    graphs.add(graph);
  }

  void reset() {
    stroke(255);
    fill(0);
    rect( _x, _y, _width, _height);
    
    fill(255);
    textFont(font, 14);
    text(this.label, this._x + 5, this._y + 16);

    textFont(font, 10);
    for (int i = 0; i < graphs.size(); i++) {
      LineGraph g = (LineGraph)graphs.get(i);
      
      fill(g.getColor());
      text(g.getLabel(), this._x + 5, this._y + 16 + 14 * (i + 1));
    }
  }

  void plot() {

    if(this._pos + this._speed > this._width - 2) {
      this._pos = 1;
      this.reset();
    }

    for (int i = 0; i < graphs.size(); i++) {
      LineGraph g = (LineGraph)graphs.get(i);
      stroke(g.getColor());

      int value = (int)map(constrain(g.getValue(), g.getMinValue(), g.getMaxValue()), g.getMinValue(), g.getMaxValue(), 0, this._height - 2);
      int lastVal = (int)map(constrain(g.getLastValue(), g.getMinValue(), g.getMaxValue()), g.getMinValue(), g.getMaxValue(), 0, this._height - 2);
      
      line(this._x + this._pos + this._speed, this._y + this._height - 1 - value, this._x + this._pos, this._y + this._height - 1 - lastVal);
    }

    this._pos += this._speed;
  }
}

class LineGraph {
  color _color = color(255);
  String _label = "";

  int minValue = 0;
  int maxValue = 255;

  int _value = 0;
  int _lastVal = 0;

  LineGraph(String label, color col) {
    this._label = label;
    this._color = col;
  }

  color getColor() {
    return this._color;
  }

  String getLabel() {
    return this._label;
  }

  void setValue(int value) {
    this._value = value;
  }

  int getValue() {
    return this._value;
  }

  int getLastValue() {
    int r = this._lastVal;
    this._lastVal = this._value;
    return r;
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
}
