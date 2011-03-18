//TODO: create library graph
class BarGraphView {
  String label;
  int x;
  int y;
  int width;
  int height;

  ArrayList graphs = new ArrayList();

  BarGraphView(String label, int x, int y, int width, int height) {
    this.label = label;
    this.x = x;
    this.y = y;
    this.width = width;
    this.height = height;

    this.reset();
  }

  void addGraph(BarGraph graph) {
    graphs.add(graph);
  }

  void reset() {
    stroke(255);
    fill(0);
    rect( this.x, this.y, this.width, this.height);

    fill(255);
    textFont(font, 14);
    text(this.label, this.x + 5, this.y + 16);

    if(graphs.size() > 0) {
      int graphWidth = constrain((this.width - 5) / graphs.size()  - 5, 1, 40);


      textFont(font, 10);
      textAlign(CENTER);
      for (int i = 0; i < graphs.size(); i++) {
        BarGraph g = (BarGraph)graphs.get(i);

        fill(255);
        text(g.getLabel(), this.x + 5 + graphWidth / 2 + (graphWidth + 5) * i, this.y + this.height -5);
      }
      textAlign(LEFT);
    }
  }

  void update() {

    if(graphs.size() > 0) {
      int graphWidth = constrain((this.width - 5) / graphs.size()  - 5, 1, 40);
      
      noStroke();
      for (int i = 0; i < graphs.size(); i++) {
        BarGraph g = (BarGraph)graphs.get(i);

        int value = (int)map(constrain(g.getValue(), g.getMinValue(), g.getMaxValue()), g.getMinValue(), g.getMaxValue(), 0, this.height - 35);
        
        fill(0);
        rect(this.x + 5 + (graphWidth + 5) * i, this.y + this.height - 15, graphWidth, - this.height + 35);
        
        
        fill(g.getColor());
        rect(this.x + 5 + (graphWidth + 5) * i, this.y + this.height - 15, graphWidth, -value);
      }
      stroke(0);
    }
  }
}

class BarGraph {
  color _color = color(255);
  String _label = "";

  int minValue = 0;
  int maxValue = 255;

  int _value = 0;
  int _lastVal = 0;

  BarGraph(String label, color col) {
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

